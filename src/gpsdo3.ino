/*
 * Control Systemm for a GPS Disciplined Oscillator
 *
 * Copyright 2023-2024 Glen Overby
 * 
 * This program is free software; you can redistribute it and/or modify it
 * under the terms of the GNU General Public License version 2 as published
 * by the Free Software Foundation
 * 
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 * 
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, see <http://www.gnu.org/licenses/>.
 *
 */

// Arduino Pin  Processor Port  Function
// D13          PA17            Blue LED
// D12          PA19            1PPS from GPS
// D7           PA21            10MHz from oscillator
// D3           PA09            PWM output for VControl
//
// D2           PA14            Status LED - on when stable
// D5           PA15            Power LED - on when thinking
//
// D7 is difficult to move because TC4's input is on that pin
//
// LEDs:
//  D1  green     Lock
//  D2  red       Ready
//  D3  orange    Heartbeat
//  D4  red/green GPS status: red no lock, green lock, off no data


// Oscillator Configuration
// P_ADJUST  P controller adjustment factor for PWM DAC (not used)
// PD_ADJUST P controller adjustment factor for DAC
// If_ADJUST I controller adjustment factor
// PD_INIT   Initialization value for DAC
//            These values came from running the GPSDO during development
// USE_HOT  Use the OSC HOT input

#define CTI

#ifdef CTI  // CTi OS5SC25  square wave output oscillator
#define P_ADJUST  266
#define PD_ADJUST 10
#define If_ADJUST 16 / 100
#define PD_INIT -37	// SN#2
// #define ON_STABLE	// SN#1 has a green LED; SN#2 has a red LED
#endif
#ifdef OCXO13410  // Isotemp OCXO134-10
#define P_ADJUST  266
#define PD_ADJUST 25
#define If_ADJUST 24
#define PD_INIT 0
#define USE_HOT
#endif
#ifdef IQD2202   // IQD IQXT-220-2
#define P_ADJUST  266
#define PD_ADJUST 25
#define If_ADJUST 24
#define PD_INIT 0
#endif
#ifdef FOX801   // FOX 801
#define P_ADJUST  2
#define PD_ADJUST 3
#define If_ADJUST 1
#define PD_INIT  19      // initial value for P
#endif
#ifdef CW_T604   // Connor Williams (never got it fully working)
#define P_ADJUST  2
#define PD_ADJUST 1
#define If_ADJUST 1
#define PD_INIT  0      // initial value for P
#endif

#define HEARTBEAT_LED 10   // changes every second to make me happy.
                          // Shows that 1PPS is coming from GPS
#define GPS_LED_A   8     // Red/Green LED. Off=no data, R=no lock, G=2d/3d lock
#define GPS_LED_B   9
#define LOCK_LED   13   // oscillator is locked. Also blue LED
#define READY_LED  11   // general ready.
                        // Oscillator HOT signal would contribute to this.

#define HOT_PIN   2     // Oscillator HOT input (use pull-up)


uint32_t last_count, current_count;
volatile int flag = 0;
uint dacv = 0; // Hardware DAC (10 bits)
int settle=3, Isettle=0;  // Settle timer for startup, etc.
int Pd=0, Id=0;   // I for hardware DAC
double Ia = 0;    // I controller average
float Iea = 0;    // I controller error average
int stable = 0;
int outliers = 0; // Counter for ignoring outliers
int locked = 0;   // Flag for identifying when "locked".
                  // Lock is when no controller is making changes.
int debug = 0;
int Ienable = 1;
int ADJUST = 0;

// These would be locals, but the debug print needs access to them
float Ierror;
int32_t padjust=0, iadjust=0, dadjust=0;

int gpslock = 0;  // non-zero when GPS Lock (from $GPGSA)
int gpsdata = 0;  // last time GPS serial data was seen (counter)
int gpsdebug = 0; // Report data from GPS receiver
int gprmc = 0;    // set true to report $GPRMC. Can be used to set computer's time.

int hb = 0;       // Heartbeat LED flag

ulong nexthb;     // Time of next "software" heartbeat -- using millis()
int pps = 0;      // 1 pulse-per-second from GPS detected

// Wait for synchronization of registers between the clock domains
static __inline__ void syncTCC(Tcc* TCCx) __attribute__((always_inline, unused));
static void syncTCC(Tcc* TCCx) {
  while (TCCx->SYNCBUSY.reg & TCC_SYNCBUSY_MASK);
}


void setup()
{
    // change here to use main clock or input pin
    bool use_clk_pin = true; //false ;

    SerialUSB.begin(115200);
    SerialUSB.println("Hello World");

    Serial1.begin(9600);
    //Serial1.begin(4800);
    Serial.setTimeout(10);

    pinMode(HEARTBEAT_LED, OUTPUT);
    pinMode(LOCK_LED, OUTPUT);
    pinMode(READY_LED, OUTPUT);
    pinMode(GPS_LED_A, OUTPUT);
    pinMode(GPS_LED_B, OUTPUT);
    digitalWrite(HEARTBEAT_LED, LOW); // Turn on all LEDs
    digitalWrite(LOCK_LED, LOW);
    digitalWrite(READY_LED, LOW);
    digitalWrite(GPS_LED_A, LOW);   // Red
    digitalWrite(GPS_LED_B, HIGH);
    pinMode(HOT_PIN, INPUT_PULLUP);

    nexthb = millis() + 2000;

    // Configure hardware DAC
    analogWriteResolution(10); // Set analog out resolution to max, 10-bits
    analogReadResolution(12); // Set analog input resolution to max, 12-bits
    // pinMode(A0, OUTPUT);  BAD - causes DAC to not go over 2.2v
    analogWrite(A0, 0);

    // Configuration Stolen from the internet at 
    // https://forum.arduino.cc/t/running-tc-with-an-external-clock-and-externally-triggered-capture-zero-m0/495066
    //
    // setup main clocks first
    REG_PM_APBAMASK |= PM_APBAMASK_GCLK ;
    REG_PM_APBBMASK |= PM_APBBMASK_PORT ;
    REG_PM_APBCMASK |= PM_APBCMASK_EVSYS | PM_APBCMASK_TC4 | PM_APBCMASK_TC5 ;

    //
    // Configure 32-bit counter TC4/TC5 with it's clock input from PA21(D7)
    // and capture trigger from PA19(D12). Get an interrupt on capture.
    //

    // send generic GCLK0 to the EIC peripheral. This allows using the
    // generic clock for things.
    REG_GCLK_CLKCTRL = GCLK_CLKCTRL_CLKEN | GCLK_CLKCTRL_GEN_GCLK0 | GCLK_CLKCTRL_ID_EIC ;
    while(GCLK->STATUS.bit.SYNCBUSY) ;
    // and to EVSYS peripheral
    REG_GCLK_CLKCTRL = GCLK_CLKCTRL_CLKEN | GCLK_CLKCTRL_GEN_GCLK0 | GCLK_CLKCTRL_ID_EVSYS_0 ;
    while(GCLK->STATUS.bit.SYNCBUSY) ;

    // Send pin PA19 (D12) to EIC (line 1). This is the 1PPS signal from GPS
    //PORT->Group[PORTA].PMUX[17 >> 1].reg |= PORT_PMUX_PMUXO_A ;
    //PORT->Group[PORTA].PINCFG[17].reg |= PORT_PINCFG_PMUXEN ;

    // Enable the port multiplexer on digital pin D12 (port pin PA19)
    PORT->Group[g_APinDescription[12].ulPort].PINCFG[g_APinDescription[12].ulPin].bit.PMUXEN = 1;
    // Feed GCLK5 to TC4/TC5
    // Set up the pin as an EIC (interrupt) peripheral on D12
    PORT->Group[g_APinDescription[12].ulPort].PMUX[g_APinDescription[12].ulPin >> 1].reg |= PORT_PMUX_PMUXO_A;

    // enable ext3 interrupt on rising edge to event system
    REG_EIC_EVCTRL |= EIC_EVCTRL_EXTINTEO3;
    REG_EIC_CONFIG0 |= EIC_CONFIG_SENSE3_RISE;
    // Disable interrupts on interupt 3. We want it as a trigger, not an int
    EIC->INTENCLR.reg = EIC_INTENCLR_EXTINT3; 
    REG_EIC_CTRL |= EIC_CTRL_ENABLE;
    while (EIC->STATUS.bit.SYNCBUSY);

    // direct ext3 event to TC4
    REG_EVSYS_USER = EVSYS_USER_CHANNEL(1) | EVSYS_USER_USER(EVSYS_ID_USER_TC4_EVU) ;
    REG_EVSYS_CHANNEL = EVSYS_CHANNEL_EDGSEL_RISING_EDGE | EVSYS_CHANNEL_PATH_RESYNCHRONIZED 
                        | EVSYS_CHANNEL_EVGEN(EVSYS_ID_GEN_EIC_EXTINT_3) | EVSYS_CHANNEL_CHANNEL(0) ;

    // Source GCLK from PA21 (D7) using gclkio 5
    PORT->Group[PORTA].PMUX[21 / 2].reg = PORT_PMUX_PMUXO_H ;
    PORT->Group[PORTA].PINCFG[21].reg = PORT_PINCFG_PMUXEN ;

    REG_GCLK_GENDIV = GCLK_GENDIV_ID(5) ; // GCLK5: no div

    // Debugging allows use of the main clock instead of the input clock
    if(use_clk_pin) {
        // Feed from GCLK_IO to GCLK5
        REG_GCLK_GENCTRL = GCLK_GENCTRL_GENEN | GCLK_GENCTRL_RUNSTDBY | GCLK_GENCTRL_SRC_GCLKIN  | GCLK_GENCTRL_ID(5) ;
        while (GCLK->STATUS.bit.SYNCBUSY);
    } else {
        // clock from main clock
        REG_GCLK_GENCTRL = GCLK_GENCTRL_GENEN | GCLK_GENCTRL_RUNSTDBY | GCLK_GENCTRL_SRC_DFLL48M | GCLK_GENCTRL_ID(5) ;
        while (GCLK->STATUS.bit.SYNCBUSY);
    }

    // Feed GCLK5 to TC4/TC5
    REG_GCLK_CLKCTRL = GCLK_CLKCTRL_CLKEN | GCLK_CLKCTRL_GEN_GCLK5 | GCLK_CLKCTRL_ID_TC4_TC5 ;
    while (GCLK->STATUS.bit.SYNCBUSY);
    
    // Enable TC4 event input
    REG_TC4_EVCTRL |= TC_EVCTRL_TCEI ;

    // set capture on TC4/5 channel 0
    REG_TC4_READREQ = TC_READREQ_RREQ | TC_READREQ_ADDR(0x06) ;
    while (TC4->COUNT32.STATUS.bit.SYNCBUSY) ;
    REG_TC4_CTRLC |= TC_CTRLC_CPTEN0 ;
    while (TC4->COUNT32.STATUS.bit.SYNCBUSY) ;

    // enable timer TC4 interrupt for Capture events
    NVIC_SetPriority(TC4_IRQn, 0) ;
    NVIC_EnableIRQ(TC4_IRQn) ;
    REG_TC4_INTENSET = TC_INTENSET_MC0 ;

    // no prescaler, 32 bit mode (uses TC4 & TC5)
    REG_TC4_CTRLA |= TC_CTRLA_PRESCALER_DIV1 | TC_CTRLA_MODE_COUNT32 | TC_CTRLA_ENABLE ;
    while (TC4->COUNT16.STATUS.bit.SYNCBUSY) ;

    // Read counter register continuously
    REG_TC4_READREQ = TC_READREQ_RCONT | TC_READREQ_ADDR(0x10); // COUNT offset 0x10

    Pd = PD_INIT;
    adjust(1, 0, 0, 0);
    SerialUSB.println("config done");
}

void TC4_Handler()
{
    uint32_t count ;
  
    // Check for match counter 0 (MC0) interrupt
    // read automatically clears flag      
    if (TC4->COUNT16.INTFLAG.bit.MC0) {
        REG_TC4_READREQ = TC_READREQ_RREQ | TC_READREQ_ADDR(0x18);
        while (TC4->COUNT32.STATUS.bit.SYNCBUSY);
        count = REG_TC4_COUNT32_CC0;

        current_count = count;
        flag = 1;
    }
}


void serial_status()
{
  if (SerialUSB.available() > 0) {
    // read the incoming byte:
    int incomingByte;
    incomingByte = SerialUSB.read();
    if (incomingByte == 'd') {  // Control general debug
      debug++;
    } else if (incomingByte == 'D') {
      debug = 0;
    } else if (incomingByte == 's') { // Report status
      volatile uint32_t count = REG_TC4_COUNT32_COUNT ;
      SerialUSB.print("Count ");
      SerialUSB.print(current_count);
      SerialUSB.print(" ");
      SerialUSB.print(count);
      SerialUSB.print(" settle ");
      SerialUSB.print(settle);
      SerialUSB.print(" Ia ");
      SerialUSB.print(Ia);
      SerialUSB.print(" Iea ");
      SerialUSB.print(Iea);
      SerialUSB.print(" dacv ");
      SerialUSB.print(dacv);
      SerialUSB.print(" stable ");
      SerialUSB.print(stable);
      SerialUSB.print(" locked ");
      SerialUSB.print(locked);
      SerialUSB.print(" GPSlock ");
      SerialUSB.print(gpslock);
      SerialUSB.print(" GPSdata ");
      SerialUSB.print(gpsdata);
      SerialUSB.print(" PPS ");
      SerialUSB.print(pps);
      SerialUSB.println("");
    } else if (incomingByte == 'S') { // Report more status
      SerialUSB.print("HOT ");
      SerialUSB.print(digitalRead(HOT_PIN));
      SerialUSB.print(" int-flag ");  // Interrupt flag
      SerialUSB.print(flag);
      //SerialUSB.print(" test pin ");
      //SerialUSB.print(digitalRead(4));
      SerialUSB.println("");
    } else if (incomingByte == 'G') { // Control GPS data debug
      gpsdebug = 0;
    } else if (incomingByte == 'g') {
      gpsdebug++;
    } else if (incomingByte == 'T') { // Control printing $GPRMC lines
      gprmc = 0;
    } else if (incomingByte == 't') {
      gprmc = 1;
    } else if (incomingByte == '8') { // Serial1 speed = 4800
      Serial1.begin(4800);
    } else if (incomingByte == '9') { // Serial1 speed = 9600
      Serial1.begin(9600);
    } else if (incomingByte == 'a') { 
      float value = analogRead(A1);
      float voltage = value * 3.3 / 4096.0;
      SerialUSB.print(value);
      SerialUSB.print(" ");
      SerialUSB.println(voltage); // Print the voltage.
    } else if (incomingByte == 'Q') { // Set specific DAC values
      analogWrite(A0, 1023);
    } else if (incomingByte == 'A') {
      analogWrite(A0, 512);
    } else if (incomingByte == 'Z') {
      analogWrite(A0, 0);
    } else if (incomingByte == 'W') {
      analogWrite(A0, 768);
    } else if (incomingByte == 'X') {
      analogWrite(A0, 256);
    } else if (incomingByte == 'R') {
      Pd += 10;
      ADJUST = 1;
    } else if (incomingByte == 'V') {
      Pd -= 10;
      ADJUST = 1;
    } else if (incomingByte == 'I') { // Control I operations
      Ienable = 1;
    } else if (incomingByte == 'i') {
      Ienable = 0;
    } else if (incomingByte == 'O') { // Turn off actions
      settle = 1000000;
    } else if (incomingByte == 'o') {
      settle = 1;
    } else if (incomingByte == 'R') { // Reset
      dacv=1;
      Pd = Id = stable = 0;
      ADJUST = 1;
    } else if (incomingByte == '0') { // Test of GPS data
      Serial1.println("Test 1");
      if (Serial1.available() > 0) {
        SerialUSB.println("1: serial available");
      }
    } else if (incomingByte == ' ') { // Hello?
      SerialUSB.println("-");
    }
  }
}

// Monitor the GPS serial data
// Set a valid data flag (counter actually) when there is data
// Look at GPS data to see if it has a valid lock
int monitor_gps_serial()
{
  // $GPGSA,A,3,,,,,,16,18,,22,24,,,3.6,2.1,2.2*3C
  //          ^ Fix: 1=No fix 2=2d, 3=3d
  // $GPRMC,022615.00,A,LLLL.94152,N,lllll.79113,W,0.082,,280816,,,A*63
  //   I use this for setting computer time
  if (Serial1.available() > 0) {
    char input[100], fix;
    int length, newlock;
    gpsdata = 5;  // time out in 5 seconds
    //input = Serial1.readStringUntil('\n');
    length = Serial1.readBytesUntil('\n',input, 99);
    input[length] = '\0';   // strip \n
    input[length-1] = '\0'; // strip \r
    if (gpsdebug > 2) {
      SerialUSB.print("> ");
      SerialUSB.println(input);
    }
    if (!strncmp("$GPGSA", input, 6)) {
      fix = input[9];
      newlock = fix - '0';
      if (newlock == 1) {
        newlock = 0;
      }
      if (newlock != gpslock) {
        gpslock = newlock;
        SerialUSB.print("$GPGSA = ");
        SerialUSB.println(gpslock);
      }
      if (gpsdebug) {
        SerialUSB.print('=');
        SerialUSB.print(input);
        SerialUSB.print(" ");
        SerialUSB.print(newlock);
        SerialUSB.println("");
      } else if (gprmc) {
        SerialUSB.print(input);
      }
    } else if (gprmc && !strncmp("$GPRMC", input, 6)) {
      // Pass GPRMC lines through. That way a computer on the USB port can
      // set it's clock from the GPS
      SerialUSB.print(input);
    }
  }
}

// Set GPS LED based on the GPS status
// Off = no data
// Red = no lock
// Green = lock
// This is called every second.
void gps_status()
{
  if (gpsdata) {
    gpsdata--;  // count down the data received timer
    if (gpsdata == 0)
      SerialUSB.println("Lost GPS data"); // Reports only when first lost
  }
  if (gpsdata == 0) {
    // Turn off LED
    digitalWrite(GPS_LED_A, LOW);
    digitalWrite(GPS_LED_B, LOW);
  } else if (gpslock) {
    // Turn LED Green
    digitalWrite(GPS_LED_A, HIGH);
    digitalWrite(GPS_LED_B, LOW);
  } else {
    // Turn LED Red
    digitalWrite(GPS_LED_A, LOW);
    digitalWrite(GPS_LED_B, HIGH);
  }
}

void lock_status()
{
  // blink lock LED when stable but not locked
  //digitalWrite(LOCK_LED, locked? LOW::HIGH);

  if (locked) {
    digitalWrite(LOCK_LED, LOW);
  } else {
    if (stable) {
      digitalWrite(LOCK_LED, hb? HIGH:LOW);
    } else {
      digitalWrite(LOCK_LED, HIGH);
    }
  }
}

void ready_status()
{
#ifdef USE_HOT
  int hot, on;

  hot = digitalRead(HOT_PIN);
  on = hot && ready;
  digitalWrite(READY_LED, on? LOW:HIGH);
#else
#ifdef ON_STABLE
  digitalWrite(READY_LED, stable? LOW:HIGH);  // ON when stable
#else
  digitalWrite(READY_LED, stable? HIGH:LOW);  // OFF when stable (RED LED)
#endif
#endif
}

void heartbeat()
{
  // Called once per second when the GPS 1PPS is running
  // Toggle heartbeat LED
  hb = !hb;
  digitalWrite(HEARTBEAT_LED, hb? HIGH:LOW);
  //if (debug > 1) {
  //  SerialUSB.print("Heartbeat ");
  //  SerialUSB.println(hb);
  //}
}

int count(uint32_t *delta)
{
  int skip = 0;

  // Request count, synchronize with counter
  //REG_TC4_READREQ = TC_READREQ_RREQ | TC_READREQ_ADDR(0x10) ; // COUNT offset 0x10
  //while (TC4->COUNT32.STATUS.bit.SYNCBUSY) ;
  volatile uint32_t count = REG_TC4_COUNT32_COUNT ;
  //while (TC4->COUNT32.STATUS.bit.SYNCBUSY) ;

  if (current_count > last_count) {
    *delta = current_count - last_count;
  } else {
    // Counter wrapped.  The count is always high, so throw it away.
    *delta = ((uint32_t)2^32 - last_count) + current_count;
    if (debug) {
      SerialUSB.print("Wrap ");
      SerialUSB.print(last_count);
      SerialUSB.print(" ");
      SerialUSB.print(((uint32_t)2^32-last_count));
      SerialUSB.print(" ");
      SerialUSB.print(current_count);
      SerialUSB.print(" = ");
      SerialUSB.print(*delta);
      SerialUSB.println(" ");
    }
    skip=1;
  }

  last_count = current_count;

  return skip;
}

int check_outliers(int error)
{
  int skip = 0;
  if (stable && (error < -10 || error > 10)) {  // Bad sample - throw away.
    outliers++;
    SerialUSB.print(" Outlier ");
    SerialUSB.print(outliers);
    SerialUSB.print(" ");
    skip = 1;
    if (outliers > 10) {
      outliers = 0;
      stable = 0;
      skip = 0;
      SerialUSB.print("Too many outliers ");
    }
  } else {
    outliers = 0;
  }

  return skip;
}

// P controller
int Pcontroller(int error)
{
  int skip = 0;

  // The initial hunt should stop when the error is zero. After that,
  // Use an error band to prevent moving on GPS jitter
  if (!skip && ((stable && (error < -1 || error > 1)) || (!stable && error != 0)))  {
    //if (!skip && (error < -2 || error > 2))  // << only error band
    padjust = error * PD_ADJUST;
    Pd += padjust;
    SerialUSB.print(" P Adjust ");
    SerialUSB.print(padjust);
    SerialUSB.print(" ");

    // Reset I controller values
    Ia = 0;
    Iea = 0;
    Id = 0;
    skip = 1; // skip I controller
    stable = 0;
    locked = 0;
  } 

  return skip;
}

  // I controller
int Icontroller(uint32_t delta)
{
  int report = 0;
  int32_t diff;

  // Keep a moving average of the error over several seconds.
  // By keeping the average of the error, a float can be used instead of a double.
  // 24 bits is needed to hold a value of 10,000,000 and the FP internals results in
  // no bits being available for the fraction. Thus, a double is required.
  //
  // I'm not observing a significant difference in averaging over 64 seconds vs 16 seconds
  // a off-by-1 jitter changes the error by 0.04 instead of 0.08.  I don't think this
  // allows tightening up the tolerance (+-10, below).
  diff = delta - 10000000;
  //Ia = ((Ia * 7.0)+diff) / 8.0;
  Iea = ((Iea * 15.0)+diff) / 16.0;
  //Iea = ((Iea * 63.0)+diff) / 64.0;

  //Ierror = round((Ia - 10000000) * 100);
  Ierror = round((Iea) * 100);
  if (Ierror < -10 || Ierror > 10) {
    // +-5 was too tight of a tolerance on a FOX 801 
    float adjustf;
    adjustf = Ierror * If_ADJUST / 100;
    iadjust = -(int)adjustf;
    if (iadjust == 0) {
      iadjust = (adjustf<0) ? 1 : -1;
    }

    SerialUSB.print(" I AdjustF ");
    SerialUSB.print(Ierror);
    SerialUSB.print(" ");
    SerialUSB.print(iadjust);
    if (Isettle)
      Isettle--;
    if (Ienable && Isettle==0) {
      // Isettle is a settling time between adjustments. This gives the
      // I averaging time for the adjustment to take affect.  This prevents
      // overshooting or, worse, oscillating.
      // Alternatively, resetting the error accumulator will also give it
      // time to take affect.
      Id += iadjust;
      //Ia = 10000000;  // reset error accumulator
      Isettle = 4;
      locked = 0;
    } else if (iadjust != 0) {
      SerialUSB.print("(skipped) ");
    }
    SerialUSB.println("");
  } else {
    Isettle = 0;
    locked = 1;   // I isn't making changes, so it's locked
  }
}

int check_stable(int stable, int32_t error)
{
  // Set stable flag when the error is zero.
  // Stable flag is cleared in the P controller
  if (!stable && !(error < -1 || error > 1)) {
    stable = 1;
  } 

  return stable;
}

void info(int report, uint32_t delta, int32_t error)
{
  if (report) {
    SerialUSB.print("Count ");
    SerialUSB.print(current_count);
    SerialUSB.print(" Delta ");
    SerialUSB.print(delta);
    SerialUSB.print(" Error ");
    SerialUSB.print(error);
    SerialUSB.print(" P ");
    SerialUSB.print(Pd);
    SerialUSB.print(" I ");
    SerialUSB.print(Id);
    SerialUSB.print(" Ia ");
    SerialUSB.print(Ia);
    SerialUSB.print(" Iea ");
    SerialUSB.print(Iea);
    SerialUSB.print(" Ie ");
    SerialUSB.print(Ierror);
    SerialUSB.print(" dacv ");
    SerialUSB.print(dacv);
    SerialUSB.print(" stable ");
    SerialUSB.print(stable);
    SerialUSB.print(" GPSlock ");
    SerialUSB.print(gpslock);
    SerialUSB.print(" GPSdata ");
    SerialUSB.print(gpsdata);
    SerialUSB.println("");
    report = 0;
  }
}

void broken()
{
  if (dacv > 1024) { // something is broken!
    dacv = 512;
    Ia = 0;
    Iea = 0;
    Pd = 0;
    Id = 0;
    SerialUSB.println(" RESET");
  }
}

void adjust(int padjust, int iadjust, int stable, int skip)
{
  dacv = 512 + Pd + Id;

  if (padjust | iadjust | ADJUST) {
    analogWrite(A0, dacv);
    ADJUST = 0;
  }
}


void loop()
{
  uint32_t delta;
  int32_t error;
  int skip = 0, report = 0;

  padjust = 0;
  iadjust = 0;

  serial_status();
  monitor_gps_serial(); 

  // check 1pps interrupt flag
  if (!flag) {
    if (millis() > nexthb) {
      // 1PPS from GPS is not being received.
      // Run LED status
      if (!gpslock) {
        locked=0;
      }
      gps_status();
      lock_status();
      ready_status();
      heartbeat();
      nexthb = millis() + 2000;
      pps = 0;
    }
    return;
  }
  flag = 0; // Clear
  nexthb = millis() + 2000;
  pps = 1;

  if ((skip = count(&delta)) > 0) {
  }

  // Settle counter allows skipping a sample to allow the oscillator to settle,
  // e.g. on start-up or after making a large adjustment
  if (settle > 0) {
    skip = 1;
    settle--;
  }

  if (skip && debug) {
    SerialUSB.print("Skip ");
    report = 1;
  }

  // Don't operate if there isn't GPS lock.
  // Require a sample after getting lock before operating.
  // The NEO 7 GPS I have doesn't produce 1PPS when it doesn't have lock.
  // (this is located after the above skip check to avoid setting report=1)
  if (!gpslock) {
    skip = 1;
    settle=1;
    locked=0;
  }

  error = 10000000 - delta;

  if (!skip) {
    if ((skip = check_outliers(error)) > 0) {
      report = 1;
    }
  }

  if (!skip) {
    if ((skip = Pcontroller(error)) > 0) {
      report = 1;
    }
  }

  if (!skip) {
    stable = check_stable(stable, error);
  }

  if (!skip && stable) {
    Icontroller(delta);
  }

  info(padjust | iadjust | debug | report, delta, error);

  broken();

  adjust(padjust, iadjust, stable, skip);

  gps_status();
  lock_status();
  ready_status();
  heartbeat();
}
// vim: tabstop=8 expandtab shiftwidth=2 softtabstop=2
// vi: tabstop=8 expandtab shiftwidth=2 softtabstop=2
