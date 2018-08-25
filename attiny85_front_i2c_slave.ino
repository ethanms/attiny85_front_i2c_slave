#include <TinyWireS.h>

#include <avr/sleep.h>
#include <avr/wdt.h>

/*
 * Output connected to the ESP8266 RST pin, should be pulled LOW to reset ESP
 */
#define RST_OUT         1     // Output connected to Wemos 3.3V regulator EN

/*
 * Input connected to reed switch, LOW is closed
 */
#define SW_IN           3     // Input from switch (closes to ground)

/*
 * Input connected to Photocell
 */
#define PC_IN           4     // Input from photocell (ADC)

#define I2C_SLAVE_ADDRESS 0x4 // ATTiny's address as an I2C slave

#ifndef TWI_RX_BUFFER_SIZE
#define TWI_RX_BUFFER_SIZE ( 16 )
#endif

#ifndef cbi
#define cbi(sfr, bit) (_SFR_BYTE(sfr) &= ~_BV(bit))
#endif
#ifndef sbi
#define sbi(sfr, bit) (_SFR_BYTE(sfr) |= _BV(bit))
#endif

#define             DEBOUNCE_MS           150                       // Debounce Time
#define             DEFAULT_LIGHT_LOW     200                       // Below this level the lights are "off"
#define             DEFAULT_LIGHT_HIGH    400                       // Above this level the lights are "on"
#define             DEFAULT_VOLT_LOW      3300                      // Below this level the battery is "low"
#define             DEFAULT_VOLT_HYST     500                       // This level added to low level will constitute leaving the low condition
#define             DEFAULT_HEARTBEAT_S   3600                      // Heartbeat every 3600s aka every hour
#define             DEFAULT_LIGHT_CHK_S   120                       // Check photoeye every 120s aka every other minute
#define             ESP_LOW_RESET_MS      100                       // ms to hold ESP8266 in RESET to wake it up


bool                espActive             = false;                  // True is the ESP is set active, when true this device is an I2C Slave

volatile byte       reg_position          = 0x00;

volatile uint8_t i2c_reg [] =
{
  0x08,                       // 0x00 - Status/State, b0 door sw, b1 lightsOn, b2 heartbeat, b3 state change, b4 low batt 
  0x00,                       // 0x01 - temperatureC Low
  0x00,                       // 0x02 - temperatureC High
  0x00,                       // 0x03 - voltage Low
  0x00,                       // 0x04 - voltage High
  0x00,                       // 0x05 - sleepPeriodCounter Low
  0x00,                       // 0x06 - sleepPeriodCounter High
  DEFAULT_LIGHT_CHK_S & 0xFF, // 0x07 - lightCheckFrequency Low
  DEFAULT_LIGHT_CHK_S >> 8,   // 0x08 - lightCheckFrequency High
  DEFAULT_HEARTBEAT_S & 0xFF, // 0x09 - heartbeatFrequency Low
  DEFAULT_HEARTBEAT_S >> 8,   // 0x0A - heartbeatFrequency High
  0x00,                       // 0x0B - lightLevel Low
  0x00,                       // 0x0C - lightLevel High
  DEFAULT_VOLT_LOW & 0xFF,    // 0x0D - voltLowThreshold Low
  DEFAULT_VOLT_LOW >> 8,      // 0x0E - voltLowThreshold High
  DEFAULT_LIGHT_LOW & 0xFF,   // 0x0F - lightLowThreshold Low
  DEFAULT_LIGHT_LOW >> 8,     // 0x10 - lightLowThreshold High
  DEFAULT_LIGHT_HIGH & 0xFF,  // 0x11 - lightHighThreshold Low
  DEFAULT_LIGHT_HIGH >> 8     // 0x12 - lightHighThreshold High
};

const byte          reg_size              = sizeof(i2c_reg);

enum i2cIndex {
  STATE,              // 0x00 Status/State, b0 door sw, b1 lightsOn, b2 heartbeat, b3 state change, b4 low batt
  TEMP_LOW,           // 0x01
  TEMP_HIGH,          // 0x02
  VOLT_LOW,           // 0x03
  VOLT_HIGH,          // 0x04
  SLEEP_LOW,          // 0x05
  SLEEP_HIGH,         // 0x06
  LIGHT_CHK_LOW,      // 0x07
  LIGHT_CHK_HIGH,     // 0x08
  HEART_LOW,          // 0x09
  HEART_HIGH,         // 0x0A
  LIGHT_LOW,          // 0x0B
  LIGHT_HIGH,         // 0x0C
  VOLT_TH_LOW,        // 0x0D
  VOLT_TH_HIGH,       // 0x0E
  LIGHT_LOW_TH_LOW,   // 0x0F
  LIGHT_LOW_TH_HIGH,  // 0x10
  LIGHT_HIGH_TH_LOW,  // 0x11
  LIGHT_HIGH_TH_HIGH  // 0x12
};

enum stateIndex {
  DOOR_SW      = 0x01,
  LIGHTS_ON    = 0x02,
  HEARTBEAT    = 0x04,
  STATE_CHANGE = 0x08,
  LOW_BATTERY  = 0x10 
};


void setup()
{
  // Setup and read the switch
  pinMode(SW_IN, INPUT_PULLUP);
  changeBit(DOOR_SW, (digitalRead(SW_IN) ? 1 : 0));

  // Setup the RESET pin 
  pinMode(RST_OUT, INPUT);

  // Setup the PC input pint
  pinMode(PC_IN, INPUT);

  // Setup the I2C Slave stuff
  TinyWireS.begin(I2C_SLAVE_ADDRESS);
  TinyWireS.onReceive(receiveEvent); 
  TinyWireS.onRequest(requestEvent);
}

void loop()
{
  // If the ESP is active we should be in a tight loop checking the I2C
  if (espActive){
    TinyWireS_stop_check();
  }

  // If the ESP is not active, we need to monitor switches and timers
  else{
    // Check the door switch for changes, debounce if necessary
    uint8_t switchInput = (digitalRead(SW_IN) ? 1 : 0);   // Read our input
    if (getBit(DOOR_SW) != switchInput) {                 // Check out input not equal to current state
      delay(DEBOUNCE_MS);                                 // Debounce delay
      switchInput = (digitalRead(SW_IN) ? 1 : 0);         // Re-read our input
      if (getBit(DOOR_SW) != switchInput){                // Re-check our input not equal to current state
        changeBit(DOOR_SW, switchInput);                  // Set DOOR_SW to equal switchInput
        changeBit(STATE_CHANGE, 1);                       // Set STATE_CHANGE true
      }
    }

    // If our state has changed, or if we're at a light check point
    else if (getBit(STATE_CHANGE) || (getValue(SLEEP_LOW) % getValue(LIGHT_CHK_LOW))){
      updatePhotocell();
    }

    // At this point we have debounced a changing switch and checked our photoeye if needed
  
    // If our state has changed, or if we're at a heart beat check point
    if (getBit(STATE_CHANGE) || (getValue(SLEEP_LOW) % getValue(HEART_LOW))){
      enableESP(); // This will do a data posting and put us into I2C Slave mode
    }

    // If we're here, nothing has changed recently so we can just go to sleep...
    else{
      systemSleep();  // Send the unit to sleep
    }
  }
}

ISR(WDT_vect) {
  uint16_t sleepPeriodCounter = getValue(SLEEP_LOW);
  if(++sleepPeriodCounter > 65630){
    sleepPeriodCounter = 0;
  }
  setValue(SLEEP_LOW, sleepPeriodCounter);
}

uint16_t getValue(uint8_t index){
  return ((i2c_reg[index+1] << 8) + i2c_reg[index]);
}

void setValue(uint8_t index, uint16_t value){
  i2c_reg[index+1] = value >> 8;
  i2c_reg[index] = value & 0xFF;
}

uint8_t getBit(uint8_t stateBit){
  uint8_t workingVal = (i2c_reg[STATE] & stateBit);
  if (workingVal){
    return 1;
  }
  else{
    return 0;
  }
}

void changeBit(uint8_t stateBit, uint8_t value){
  if (value){
    i2c_reg[STATE] |= stateBit;  
  }
  else{
    i2c_reg[STATE] &= (0xFF - stateBit);
  } 
}

void disableESP(){
  espActive = false;
  
  // Set State Change False
  changeBit(STATE_CHANGE, 0);
}

void enableESP(){
  updateTemperature();
  updateVoltage();
  updatePhotocell();

  setValue(SLEEP_LOW, 0);
  pinMode(RST_OUT, OUTPUT);
  digitalWrite(RST_OUT, LOW);
  espActive = true;
  delay(100);
  pinMode(RST_OUT, INPUT);
}

// set system into the sleep state 
// system wakes up when wtchdog is timed out
void systemSleep() {
//  pinMode(SW_IN, INPUT); // Disable the pull down to save power
  setup_watchdog(6); //Setup watchdog to go off after 1sec
  cbi(ADCSRA,ADEN);                    // switch Analog to Digitalconverter OFF
 
  set_sleep_mode(SLEEP_MODE_PWR_DOWN); // sleep mode is set here
  sleep_enable();
 
  sleep_mode();                        // System actually sleeps here
 
  sleep_disable();                     // System continues execution here when watchdog timed out 
//  pinMode(SW_IN, INPUT_PULLUP);       // Enable the pull up in anticipation of reading the switch
  sbi(ADCSRA,ADEN);                    // switch Analog to Digitalconverter ON
  delay(10);
}

void updateTemperature() {
  analogReference(INTERNAL1V1);
  int raw = analogRead(A0+15); 
  /* Original code used a 13 deg adjustment. But based on my results, I didn't seem to need it. */
  // raw -= 13; // raw adjust = kelvin //this value is used to calibrate to your chip
  setValue(TEMP_LOW, (raw - 273)); // celcius
  analogReference(DEFAULT);
}

void updateVoltage() {
  // Read 1.1V reference against AVcc
  // set the reference to Vcc and the measurement to the internal 1.1V reference
  #if defined(__AVR_ATmega32U4__) || defined(__AVR_ATmega1280__) || defined(__AVR_ATmega2560__)
    ADMUX = _BV(REFS0) | _BV(MUX4) | _BV(MUX3) | _BV(MUX2) | _BV(MUX1);
  #elif defined (__AVR_ATtiny24__) || defined(__AVR_ATtiny44__) || defined(__AVR_ATtiny84__)
    ADMUX = _BV(MUX5) | _BV(MUX0);
  #elif defined (__AVR_ATtiny25__) || defined(__AVR_ATtiny45__) || defined(__AVR_ATtiny85__)
    ADMUX = _BV(MUX3) | _BV(MUX2);
  #else
    ADMUX = _BV(REFS0) | _BV(MUX3) | _BV(MUX2) | _BV(MUX1);
  #endif  
 
  delay(2); // Wait for Vref to settle
  ADCSRA |= _BV(ADSC); // Start conversion
  while (bit_is_set(ADCSRA,ADSC)); // measuring
 
  uint8_t low  = ADCL; // must read ADCL first - it then locks ADCH  
  uint8_t high = ADCH; // unlocks both
 
  uint16_t voltage = 1125300L / ((high<<8) | low); // Calculate Vcc (in mV); 1125300 = 1.1*1023*1000

  // Are we below the voltLowThreshold
  if (voltage < getValue(VOLT_TH_LOW)){
    // Set Low Battery
    changeBit(LOW_BATTERY, 1);
  }
  // Are we above the voltLowThreshold + Hysterisis?
  else if (voltage > (getValue(VOLT_TH_LOW) + DEFAULT_VOLT_HYST)){
    // Unset Low Battery
    changeBit(LOW_BATTERY, 0);
  }

  setValue(VOLT_LOW, voltage);
}

void updatePhotocell(){
  uint16_t lightLevel = analogRead(PC_IN);
  setValue(LIGHT_LOW, lightLevel);
  
  if (getBit(LIGHTS_ON)){
    // Are we below the lightLowThreshold
    if (lightLevel < getValue(LIGHT_LOW_TH_LOW)){
      // Lights Off
      changeBit(LIGHTS_ON, 0);
      // State Change True
      changeBit(STATE_CHANGE, 1);
    }
  }
  else{
    // Are we above the lightHighThreshold
    if (lightLevel > getValue(LIGHT_HIGH_TH_LOW)){
      // Lights On True, State Change True
      changeBit(LIGHTS_ON, 1);
      changeBit(STATE_CHANGE, 1);
    }
  }
}
 
// 0=16ms, 1=32ms,2=64ms,3=128ms,4=250ms,5=500ms
// 6=1 sec,7=2 sec, 8=4 sec, 9= 8sec
void setup_watchdog(int ii) {
 
  byte bb;
  int ww;
  if (ii > 9 ) ii=9;
  bb=ii & 7;
  if (ii > 7) bb|= (1<<5);
  bb|= (1<<WDCE);
  ww=bb;
 
  MCUSR &= ~(1<<WDRF);
  // start timed sequence
  WDTCR |= (1<<WDCE) | (1<<WDE);
  // set new watchdog timeout value
  WDTCR = bb;
  WDTCR |= _BV(WDIE);
}


/**
 * This is called for each read request we receive, never put more than one byte of data (with TinyWireS.send) to the 
 * send-buffer when using this callback
 */
void requestEvent()
{  
    TinyWireS.send(i2c_reg[reg_position]);
//    // Increment the reg position on each read, and loop back to zero
    reg_position++;
    if (reg_position >= reg_size)
    {
        reg_position = 0;
    }
}


/**
 * The I2C data received -handler
 *
 * This needs to complete before the next incoming transaction (start, data, restart/stop) on the bus does
 * so be quick, set flags for long running tasks to be called from the mainloop instead of running them directly,
 */
void receiveEvent(uint8_t howMany)
{
    if (howMany < 1)
    {
        // Sanity-check
        return;
    }
    if (howMany > TWI_RX_BUFFER_SIZE)
    {
        // Also insane number
        return;
    }

    reg_position = TinyWireS.receive();

    // Handle a sleep request
    if (reg_position == 0x5E){
      disableESP();
      return;
    }
    
    howMany--;
    if (!howMany)
    {
        // This write was only to set the buffer for next read
        return;
    }
    while(howMany--)
    {
        i2c_reg[reg_position] = TinyWireS.receive();
        reg_position++;
        if (reg_position >= reg_size)
        {
            reg_position = 0;
        }
    }
}
