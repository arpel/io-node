//--------------------------------------------------------------------------------------
// Internal temperature measurement of Attiny84 based FunkySensor derivative
// based on work from harizanov.com and jeelabs.org
// Arpel @ 2012-2013
// GNU GPL V3
//--------------------------------------------------------------------------------------
#include <JeeLib.h> // https://github.com/jcw/jeelib
#include "pins_arduino.h"
#include <avr/sleep.h>

#include <EEPROM.h>

//########################################################################################################################
// Default Configuration
//########################################################################################################################
#define STATIC_ONEWIRE_INDEXES 1 // Save time and 118 Bytes of code : good for ATTiny !

//########################################################################################################################
// OTA Configuration
//########################################################################################################################
// ID of the settings block
#define CONFIG_VERSION "tom" //keep this 3 chars long
#define CONFIG_START 32      // Offset of the configuration in EEPROM

struct StoreStruct {
  // This is for mere detection if they are your settings
  char version[4];  // 3+trailing zero
  byte myNodeID, needACK, loopIterations, loopDuration;
} storage = {
  CONFIG_VERSION,
  // The default values
  30, false, 5, 1
};

//########################################################################################################################
// General Configuration
//########################################################################################################################
#define destNodeID 1      // Valid for both rootNode and confNode
#define productionNetwork 100
#define configurationNetwork 200
//#define network 100      // RF12 Network group
#define freq RF12_868MHZ // Frequency of RFM12B module

#define NEED_ACK 0
#define RETRY_PERIOD 5    // How soon to retry (in seconds) if ACK didn't come in
#define RETRY_LIMIT 5     // Maximum number of times to retry
#define ACK_TIME 100       // Number of milliseconds to wait for an ack

#include <OneWire.h>   // http://www.pjrc.com/teensy/arduino_libraries/OneWire.zip
#include <DallasTemperature.h>
#include <ds2408.h>  //https://bitbucket.org/lutorm/arduino/src/7231276bd618/libraries/ds2408?at=default

#define ASYNC_DELAY 750 // 9bit requres 95ms, 10bit 187ms, 11bit 375ms and 12bit resolution takes 750ms
 
#define ONE_WIRE_BUS PIN_A7    // pad 5 of the Funky
#define tempPower PIN_A3       // Power pin is connected pad 4 on the Funky
#define LEDpin PIN_A0

#define ADCFORVCCONLY 1

#define LEVEL_CLOSED 0
#define LEVEL_FULL_OPEN 255

#define COMMAND_UNKNOWN 0
#define COMMAND_OPEN 1
#define COMMAND_CLOSE 2
#define COMMAND_STOP 3

// 250ms loop duration, 30s max action list : 120 iterations
#define CURTAIN_LOOP_MAX_ITERATIONS 128
#define CURTAIN_LOOP_DURATION 250

#define CURTAIN_ACTION_LIST_LEN 32

//########################################################################################################################
// Local typefdefs
//########################################################################################################################
typedef struct {
  byte nodeid;          // Node ID
  byte id;              // Packet ID
  byte status;          // 
  byte nbCommands;	
  byte lastCommand;
  unsigned int supplyV; // Supply voltage
  byte numsensors;
} Payload_t;

typedef struct {
  byte group;
  byte timeToOpen;
  byte timeToClose;
  byte currentLevel;
  byte lastCommand;
} curtain_t;

typedef struct {
  byte command;
  byte action_tic;
  byte curtain_index;
  byte curtain_next_level;
} curtain_action_t;

//########################################################################################################################
// Local functions
//########################################################################################################################
#ifdef ADCFORVCCONLY
static unsigned int vccRead(unsigned int count);
#else
static unsigned int adcMeanRead(byte adcmux, unsigned int count, bool tovdc);
#endif
void RF_AirSend(Payload_t *pl);

// Special for configuration
void listenConfig(void);
void loadConfig();
void saveConfig();

// Utilities
static void blinkLED(byte ntimes, byte time);
static byte waitForAck();
static void loseSomeTime(unsigned int ms);

//########################################################################################################################
// Local variables
//########################################################################################################################
// Setup a DS2408 (OneWire BUS created on the fly)
DS2408 iocontrol(ONE_WIRE_BUS);
Devices devices;

volatile bool adcDone;

// for low-noise/-power ADC readouts, we'll use ADC completion interrupts
ISR(ADC_vect) { adcDone = true; }

// this must be defined since we're using the watchdog for low-power waiting
ISR(WDT_vect) { Sleepy::watchdogEvent(); }

Payload_t staticpayload;

curtain_t curtains[] = {  {1, 15, 20, LEVEL_CLOSED, COMMAND_UNKNOWN},
                          {2, 15, 20, LEVEL_CLOSED, COMMAND_UNKNOWN},
                          {3, 15, 20, LEVEL_CLOSED, COMMAND_UNKNOWN},
                          {4, 15, 20, LEVEL_CLOSED, COMMAND_UNKNOWN},
                          {5, 15, 20, LEVEL_CLOSED, COMMAND_UNKNOWN},
                          {0}};

curtain_action_t curtain_actions[CURTAIN_ACTION_LIST_LEN];

//########################################################################################################################
// LOCAL Functions
//########################################################################################################################
//--------------------------------------------------------------------------------------------------
// Send payload data via RF
//--------------------------------------------------------------------------------------------------
#ifdef ADCFORVCCONLY
static unsigned int vccRead(unsigned int count = 10) {
#else
static unsigned int vccRead(unsigned int count = 10) {
  return adcMeanRead(B00100001, count, true);
}
static unsigned int adcMeanRead(byte adcmux, unsigned int count = 10, bool tovdc = false) {
#endif
  unsigned long adccumulative = 0;
  unsigned int totalcount = count;
  
  ADCSRB = 0;
  ADCSRA = bit(ADEN) | bit(ADIE) | bit(ADATE); // Enable + Interrupt Enable Auto Trigger Enable - Start  
#ifdef ADCFORVCCONLY
  ADMUX = B00100001;
#else
  ADMUX = adcmux;
#endif

  set_sleep_mode(SLEEP_MODE_ADC);
  
  bitSet(ADCSRA, ADSC); // Start first conversion
  // Throw first value
  adcDone = false;
  while (!adcDone)
    sleep_mode();

  while (count-- > 0) {
    adcDone = false;
    while (!adcDone)
      sleep_mode();
    adccumulative += ADC;
  }
  ADCSRA = 0;  

#ifdef ADCFORVCCONLY
  adccumulative = 1125300L / (adccumulative / totalcount);
  return (unsigned int)adccumulative;
#else
  if (tovdc == true){
    adccumulative = 1125300L / (adccumulative / totalcount);
    return (unsigned int)adccumulative;
  } else {
    return (unsigned int)(adccumulative / totalcount);
  }
#endif
}

//--------------------------------------------------------------------------------------------------
// Send payload data via RF
//--------------------------------------------------------------------------------------------------
void RF_AirSend(Payload_t *pl){
  bitClear(PRR, PRUSI); // enable USI h/w
  digitalWrite(LEDpin, LOW);
   
  for (byte i = 0; i <= RETRY_LIMIT; ++i) {  // tx and wait for ack up to RETRY_LIMIT times
       rf12_sleep(RF12_WAKEUP);
            
       byte header = RF12_HDR_ACK | RF12_HDR_DST | destNodeID;
  
       rf12_sendNow(header, pl, sizeof *pl);
       rf12_sendWait(2); // Wait for RF to finish sending while in standby mode
#if NEED_ACK
         byte acked = waitForAck();  // Wait for ACK
#endif
       rf12_sleep(RF12_SLEEP);
#if NEED_ACK
         if (acked) { break; }      // Return if ACK received
         loseSomeTime(RETRY_PERIOD * 500);     // If no ack received wait and try again
#else
          break;
#endif
  } 
   
  digitalWrite(LEDpin, HIGH);
  bitSet(PRR, PRUSI); // disable USI h/w
}

//########################################################################################################################
// GLOBAL Functions
//########################################################################################################################
//--------------------------------------------------------------------------------------------------
// Arduino STYLE / SETUP
//--------------------------------------------------------------------------------------------------
void setup() {
   pinMode(LEDpin, OUTPUT);
   blinkLED(1, 50);
  
   cli();
   CLKPR = bit(CLKPCE);
#if defined(__AVR_ATtiny84__)
   CLKPR = 0; // div 1, i.e. speed up to 8 MHz
#else
   CLKPR = 1; // div 2, i.e. slow down to 8 MHz
#endif
   sei();
   
   // Special configuration mode
   rf12_initialize(storage.myNodeID, freq, configurationNetwork);
   listenConfig();
   
   // Load configuration, previous one or just saved one
   loadConfig();
  
   rf12_initialize(storage.myNodeID, freq, productionNetwork); // Initialize RFM12 with settings defined above 
   // Adjust low battery voltage to 2.2V
   rf12_control(0xC040);
   rf12_sleep(RF12_SLEEP);  // Put the RFM12 to sleep
 
   PRR = bit(PRTIM1); // only keep timer 0 going

   rf12_recvDone();
   rf12_recvDone();
   rf12_recvDone();
  
   /* Setup Dallas Temp probe */
   pinMode(tempPower, OUTPUT); // set power pin for DS18B20 to output
   digitalWrite(tempPower, HIGH); // turn sensor power on
   loseSomeTime(50); // Allow 50ms for the sensor to be ready
   // Start up the library
   //sensors.begin();
   //sensors.setWaitForConversion(false); 
   //staticpayload.numsensors = sensors.getDeviceCount();

  staticpayload.numsensors = iocontrol.find(&devices);
  for(int index=0; index < staticpayload.numsensors; index++) {
    iocontrol.set_mode(devices[index], RESET_PIN_MODE(STROBE_MODE));
  }
  
  // Initialize action list at 0
  memset(curtain_actions, 0, sizeof curtain_actions);

  blinkLED(1, 50);
}


void curtain_loop(){
  for (byte tic = 0; tic < CURTAIN_LOOP_MAX_ITERATIONS; ++tic){
    // Parse action list
    for (byte action_index = 0; action_index < CURTAIN_ACTION_LIST_LEN; ++action_index){
      curtain_action_t *action = &(curtain_actions[action_index]);
      if( (action->command != COMMAND_UNKNOWN) &&
          (action->action_tic < tic)){
        // Action
        // set_curtain(action->curtain_index, action->curtain_next_level);
        // Status
        curtains[action->curtain_index].currentLevel = action->curtain_next_level;
        curtains[action->curtain_index].lastCommand = action->command;
        // Mark as done
        action->command = COMMAND_UNKNOWN;
      }
    }
    loseSomeTime(CURTAIN_LOOP_DURATION);
  }
}


//--------------------------------------------------------------------------------------------------
// Arduino STYLE / LOOP
//--------------------------------------------------------------------------------------------------
void loop() {
   bitClear(PRR, PRADC); // power up the ADC
   loseSomeTime(16); // Allow 10ms for the sensor to be ready
  
   staticpayload.supplyV = vccRead(4);
   // staticpayload temp1 & temp2 set according configuration
   //readDS18120();
  uint8_t state = 0;
  iocontrol.set_state(state, true);

   bitSet(PRR, PRADC); // power down the ADC

   staticpayload.nodeid = storage.myNodeID;
   staticpayload.id += 1;
    
  RF_AirSend(&staticpayload);
  
  curtain_loop();

  // Controlled by Configuration !
  for (byte i = 0; i < storage.loopIterations; ++i)
    loseSomeTime(storage.loopDuration*1000);
}


//########################################################################################################################
// Configuration functions
//########################################################################################################################
void loadConfig() {
  // To make sure there are settings, and they are ours. If nothing is found it will use the default settings.
  if (EEPROM.read(CONFIG_START + 0) == CONFIG_VERSION[0] &&
      EEPROM.read(CONFIG_START + 1) == CONFIG_VERSION[1] &&
      EEPROM.read(CONFIG_START + 2) == CONFIG_VERSION[2])
  {
      for (unsigned int t=0; t<sizeof(storage); t++)
        *((char*)&storage + t) = EEPROM.read(CONFIG_START + t);
  }
}

void saveConfig() {
  for (unsigned int t=0; t<sizeof(storage); t++)
    EEPROM.write(CONFIG_START + t, *((char*)&storage + t));
}

void listenConfig(void){
  // Send a Bottle to the see
  bitClear(PRR, PRUSI); // enable USI h/w
  blinkLED(2, 25);
   
  for (byte j = 0; j <= 5; ++j) {
      blinkLED(1, 50);

      rf12_sleep(RF12_WAKEUP);
              
      int ask_for_configuration = 0xCAFE;
      rf12_sendNow((RF12_HDR_ACK | RF12_HDR_DST | destNodeID), &ask_for_configuration, 2);
      rf12_sendWait(2); // Wait for RF to finish sending while in standby mode
      byte acked = waitForAck();  // Wait for ACK
      rf12_sleep(RF12_SLEEP);

      // Ok some one
      if (acked) { 
          do {
              blinkLED(1, 50);
              loseSomeTime(250);
              
              if (rf12_recvDone() && rf12_len == 6 && rf12_data[0] == 0xDE && rf12_data[1] == 0xCA) {
                  storage.myNodeID = rf12_data[2];
                  storage.needACK = rf12_data[3];
                  storage.loopIterations = rf12_data[4];
                  storage.loopDuration = rf12_data[5];
                  saveConfig();
                  j = 6;
                  break;
              }
          }while(1);
      } else {
        loseSomeTime(1 * 1000);
      }
  }
   
  blinkLED(2, 25);
  bitSet(PRR, PRUSI); // disable USI h/w
}

//########################################################################################################################
// Utilities
//########################################################################################################################
//--------------------------------------------------------------------------------------------------
// waitForAck
//--------------------------------------------------------------------------------------------------
static byte waitForAck() {
  MilliTimer ackTimer;
  while (!ackTimer.poll(ACK_TIME)) {
   if (rf12_recvDone() && rf12_crc == 0 && rf12_hdr == (RF12_HDR_CTL | destNodeID))
     return 1;
  }
   return 0;
}
//--------------------------------------------------------------------------------------------------
// loseSomeTime
//--------------------------------------------------------------------------------------------------
static void loseSomeTime(unsigned int ms){
    byte oldADCSRA=ADCSRA;      // Save ADC state
    byte oldADCSRB=ADCSRB;
    byte oldADMUX=ADMUX;
    
    Sleepy::loseSomeTime(ms);   // JeeLabs power save function: enter low power mode for x seconds (valid range 16-65000 ms)
    
    ADCSRA=oldADCSRA;           // Restore ADC state
    ADCSRB=oldADCSRB;
    ADMUX=oldADMUX;    
}
//--------------------------------------------------------------------------------------------------
// blinkLED
//--------------------------------------------------------------------------------------------------
static void blinkLED(byte ntimes, byte time){
  for (byte i = 0; i <= ntimes; ++i) {
      digitalWrite(LEDpin,LOW);
      loseSomeTime(time/2);
      digitalWrite(LEDpin,HIGH);
      loseSomeTime(time/2);
  }
}