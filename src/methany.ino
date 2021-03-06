/*
 * Project frosty-nose
 * Description:
 * Author: E. Chiu
 * Date: Aug 2020
 * Remeber to check DEBUGMODE and TRANSMIT settings
 */

#include <MCP342X.h>
//#include <cstdlib>
//#include <iostream>

#define PIN_LED D7
#define PIN_RELAY D8

//conversion constants
#define ADC_COUNTS_4mA 5813
#define ADC_COUNTS_20mA 29390
#define FLOW_READING_AT_4mA 0.f //cfm
#define FLOW_READING_AT_20mA 33.f
#define PRESSURE_READING_AT_4mA 0.f //psi
#define PRESSURE_READING_AT_20mA 30.f
#define TEMP_FEED_READING_AT_4mA -20.f //degC
#define TEMP_FEED_READING_AT_20mA 100.f
#define TEMP_BURNER_READING_AT_4mA -17.78f //degC
#define TEMP_BURNER_READING_AT_20mA 1093.33f

#define DEBUGMODE 0   //in debug mode we skip the sleep time and don't power off the sensors
#define TRANSMIT 1    //set to 1 to enable celluar reporting, set to 0 to disable
                      //check this one to make sure you don't unintentionally consume data during debugging

//timing parameters, in seconds
#define SENSOR_WARMUP 15  //time required between sensor turn on and reading
#define SLEEP_TIME 60 //time between cycles of sensor read + report 
#define HOUSEKEEPING_CYCLE 0.5 //how often the housekeeping cycle runs during the idle state
#define WARMUP_CYCLE 0.1 //this is basically to blink the LED rapidly during warmup

//instatiate global state variables
static int state = 0;
static int cyclecounter = 0;

//instantiate 4-20mA loop ADC
MCP342X loopADC;

//instantiate sensor globals
double flow = 1.1f;
double temp_feed = 1.2f;
double pressure = 1.3f;
double temp_burner = 1.4f;
double percent_ch4 = 1.5f;

// Using SEMI_AUTOMATIC mode to get the lowest possible data usage by
// registering functions and variables BEFORE connecting to the cloud.
SYSTEM_MODE(SEMI_AUTOMATIC);

// setup() runs once, when the device is first turned on.
void setup() {
  // Put initialization like pinMode and begin functions here.

  //start i2c and serial
  Wire.begin();
  Serial.begin(9600); //debug USB serial
  Serial1.begin(38400); // hardware serial for methane sensor
  Serial1.setTimeout(10);

  while (!Serial) {}  // wait for Serial comms to become ready
  Serial.println("Starting up");
  Serial.println("Testing device connection...");
    Serial.println(loopADC.testConnection() ? "MCP342X connection successful" : "MCP342X connection failed");

  //define visible variables
  Particle.variable("flow", flow);
  Particle.variable("pressure", pressure);
  Particle.variable("temp_feed", temp_feed);
  Particle.variable("temp_burner", temp_burner);
  Particle.variable("percent_ch4", percent_ch4);

  //establish connection
  Particle.connect();

  //set pin modes
  pinMode(PIN_LED, OUTPUT);
  pinMode(PIN_RELAY, OUTPUT);

  //set up MCP3428 ADC
  loopADC.configure( MCP342X_MODE_CONTINUOUS |
                   MCP342X_SIZE_16BIT |
                   MCP342X_GAIN_2X
                 );
}

// loop() runs over and over again, as quickly as it can execute.
void loop() {
  // The core of your code will likely live here.

  switch(state) {
    case 0:  //idle, check UART buffer, update CH4 Concentration reading, flash LED
      cyclecounter++;
      digitalWrite(PIN_LED, (cyclecounter/2)%2 > 0.5);  //toggle LED every two cycles
      if (DEBUGMODE > 0) {
        digitalWrite(PIN_RELAY, HIGH);  //leave sensors on in debug state
      } else {
        digitalWrite(PIN_RELAY, LOW);  //sensor power off in this state
      }
      updateMethaneSensor();
      if (cyclecounter * HOUSEKEEPING_CYCLE > SLEEP_TIME) {
        cyclecounter = 0;
        state = 1;
      }
      if (DEBUGMODE > 0) {
        state = 1;  //immediately skip the sleep state in debug mode
      }
      delay(HOUSEKEEPING_CYCLE*1000);
      break;
    
    case 1:  //wake up the sensors and wait for them to warm up
      cyclecounter++;
      digitalWrite(PIN_LED, cyclecounter%2);  //fast LED flash to indicate warmup period
      digitalWrite(PIN_RELAY, HIGH);  //sensor power on in this state
      updateMethaneSensor();      
      if (cyclecounter * WARMUP_CYCLE > SENSOR_WARMUP) {
        cyclecounter = 0;
        state = 2;
      }
      delay(WARMUP_CYCLE*1000);
      break;

    case 2:  //read all sensors and convert to engineering units
      cyclecounter++;
      digitalWrite(PIN_LED, HIGH);  //SOLID LED to indicate report
      digitalWrite(PIN_RELAY, HIGH);  //sensor power on in this state

      static int16_t adcCh1;
      static int16_t adcCh2;
      static int16_t adcCh3;
      static int16_t adcCh4;
      loopADC.startConversion(MCP342X_CHANNEL_1);
      loopADC.getResult(&adcCh1);
      loopADC.startConversion(MCP342X_CHANNEL_2); 
      loopADC.getResult(&adcCh2);
      loopADC.startConversion(MCP342X_CHANNEL_3);
      loopADC.getResult(&adcCh3); 
      loopADC.startConversion(MCP342X_CHANNEL_4);
      loopADC.getResult(&adcCh4);
      updateMethaneSensor();
      
      delay(100); //just in case

      //convert ADC counts to engineering units
      flow = (adcCh1-ADC_COUNTS_4mA)*(FLOW_READING_AT_20mA-FLOW_READING_AT_4mA)/(ADC_COUNTS_20mA-ADC_COUNTS_4mA)+FLOW_READING_AT_4mA;
      temp_feed = (adcCh2-ADC_COUNTS_4mA)*(TEMP_FEED_READING_AT_20mA-TEMP_FEED_READING_AT_4mA)/(ADC_COUNTS_20mA-ADC_COUNTS_4mA)+TEMP_FEED_READING_AT_4mA;
      pressure = (adcCh3-ADC_COUNTS_4mA)*(PRESSURE_READING_AT_20mA-PRESSURE_READING_AT_4mA)/(ADC_COUNTS_20mA-ADC_COUNTS_4mA)+PRESSURE_READING_AT_4mA;
      temp_burner = (adcCh4-ADC_COUNTS_4mA)*(TEMP_BURNER_READING_AT_20mA-TEMP_BURNER_READING_AT_4mA)/(ADC_COUNTS_20mA-ADC_COUNTS_4mA)+TEMP_BURNER_READING_AT_4mA;

      state = 3;
      cyclecounter = 0;
      break;

    case 3:  //report data
      cyclecounter++;
      if (TRANSMIT > 0) {
        Particle.publish("scheduledReport", String::format("FLOW=%.1fcfm, PRES=%.1fpsi, TEMP=%.1fC, BURN=%.1fC, PCH4=%.1f%%", flow, pressure, temp_feed, temp_burner, percent_ch4), PRIVATE, NO_ACK);
      }

      //Debug data out to USB serial
      Serial.print("Ch1 ");
      Serial.println(adcCh1);
      Serial.print("Ch2 ");
      Serial.println(adcCh2);
      Serial.print("Ch3 ");
      Serial.println(adcCh3);
      Serial.print("Ch4 ");
      Serial.println(adcCh4);
      Serial.println(String::format("FLOW=%.1fcfm, PRES=%.1fpsi, TEMP=%.1fC, BURN=%.1fC, PCH4=%.1f%%", flow, pressure, temp_feed, temp_burner, percent_ch4));

      state = 4;
      cyclecounter = 0;
      break;

    case 4:  //cleanup tasks go here if needed
      //pseudocode: flush UART buffer
      cyclecounter = 0;
      state = 0;
      break;
    
    default:
      cyclecounter = 0;
      state = 0;
      break; 

  }  //end of state machine, switch statement

}


//read the UART buffer and update the methane reading
//note this needs some data handling cleanup
//to prevent hang due to a failed sensor spamming UART data
void updateMethaneSensor() {  
  unsigned int ch4_ppm = 0;
  if (Serial1.available() > 40) {
    while(Serial1.available()) {
      String inirSensorData = Serial1.readStringUntil('\n');
      if (inirSensorData == "0000005b") {  //look for the start symbol
        if(DEBUGMODE > 0) {Serial.println(inirSensorData);}  //debug out to USB serial
        inirSensorData = Serial1.readStringUntil('\n');  //next 8 bytes after start symbol are ppm data
        if(DEBUGMODE > 0) {Serial.println(inirSensorData);}  //debug out to USB serial
        ch4_ppm = std::strtoul(inirSensorData, 0, 16);
        Serial.print(">> CH4 PPM: ");
        Serial.println(ch4_ppm);
        percent_ch4 = ch4_ppm / 10000.f;
      }
      if(DEBUGMODE > 0) {Serial.println(inirSensorData);}  //debug out to USB serial
    }
  }
  return;
}