/*
 * Project frosty-nose
 * Description:
 * Author: E. Chiu
 * Date: Aug 2020
 * Remeber to check DEBUGMODE and TRANSMIT settings
 */

#include <MCP342X.h>

#define PIN_LED D7
#define PIN_RELAY D8

#define ADC_COUNTS_4mA 5813
#define ADC_COUNTS_20mA 29390
#define FLOW_CONVERSION_SLOPE 1
#define FLOW_CONVERSION_OFFSET 0
#define PRESSURE_CONVERSION_SLOPE 1
#define PRESSURE_CONVERSION_OFFSET 0
#define TEMP_FEED_CONVERSION_SLOPE 1
#define TEMP_FEED_CONVERSION_OFFSET 0
#define TEMP_BURNER_CONVERSION_SLOPE 1
#define TEMP_BURNER_CONVERSION_OFFSET 0
#define FRACTION_CONVERSION_SLOPE 1
#define FRACTION_CONVERSION_OFFSET 0

#define DEBUGMODE 0
#define TRANSMIT 1


//instantiate 4-20mA loop ADC
MCP342X loopADC;

//instantiate sensor globals
double flow = 1.1f;
double temp_feed = 1.2f;
double pressure = 1.3f;
double temp_burner = 1.4f;
double fraction_ch4 = 1.5f;

// Using SEMI_AUTOMATIC mode to get the lowest possible data usage by
// registering functions and variables BEFORE connecting to the cloud.
SYSTEM_MODE(SEMI_AUTOMATIC);

// setup() runs once, when the device is first turned on.
void setup() {
  // Put initialization like pinMode and begin functions here.

  //start i2c and serial
  Wire.begin();
  Serial.begin(9600); //debug USB serial
  Serial1.begin(34800); // hardware serial for methane sensor

  while (!Serial) {}  // wait for Serial comms to become ready
  Serial.println("Starting up");
  Serial.println("Testing device connection...");
    Serial.println(loopADC.testConnection() ? "MCP342X connection successful" : "MCP342X connection failed");

  //define visible variables
  Particle.variable("flow", flow);
  Particle.variable("pressure", pressure);
  Particle.variable("temp_feed", temp_feed);
  Particle.variable("temp_burner", temp_burner);
  Particle.variable("fraction_ch4", fraction_ch4);

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
  int loopTime = 20;

  // send data only when you receive data:
  if (DEBUGMODE > 0) {
    loopTime = 2;
  } else {
    loopTime = 20;
  }

  for (int k=0; k<loopTime; k++){
    digitalWrite(PIN_LED, LOW);
    delay(1900);
    digitalWrite(PIN_LED, HIGH);
    delay(100);
  }

  //wake up and poll sensors
  //turn relay on
  digitalWrite(PIN_RELAY, HIGH);
  //time for the sensors to stabilize, flash LED to indicate
  for (int k=0; k<75; k++){
    digitalWrite(PIN_LED, LOW);
    delay(100);
    digitalWrite(PIN_LED, HIGH);
    delay(100);
  }
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

  delay(100); //just in case

  if (DEBUGMODE > 0) {
    digitalWrite(PIN_RELAY, LOW); //don't turn off sensors in debug mode
  } else {
    digitalWrite(PIN_RELAY, LOW);
  }

  Serial.print("Ch1 ");
  Serial.println(adcCh1);
  Serial.print("Ch2 ");
  Serial.println(adcCh2);
  Serial.print("Ch3 ");
  Serial.println(adcCh3);
  Serial.print("Ch4 ");
  Serial.println(adcCh4);


  //convert ADC counts to engineering units
  flow = FLOW_CONVERSION_SLOPE*16.f*(adcCh1-ADC_COUNTS_4mA)/(ADC_COUNTS_20mA-ADC_COUNTS_4mA) + 4.f + FLOW_CONVERSION_OFFSET;
  temp_feed = TEMP_FEED_CONVERSION_SLOPE*16.f*(adcCh2-ADC_COUNTS_4mA)/(ADC_COUNTS_20mA-ADC_COUNTS_4mA) + 4.f + TEMP_FEED_CONVERSION_OFFSET;
  pressure = PRESSURE_CONVERSION_SLOPE*16.f*(adcCh3-ADC_COUNTS_4mA)/(ADC_COUNTS_20mA-ADC_COUNTS_4mA) + 4.f + PRESSURE_CONVERSION_OFFSET;
  temp_burner = TEMP_BURNER_CONVERSION_SLOPE*16.f*(adcCh4-ADC_COUNTS_4mA)/(ADC_COUNTS_20mA-ADC_COUNTS_4mA) + 4.f + TEMP_BURNER_CONVERSION_OFFSET;

  Serial.println(String::format("flow=%.1f, pressure=%.1f, temp_feed=%.1f, temp_burner=%.1f, fraction_ch4=%.1f", flow, pressure, temp_feed, temp_burner, fraction_ch4));

  if (TRANSMIT > 0) {
    Particle.publish("scheduledReport", String::format("flow=%.1f, pressure=%.1f, temp_feed=%.1f, temp_burner=%.1f, fraction_ch4=%.1f", flow, pressure, temp_feed, temp_burner, fraction_ch4), PRIVATE, NO_ACK);
  }

  //pseudocode: turn relay off
  digitalWrite(PIN_LED, LOW);
  delay(1000);
  digitalWrite(PIN_LED, HIGH);
  delay(1000);
}