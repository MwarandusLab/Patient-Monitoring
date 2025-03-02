#include <OneWire.h>
#include "MAX30105.h"
#include "heartRate.h"
#include "BluetoothSerial.h"
#include <DallasTemperature.h>

// Define the GPIO pin for DS18B20
#define ONE_WIRE_BUS 18

MAX30105 particleSensor;

// Setup a OneWire instance
OneWire oneWire(ONE_WIRE_BUS);

const byte RATE_SIZE = 4;  //Increase this for more averaging. 4 is good.
byte rates[RATE_SIZE];     //Array of heart rates
byte rateSpot = 0;
long lastBeat = 0;  //Time at which the last beat occurred

float beatsPerMinute;
int beatAvg;
long irValue;
int DataisReady = 0;
float temperatureC;

enum State{
  IDLE,
  MEASURE_TEMPERATURE,
  MEASURE_HEART_RATE
};

State currentState = IDLE;

unsigned long measurementStartTime;
unsigned long DataSendStartTime;

// Pass OneWire reference to DallasTemperature library
DallasTemperature sensors(&oneWire);
BluetoothSerial SerialBT;

void setup() {
  Serial.begin(9600);
  sensors.begin();
  SerialBT.begin("PATIENT MONITORING");

  if (!particleSensor.begin(Wire, I2C_SPEED_FAST)) {
    Serial.println("MAX30102 was not found. Please check wiring/power. ");
    while (1)
      ;
  }

  particleSensor.setup();                     // Configure sensor with default settings
  particleSensor.setPulseAmplitudeRed(0x0A);  // Turn Red LED to low to indicate the sensor is running
  particleSensor.setPulseAmplitudeGreen(0);   // Turn off Green LED
  delay(1000);
}

void loop() {
  switch (currentState){
    case IDLE:
      idle();
      break;
    case MEASURE_TEMPERATURE:
      measure_Temperature();
      break;
    case MEASURE_HEART_RATE:
      measureBP();
      break;
  }
}
void idle(){
  checkFingerDetection();
}
void checkFingerDetection() {
  irValue = particleSensor.getIR();

  if (irValue > 70000) {
    currentState = MEASURE_TEMPERATURE;
    measurementStartTime = millis();
  }
}
void measure_Temperature(){
  sensors.requestTemperatures();                    // Request temperature reading
  temperatureC = sensors.getTempCByIndex(0);  // Get temperature in Celsius

  // Serial.print("Temperature: ");
  // Serial.print(temperatureC);
  // Serial.println(" °C");
  SerialBT.print(temperatureC, 1);
  SerialBT.println(" °C                    Loading");
  delay(1000);

   if (millis() - measurementStartTime >= 10000) {
    Serial.println("Done Measuring");
    currentState = MEASURE_HEART_RATE;
  }
}
void measureBP() {
  Serial.println("Heart Measuring");
  irValue = particleSensor.getIR();
  if (checkForBeat(irValue) == true) {
    //We sensed a beat!
    long delta = millis() - lastBeat;
    lastBeat = millis();

    beatsPerMinute = 60 / (delta / 1000.0);

    if (beatsPerMinute < 255 && beatsPerMinute > 20) {
      rates[rateSpot++] = (byte)beatsPerMinute;  //Store this reading in the array
      rateSpot %= RATE_SIZE;                     //Wrap variable

      //Take average of readings
      beatAvg = 0;
      for (byte x = 0; x < RATE_SIZE; x++)
        beatAvg += rates[x];
      beatAvg /= RATE_SIZE;
    }
  }
  if (beatAvg > 65 && beatAvg < 80 && beatsPerMinute > 65 && DataisReady == 0) {
    // Send the BPM data to database
    // Serial.println("Data is Ready");
    // Serial.print("IR=");
    // Serial.print(irValue);
    // Serial.print(", BPM=");
    // Serial.print(beatsPerMinute);
    // Serial.print(", Avg BPM=");
    // Serial.print(beatAvg);
    SerialBT.print(temperatureC, 1);
    SerialBT.print(" °C");
    SerialBT.print("                    ");
    SerialBT.print(beatAvg);
    SerialBT.println(" BPM");
    delay(10000);
    SerialBT.println("DONE MEASURING");
    delay(5000);
    SerialBT.println("NEXT PATIENT");
    currentState = IDLE;
  }
}
