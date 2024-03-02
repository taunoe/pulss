/*
 Copyright 2024
 Tauno Erik
 Started 01.03.2024
 Edited  02.03.2024
 MAX30102 Heart Rate Sensor
 ESP8266 MCU

 https://forum.arduino.cc/t/managing-serial-print-as-a-debug-tool/1024824
*/

#include <Arduino.h>
#include <Wire.h>       // I2C
#include "MAX30105.h"   // SparkFun MAX3010x
#include "heartRate.h"  // SparkFun MAX3010x

#define TAUNO_DEBUG 1
#include "tauno_debug.h"

// Init Heart Rate Sensor
MAX30105 pulss;

void heart_beat_plotter_setup();
void heart_beat_plotter_loop();
void presence_sensing_setup();
void presence_sensing_loop();
void temp_sense_setup();
void temp_sense_loop();
void heart_rate_setup();
void heart_rate_loop();

// Presense
int32_t samplesTaken = 0;  // Counter for calculating the Hz or read rate
int32_t unblockedValue;  // Average IR at power up
uint64_t startTime;  // Used to calculate measurement rate

// Heart rate
const byte RATE_SIZE = 4; //Increase this for more averaging. 4 is good.
byte rates[RATE_SIZE]; //Array of heart rates
byte rateSpot = 0;
long lastBeat = 0; //Time at which the last beat occurred

float beatsPerMinute;
int beatAvg;


void setup() {
  // Serial.begin(115200);
  DEBUG_SERIAL_BEGIN(115200)
  // Serial.println("Alustame...");
  DEBUG_PRINTLN("Alustame...")

  // Initialize sensor
  // Use default I2C port, 400kHz speed
  if (!pulss.begin(Wire, I2C_SPEED_FAST)) {
    DEBUG_PRINTLN("MAX30102 was not found. Please check wiring/power. ");
    while (1) {
      // Stop! If no sensor is found
    }
  }

  // Configure Heart Rate Sensor
  // Select one:
  // heart_beat_plotter_setup();
  // presence_sensing_setup();
  // temp_sense_setup();
  heart_rate_setup();
}

void loop() {
  // heart_beat_plotter_loop();
  // presence_sensing_loop();
  // temp_sense_loop();
  heart_rate_loop();
}


/*
Good settings to plott
*/
void heart_beat_plotter_setup() {
  // Heart Rate Sensor Settings
  uint8_t led_brightness = 0x1F;  // Options: 0=Off to 255=50mA
  uint8_t sample_average = 8;  // Options: 1, 2, 4, 8, 16, 32
  uint8_t led_mode = 3;  // Options: 1 = Red, 2 = Red + IR, 3 = Red + IR + Green

  int sample_rate = 100;  // Options: 50, 100, 200, 400, 800, 1000, 1600, 3200
  int pulse_width = 411;  // Options: 69, 118, 215, 411
  int adc_range = 4096;  // Options: 2048, 4096, 8192, 16384

  // Configure Heart Rate Sensor
  pulss.setup(led_brightness,
              sample_average,
              led_mode,
              sample_rate,
              pulse_width,
              adc_range);
}

void heart_beat_plotter_loop() {
  DEBUG_PRINT("RED");
  DEBUG_PRINT(pulss.getRed());
  DEBUG_PRINT("IR");
  DEBUG_PRINT(pulss.getIR());
  // DEBUG_PRINT("Green");
  // DEBUG_PRINT(pulss.getGreen());
  DEBUG_PRINTLN();
}

void presence_sensing_setup() {
  // Heart Rate Sensor Settings
  uint8_t led_brightness = 0xFF;  // Options: 0=Off to 255=50mA
  uint8_t sample_average = 4;  // Options: 1, 2, 4, 8, 16, 32
  uint8_t led_mode = 2;  // Options: 1 = Red, 2 = Red + IR, 3 = Red + IR + Green

  int sample_rate = 400;  // Options: 50, 100, 200, 400, 800, 1000, 1600, 3200
  int pulse_width = 411;  // Options: 69, 118, 215, 411
  int adc_range = 2048;  // Options: 2048, 4096, 8192, 16384

  // Configure Heart Rate Sensor
  pulss.setup(led_brightness,
              sample_average,
              led_mode,
              sample_rate,
              pulse_width,
              adc_range);

  pulss.setPulseAmplitudeRed(0);  // Turn off Red LED
  pulss.setPulseAmplitudeGreen(0);  // Turn off Green LED

  // Take an average of IR readings at power up
  unblockedValue = 0;
  for (byte x = 0 ; x < 32 ; x++) {
    unblockedValue += pulss.getIR();
  }
  unblockedValue /= 32;

  startTime = millis();
}

void presence_sensing_loop() {
  samplesTaken++;

  Serial.print("IR[");
  Serial.print(pulss.getIR());
  Serial.print("] Hz[");
  Serial.print((float)samplesTaken / ((millis() - startTime) / 1000.0), 2);
  Serial.print("]");

  int32_t currentDelta = pulss.getIR() - unblockedValue;

  Serial.print(" delta[");
  Serial.print(currentDelta);
  Serial.print("]");

  if (currentDelta > (int32_t)120) {
    Serial.print(" Something is there!");
  }

  Serial.println();
}

void temp_sense_setup() {
  pulss.setup(0);  // Turn off LEDs
  pulss.enableDIETEMPRDY();  // Enable the temp ready interrupt. This is required.
}

void temp_sense_loop() {
  float temperature = pulss.readTemperature();

  Serial.print("tempC=");
  Serial.print(temperature, 4);

  Serial.println();
}

void heart_rate_setup() {
  pulss.setup();  // Configure sensor with default settings
  pulss.setPulseAmplitudeRed(0x0A);  // Turn Red LED to low to indicate sensor is running
  pulss.setPulseAmplitudeGreen(0);  // Turn off Green LED
}

void heart_rate_loop() {
  long irValue = pulss.getIR();

  if (checkForBeat(irValue) == true)
  {
    //We sensed a beat!
    long delta = millis() - lastBeat;
    lastBeat = millis();

    beatsPerMinute = 60 / (delta / 1000.0);

    if (beatsPerMinute < 255 && beatsPerMinute > 20)
    {
      rates[rateSpot++] = (byte)beatsPerMinute; //Store this reading in the array
      rateSpot %= RATE_SIZE; //Wrap variable

      //Take average of readings
      beatAvg = 0;
      for (byte x = 0 ; x < RATE_SIZE ; x++)
        beatAvg += rates[x];
      beatAvg /= RATE_SIZE;
    }
  }

  Serial.print("IR=");
  Serial.print(irValue);
  Serial.print(", BPM=");
  Serial.print(beatsPerMinute);
  Serial.print(", Avg BPM=");
  Serial.print(beatAvg);

  if (irValue < 50000)
    Serial.print(" No finger?");

  Serial.println();
}

