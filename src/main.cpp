/*
 Copyright 2024
 Tauno Erik
 Started 01.03.2024
 Edited  03.03.2024
 MAX30102 Heart Rate Sensor
 ESP8266 MCU

 MAX30102 does not have a Green LED.

 D1 - GPIO4 - SDA
 D2 - GPIO5 - SCL
 D5 - GPIO14 - INT
 D6 - GPIO12 - RGB LED

 https://forum.arduino.cc/t/managing-serial-print-as-a-debug-tool/1024824
*/

#include <Arduino.h>
#include <Wire.h>       // I2C
#include "MAX30105.h"   // SparkFun MAX3010x
#include "heartRate.h"  // SparkFun MAX3010x

#define TAUNO_DEBUG 1
#include "tauno_debug.h"

#include "FastLED.h"
const uint8_t LED_DATA_PIN = D5;
const uint8_t NUM_LEDS = 64;

int fadeAmount = 5;  // Set the amount to fade I usually do 5, 10, 15, 20, 25 etc even up to 255.
int brightness = 0; 

CRGB leds[NUM_LEDS];

// Init Heart Rate Sensor
MAX30105 pulss;

//
const int BUFFER_SIZE = 24;
uint32_t ir_values_buffer[BUFFER_SIZE] = {};
uint32_t no_person_value = 0;

void tauno_heart_beat_setup();
void tauno_heart_beat_loop();



int32_t tauno_map(int32_t x, int32_t in_min, int32_t in_max, int32_t out_min, int32_t out_max);

void heart_beat_plotter_setup();
void heart_beat_plotter_loop();
void presence_sensing_setup();
void presence_sensing_loop();
void temp_sense_setup();
void temp_sense_loop();
void heart_rate_setup();
void heart_rate_loop();
void max_speed_setup();
void max_speed_loop();


// Presense
int32_t samplesTaken = 0;  // Counter for calculating the Hz or read rate
int32_t unblockedValue;  // Average IR at power up
uint64_t startTime;  // Used to calculate measurement rate


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

  FastLED.addLeds<NEOPIXEL, LED_DATA_PIN>(leds, NUM_LEDS);

  // Configure Heart Rate Sensor
  // Select one:
  tauno_heart_beat_setup();
  // heart_beat_plotter_setup();
  // presence_sensing_setup();
  // temp_sense_setup();
  // heart_rate_setup();
  // max_speed_setup();
}

void loop() {
  tauno_heart_beat_loop();

/*
  int numLedsToLight = tauno_map(val, in_min, in_max, 0, 64);
  DEBUG_PRINT(" LEDS ");
  DEBUG_PRINT(numLedsToLight);
  */



  // First, clear the existing led values
 // FastLED.clear();
  //for (int led = 0; led < numLedsToLight; led++) {
  //  leds[led] = CRGB::Blue;
  //}
  /*
  if (val > 90000) {
    leds[0] = CRGB::Red;
    FastLED.show();
  } else {
    leds[0] = CRGB::Black;
    FastLED.show();
  }
  */

  // heart_beat_plotter_loop();
  // presence_sensing_loop();
  // temp_sense_loop();
  // heart_rate_loop();
  // max_speed_loop();
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
  DEBUG_PRINT(" RED");
  DEBUG_PRINT(pulss.getRed());
  DEBUG_PRINT(" IR");
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
// Heart rate
  const byte RATE_SIZE = 4; //Increase this for more averaging. 4 is good.
  static int8_t rates[RATE_SIZE]; //Array of heart rates
  static int8_t rateSpot = 0;
  static uint64_t lastBeat = 0; //Time at which the last beat occurred

  static float beatsPerMinute;
  static int beatAvg;

  int32_t irValue = pulss.getIR();

  if (checkForBeat(irValue) == true) {
    // We sensed a beat!
    uint32_t delta = millis() - lastBeat;
    lastBeat = millis();

    beatsPerMinute = 60 / (delta / 1000.0);

    if (beatsPerMinute < 255 && beatsPerMinute > 20) {
      rates[rateSpot++] = (byte)beatsPerMinute;  // Store this reading in the array
      rateSpot %= RATE_SIZE;  // Wrap variable

      // Take average of readings
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


void max_speed_setup() {
  // Heart Rate Sensor Settings
  uint8_t led_brightness = 0xFF;  // Options: 0=Off to 255=50mA
  // Set to 1 for max speed.
  uint8_t sample_average = 1;  // Options: 1, 2, 4, 8, 16, 32
  // ledMode must be 1 for 3200Hz. If ledMode is set to 2 max is 1600Hz.
  uint8_t led_mode = 1;  // Options: 1 = Red, 2 = Red + IR, 3 = Red + IR + Green

  int sample_rate = 3200;  // Options: 50, 100, 200, 400, 800, 1000, 1600, 3200
  // The pulsewidth must be as short as possible.
  int pulse_width = 69;  // Options: 69, 118, 215, 411
  int adc_range = 16384;  // Options: 2048, 4096, 8192, 16384

  // Configure Heart Rate Sensor
  pulss.setup(led_brightness,
              sample_average,
              led_mode,
              sample_rate,
              pulse_width,
              adc_range);
}

void max_speed_loop() {
  uint8_t samplesTaken = 0;
  uint64_t startTime = micros();

  while(samplesTaken < 10) {
    pulss.check();  // Check the sensor, read up to 3 samples
    while (pulss.available()) {
      samplesTaken++;
      pulss.getFIFOIR();
      pulss.nextSample();  // We're finished with this sample so move to next sample
    }
  }

  long endTime = micros();

  // Serial.print("samples ");
  // Serial.print(samplesTaken);

  Serial.print(" ");
  Serial.print((float)samplesTaken / ((endTime - startTime) / 1000000.0), 2);
  Serial.print("Hz ");

  DEBUG_PRINT("RED ");
  DEBUG_PRINT(pulss.getRed());

  Serial.println();
}


int32_t tauno_map(int32_t x, int32_t in_min, int32_t in_max, int32_t out_min, int32_t out_max) {
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

/*
*/
void tauno_heart_beat_setup() {
  // Heart Rate Sensor Settings

  // LED Pulse Amplitude Configuration
  // Options: 0=Off to 255=50mA
  // Default is 0x1F which gets us 6.4mA
  // powerLevel = 0x02, 0.4mA - Presence detection of ~4 inch
  // powerLevel = 0x1F, 6.4mA - Presence detection of ~8 inch
  // powerLevel = 0x7F, 25.4mA - Presence detection of ~8 inch
  // powerLevel = 0xFF, 50.0mA - Presence detection of ~12 inch
  uint8_t power_level = 0x1F;

  uint8_t sample_average = 8;  // Options: 1, 2, 4, 8, 16, 32

  // LEDS:
  // 1 = Red
  // 2 = Red + IR
  // 3 = Red + IR + Green
  // MAX30102 does not have a Green LED.
  uint8_t led_mode = 3;

  // Samples per second
  // Options: 50, 100, 200, 400, 800, 1000, 1600, 3200
  int sample_rate = 100;

  // The longer the pulse width the longer range of detection
  // 69us and 0.4mA it's about 2 inches
  // 118
  // 215
  // 411us and 0.4mA it's about 6 inches
  int pulse_width = 411;

  // Particle Sensing Configuration
  // 2048 - 7.81pA per LSB
  // 4096 - 15.63pA per LSB
  // 8192 - 31.25pA per LSB
  // 16384 - 62.5pA per LSB
  int adc_range = 4096;

  // Configure Heart Rate Sensor
  int setup_completed = pulss.setup(power_level,
                                   sample_average,
                                   led_mode,
                                   sample_rate,
                                   pulse_width,
                                   adc_range);
  if (setup_completed) {
    DEBUG_PRINTLN("MAX30102 Setup done!");
  }

  const uint8_t samples = 30;

  for (uint8_t i = 0; i < samples; i++) {
    no_person_value += pulss.getIR();
  }
  no_person_value /= samples;
}

/*
 Add value to the end off array
 and shift other to the left
*/
void add_to_buffer(uint32_t new_val) {
  // Shift elements to the left
  for (int i = 0; i < BUFFER_SIZE - 1; i++) {
    ir_values_buffer[i] = ir_values_buffer[i + 1];
  }

  // Add new element at the end
  ir_values_buffer[BUFFER_SIZE - 1] = new_val;
}

/*
 Print the array
*/ 
void print_buffer() {
    DEBUG_PRINT("Buffer: ");
    for (int i = 0; i < BUFFER_SIZE; i++) {
      DEBUG_PRINT(ir_values_buffer[i]);
      DEBUG_PRINT(" ");
    }
    DEBUG_PRINT("\n");
}

/*
 Returns the lowest value in array
*/
uint32_t find_min() {
  // Assume the first element is the minimum
  uint32_t min = ir_values_buffer[0];

  // Iterate through the array starting from the second element
  for (int i = 1; i < BUFFER_SIZE; i++) {
    // Update min if a smaller value is found
    if (ir_values_buffer[i] < min) {
      min = ir_values_buffer[i];
    }
  }

  return min;
}

/*
 Returns the highest value in array
*/
uint32_t find_max() {
  // Assume the first element is the maximum
  uint32_t max = ir_values_buffer[0];

  // Iterate through the array starting from the second element
  for (int i = 1; i < BUFFER_SIZE; i++) {
    // Update max if a larger value is found
    if (ir_values_buffer[i] > max) {
      max = ir_values_buffer[i];
    }
  }

  return max;
}

void tauno_heart_beat_loop() {
  uint32_t ir_val = pulss.getIR();
  // DEBUG_PRINT("IR ");
  // DEBUG_PRINT(ir_val);

  add_to_buffer(ir_val);
  // print_buffer();
  uint32_t in_min = find_min();
  uint32_t in_max = find_max();

  int numLedsToLight = tauno_map(ir_val, in_min, in_max, 0, NUM_LEDS);
  DEBUG_PRINT(" LEDS ");
  DEBUG_PRINT(numLedsToLight);
  DEBUG_PRINT("\n")

  if (ir_val > (no_person_value+120)) {
    FastLED.clear();
    for (int led = 0; led < numLedsToLight; led++) {
      if (led == (numLedsToLight-1)) {
        leds[led].setRGB(10, 0, 0);
      } else if (led == (numLedsToLight-2)) {
        leds[led].setRGB(20, 0, 0);
      } else if (led == (numLedsToLight-3)) {
        leds[led].setRGB(30, 0, 0);
      } else if (led == (numLedsToLight-4)) {
        leds[led].setRGB(40, 0, 0);
      } else {
        leds[led].setRGB(50, 0, 0);
      }

      FastLED.show();
    }
  } else {
    FastLED.clear();
    FastLED.show();
  }
}
