/*
 Copyright 2024
 Tauno Erik
 Started 01.03.2024
 Edited  11.03.2024
 MAX30102 Heart Rate Sensor
 ESP8266 MCU

 MAX30102 does not have a Green LED.

 D1 - GPIO4 - SDA
 D2 - GPIO5 - SCL
 D6 - GPIO14 - INT
 D5 - GPIO12 - RGB LED

 https://forum.arduino.cc/t/managing-serial-print-as-a-debug-tool/1024824
*/

#include <Arduino.h>
#include <Wire.h>       // I2C
#include "MAX30105.h"   // SparkFun MAX3010x
#include "heartRate.h"  // SparkFun MAX3010x

#define TAUNO_DEBUG 0
#include "tauno_debug.h"

#include "FastLED.h"
#define ON  1
#define OFF 0
const uint8_t LED_DATA_PIN = D5;
const uint8_t NUM_BEAMS = 9;
const uint8_t NUM_LEDS_IN_BEAM = 12;
const uint8_t NUM_OF_LEDS = NUM_LEDS_IN_BEAM*NUM_BEAMS;

CRGB leds[NUM_OF_LEDS];

// Init Heart Rate Sensor
MAX30105 pulss;

//
const int BUFFER_SIZE = 24;
uint32_t ir_values_buffer[BUFFER_SIZE] = {};
uint32_t no_person_value = 0;

void tauno_aura_analyzer();
void tauno_heart_beat_setup();  //  Setup sensor
void tauno_heart_beat_loop();   // Main loop
uint32_t find_min();            // find min from ir_values_buffer
uint32_t find_max();            // find max from ir_values_buffer
void print_buffer();
int32_t tauno_map(int32_t x,
                  int32_t in_min,
                  int32_t in_max,
                  int32_t out_min,
                  int32_t out_max);


// other examples
void temp_sense_setup();
void temp_sense_loop();


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

  FastLED.addLeds<NEOPIXEL, LED_DATA_PIN>(leds, NUM_OF_LEDS);

  tauno_heart_beat_setup();
}

void loop() {
  tauno_aura_analyzer();
  // tauno_heart_beat_loop();
}


void temp_sense_setup() {
  // Turn off LEDs
  pulss.setup(0);
  // Enable the temp ready interrupt. This is required.
  pulss.enableDIETEMPRDY();
}

void temp_sense_loop() {
  float temperature = pulss.readTemperature();

  DEBUG_PRINT("tempC=");
  DEBUG_PRINT(temperature, 4);

  DEBUG_PRINTLN();
}


/*
 map function
*/
int32_t tauno_map(int32_t x,
                  int32_t in_min,
                  int32_t in_max,
                  int32_t out_min,
                  int32_t out_max) {
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

/*
 Setup the sensor settings
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
 Add value to the end off IR values buffer array
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
 Print the IR values buffer array
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
 Returns the lowest value in IR values buffer array
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
 Returns the highest value in IR values buffer array
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

/*
 The main loop!
*/
void tauno_heart_beat_loop() {
  static uint8_t hue = 250;  // 0-255
  static uint8_t saturation = 200;  // 0-255
  static uint8_t value = 50;  // 0-255

  uint32_t ir_val = pulss.getIR();
  DEBUG_PRINT("IR ");
  DEBUG_PRINT(ir_val);

  add_to_buffer(ir_val);
  // print_buffer();
  uint32_t in_min = find_min();
  uint32_t in_max = find_max();

  uint8_t numLedsToLight = tauno_map(ir_val, in_min, in_max, 0, NUM_LEDS_IN_BEAM);
  DEBUG_PRINT(" LEDS ");
  DEBUG_PRINT(numLedsToLight);
  DEBUG_PRINT("\n")

  if (ir_val > (no_person_value+120)) {
    FastLED.clear();

    for (uint8_t led = 0; led < numLedsToLight; led++) {
      leds[led] = CHSV(hue, saturation, value);
      leds[(1*NUM_LEDS_IN_BEAM) + led] = CHSV(hue, saturation, value);
      leds[(2*NUM_LEDS_IN_BEAM) + led] = CHSV(hue, saturation, value);
      leds[(3*NUM_LEDS_IN_BEAM) + led] = CHSV(hue, saturation, value);
      leds[(4*NUM_LEDS_IN_BEAM) + led] = CHSV(hue, saturation, value);
      leds[(5*NUM_LEDS_IN_BEAM) + led] = CHSV(hue, saturation, value);
      leds[(6*NUM_LEDS_IN_BEAM) + led] = CHSV(hue, saturation, value);
      leds[(7*NUM_LEDS_IN_BEAM) + led] = CHSV(hue, saturation, value);
      leds[(8*NUM_LEDS_IN_BEAM) + led] = CHSV(hue, saturation, value);
      FastLED.show();
    }

  } else {
    FastLED.clear();
    FastLED.show();
  }
}


/*
 main loop
 Biovälja nalüsaator
*/
void tauno_aura_analyzer() {
  static uint8_t hue = 250;  // 0-255
  static uint8_t saturation = 220;  // 0-255
  static uint8_t value = 60;  // 0-255

  uint32_t ir_val = pulss.getIR();
  DEBUG_PRINT("IR ");
  DEBUG_PRINT(ir_val);

  hue = ir_val % 255;
  DEBUG_PRINT(" modulo ");
  DEBUG_PRINT(hue);

  add_to_buffer(ir_val);
  // print_buffer();
  uint32_t in_min = find_min();
  uint32_t in_max = find_max();

  uint8_t numLedsToLight = tauno_map(ir_val, in_min, in_max, 0, NUM_LEDS_IN_BEAM);
  DEBUG_PRINT(" LEDS ");
  DEBUG_PRINT(numLedsToLight);
  DEBUG_PRINT("\n")

  if (ir_val > (no_person_value+120)) {
    FastLED.clear();

    for (uint8_t led = 0; led < numLedsToLight; led++) {
      leds[led] = CHSV(hue, saturation, value);
      leds[(1*NUM_LEDS_IN_BEAM) + led] = CHSV(hue, saturation, value);
      leds[(2*NUM_LEDS_IN_BEAM) + led] = CHSV(hue, saturation, value);
      leds[(3*NUM_LEDS_IN_BEAM) + led] = CHSV(hue, saturation, value);
      leds[(4*NUM_LEDS_IN_BEAM) + led] = CHSV(hue, saturation, value);
      leds[(5*NUM_LEDS_IN_BEAM) + led] = CHSV(hue, saturation, value);
      leds[(6*NUM_LEDS_IN_BEAM) + led] = CHSV(hue, saturation, value);
      leds[(7*NUM_LEDS_IN_BEAM) + led] = CHSV(hue, saturation, value);
      leds[(8*NUM_LEDS_IN_BEAM) + led] = CHSV(hue, saturation, value);

      FastLED.show();
    }

  } else {
    FastLED.clear();
    FastLED.show();
  }
}
