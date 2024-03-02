/*
 Copyright 2024 Tauno Erik
 Started 02.03.2024
 Edited  02.03.2024
*/
#ifndef TAUNO_DEBUG_H_
#define TAUNO_DEBUG_H_

#if TAUNO_DEBUG == 1
  #define DEBUG_SERIAL_BEGIN(...) Serial.begin(__VA_ARGS__);
  #define DEBUG_PRINT(...)        Serial.print(__VA_ARGS__);
  #define DEBUG_PRINTLN(...)      Serial.println(__VA_ARGS__);
  #define DEBUG_WRITE(...)        Serial.write(__VA_ARGS__);
#else
  #define DEBUG_PRINT(...)
  #define DEBUG_PRINT(...)
  #define DEBUG_PRINTLN(...)
  #define DEBUG_WRITE(...)
#endif

#endif  // TAUNO_DEBUG_H_
