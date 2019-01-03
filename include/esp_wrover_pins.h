//pin mappings to esp_wrover_kit
#include <Arduino.h>
const int LIGHT_SENSOR_PIN PROGMEM = A0;
const int DHT11_DATA_PIN PROGMEM = 4;
const int I2C_CLOCK_PIN PROGMEM = 26;
const int I2C_DATA_PIN PROGMEM = 27;
const int THERMOSTAT_HEAT_CALL_PIN PROGMEM = 32;
const int THERMOSTAT_COOL_CALL_PIN PROGMEM = 33;
//const int THERMOSTAT_FAN_CALL_PIN PROGMEM = 10;
const int TEST_PIN = 34;