#include <Arduino.h>
#include "Sputnik_Identity.hpp"
#include "BME280.hpp"
#include "MPU6050.hpp"
#include "GPS_PA6H.hpp"
#include "Xbee_S2C.hpp"
#include "MicroSD.hpp"

Telemetry current_data;

FSM currentState = STATE_IDLE; //inizializzazione stato missione


void setup() {
  
}

void loop() {

}

