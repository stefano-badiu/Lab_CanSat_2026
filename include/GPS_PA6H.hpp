#pragma once
#include <SoftwareSerial.h>
#include <TinyGPS++.h>
#include "Sputnik_Identity.hpp"

extern Telemetry current_data; // Informa il driver che il posto dove inserire i dati esiste già (nel main) e non deve crearne uno.
//Definizione pin
const int GPS_RX_PIN = 4;  // L'Arduino riceve (RX) dal TX del GPS
const int GPS_TX_PIN = 3;  // L'Arduino trasmette (TX) al RX del GPS

extern SoftwareSerial gpsSerial;
extern TinyGPSPlus GPS;

bool init_GPS();
void update_GPS_data();