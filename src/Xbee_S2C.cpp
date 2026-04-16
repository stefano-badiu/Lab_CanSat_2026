#include "Xbee_S2C.hpp"
#include <Arduino.h>     // Serial, millis

Xbee_S2C::Xbee_S2C(Stream& serialPort) : serial(serialPort) {}

void Xbee_S2C::sendTelemetry(const Telemetry& data) {
    serial.print("$"); // Start marker: utile per la Ground Station
    serial.print(",");
    serial.print(data.TEAM_ID); serial.print(",");
    serial.print(data.MISSION_TIME); serial.print(",");
    serial.print(data.STATE); serial.print(",");
    serial.print(data.ALTITUDE); serial.print(",");
    serial.print(data.PRESSURE); serial.print(","); 
    serial.print(data.TEMPERATURE); serial.print(",");
    serial.print(data.GPS_LATITUDE, 6); serial.print(",");
    serial.print(data.GPS_LONGITUDE, 6); serial.print(",");
    serial.print(data.GPS_SATS); serial.print(",");
    serial.print(data.TILT_X); serial.print(",");
    serial.print(data.TILT_Y); serial.print(",");
    serial.print(data.TILT_Z); serial.print(",");
    serial.print(data.ACC_X); serial.print(",");
    serial.print(data.ACC_Y); serial.print(",");
    serial.println(data.ACC_Z); 
}

bool Xbee_S2C::available() {
    return serial.available() > 0;
}

String Xbee_S2C::readData() {
    if (serial.available() > 0) {
        return serial.readStringUntil('\n');
    }
    return "";
}
