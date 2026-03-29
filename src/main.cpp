#include <Arduino.h>
#include "../include/Sputnik_Identity.hpp"    //../include, aggiunto per accedere alle librerie nella cartella include (non bastava #include non so perchè) 
#include "../include/BMP280.hpp"
#include "../include/MPU6050.hpp"
#include "../include/GPS_PA6H.hpp"
#include "../include/Xbee_S2C.hpp"
#include "../include/MicroSD.hpp"
#include <Wire.h>



Telemetry current_data;
FSM currentState = STATE_IDLE;

bool bmp_ok = false;
bool mpu_ok = false;

void setup() {
    Serial.begin(115200);
    Wire.begin();
    while (!Serial); // Attende che la porta seriale sia pronta (solo per test)
    
    Serial.println("--- SPUTNIK_1: TEST SISTEMI ---");

    bmp_ok = init_BMP280();
    if (bmp_ok) {
        Serial.println("BMP280 inizializzato con successo.");
    } else {
        Serial.println("AVVISO: BMP280 non risponde.");
    }

    mpu_ok = init_MPU6050();
    if (mpu_ok) {
        Serial.println("MPU6050 pronto");
    } else {
        Serial.println("MPU6050 NON trovato");
    }
}

void loop() {
    if (bmp_ok) {
        read_BMP280();
        Serial.print("Altitudine: ");
        Serial.print(current_data.ALTITUDE);
        Serial.print(" m | Temp: ");
        Serial.print(current_data.TEMPERATURE);
        Serial.println(" *C");
    }

    if (mpu_ok) {
        read_MPU6050();
        Serial.print("TILT_X: ");
        Serial.print(current_data.TILT_X);
        Serial.print("  TILT_Y: ");
        Serial.print(current_data.TILT_Y);
        Serial.print("  TILT_Z: ");
        Serial.println(current_data.TILT_Z);
    }

    delay(500);
}


