#include <SoftwareSerial.h>
#include <Arduino.h>
#include <Wire.h>
#include "Sputnik_Identity.hpp"
#include "BMP280.hpp"
#include "MPU6050.hpp"
#include "GPS_PA6H.hpp"
#include "Xbee_S2C.hpp"
#include "MicroSD.hpp"

// --- INTERRUTTORE DI MISSIONE ---
#define MODO_TEST // %%%%%%%%%%%%Commenta questa riga per il lancio reale%%%%%%%%%%%%%

Telemetry current_data;
FSM currentState = STATE_IDLE; 

void setup() {
 // inizializzazione Hardware Universale
Wire.begin(); 
Serial.begin(9600);   
current_data.TEAM_ID = 33; 
current_data.MISSION_TIME = 0;
current_data.STATE = currentState;
current_data.ALTITUDE = 0;
current_data.TEMPERATURE = 0;
current_data.GPS_LATITUDE = 0;
current_data.GPS_LONGITUDE = 0;
current_data.GPS_SATS = 0;
current_data.TILT_X = 0;
current_data.TILT_Y = 0;
current_data.TILT_Z = 0;
current_data.ACC_X = 0;
current_data.ACC_Y = 0;
current_data.ACC_Z = 0;

    #ifdef MODO_TEST
        // Logica specifica per il TEST (PC)
        while (!Serial); // Aspetta che si accenda il monitor seriale
        Serial.println(F("--- SPUTNIK-33cl: SYSTEM CHECK (TEST) ---"));
        if (!init_GPS()) {
            Serial.println(F("AVVISO: GPS non risponde."));
        } else {
            Serial.println(F("GPS: Inizializzato (9600 baud)."));
        }
        
        if (!init_BMP280()) {
            Serial.println(F("ERRORE CRITICO: BMP280 non trovato!"));
        } else {
            Serial.println(F("BMP280: OK"));
        }

        if(!init_MPU6050()) {
            Serial.println(F("ERRORE: MPU6050 NON TROVATO"));
        } else {
            Serial.println(F("MPU6050: OK"));
        }
        if(!init_MicroSD()) {
            Serial.println(F("ERRORE: MicroSD NON TROVATA"));
        } else {
            Serial.println(F("MicroSD: OK"));
            if (!createLogFile()) {
                Serial.println(F("ERRORE: Impossibile creare il file di log."));
            } else {
                Serial.println("File di log creato con successo.");
                if (!writeLogHeader()) {
                    Serial.println(F("ERRORE: Impossibile scrivere l'intestazione del log."));
                } else {
                    Serial.println(F("Intestazione del log scritta con successo."));
                    flushLogFile(); // log pronto per la scrittura
                    Serial.println(F("Log file pronto per la scrittura."));
                }
            }
        }
    
        #else
        // 3. Logica specifica per il LANCIO (XBee/Autonomo)
        delay(2000); // Pausa di sicurezza per stabilizzare l'elettronica
       // init_BMP280(); // Inizializzazione
        init_GPS(); //Inizializzazione
    #endif
}

unsigned long lastTelemetryTime = 0;

void loop() {
    update_GPS_data();
    // Lettura dati (Sempre attiva)
    read_BMP280();//&&&&&&&&&&&&&
    read_MPU6050();
    // Qui aggiungeremo il resto dei sensori

    // 2. INVIO DATI OGNI SECONDO (1000ms)
    if (millis() - lastTelemetryTime >= 1000) {
        lastTelemetryTime = millis(); // Resetta il timer
        current_data.MISSION_TIME = millis(); // Aggiorna il tempo di missione
       // current_data.STATE = currentState; // Aggiorna lo stato attuale (da implementare la logica di transizione)

    #ifdef MODO_TEST
        // Telemetria Dettagliata per il TEST (PC)
        Serial.print(F(" | Lon: ")); Serial.print(current_data.GPS_LONGITUDE, 6);
        Serial.print(F(" | Lat: ")); Serial.print(current_data.GPS_LATITUDE, 6);
        Serial.print(F(" | Sats: ")); Serial.println(current_data.GPS_SATS);
        Serial.print(F("Altitudine: "));
        Serial.print(current_data.ALTITUDE);
        Serial.print(F(" m | Temp: "));
        Serial.print(current_data.TEMPERATURE);
        Serial.println(F(" *C"));
        Serial.print(F("TILT_X: "));
        Serial.print(current_data.TILT_X);
        Serial.print(F("  TILT_Y: "));
        Serial.print(current_data.TILT_Y);
        Serial.print(F("  TILT_Z: "));
        Serial.println(current_data.TILT_Z);
        Serial.print(F("ACC_X: "));
        Serial.print(current_data.ACC_X);
        Serial.print(F("  ACC_Y: "));
        Serial.print(current_data.ACC_Y);
        Serial.print(F("  ACC_Z: "));
        Serial.println(current_data.ACC_Z);
        
        
    #else
        // Telemetria CSV Compatta per la Ground Station (XBee)
        // Formato: TEAM_ID, MISSION_TIME, STATE, ALTITUDE, TEMP, ... %%%%%%%%DA RIVEDERE%%%%%%%%%%%
        Serial.print(current_data.TEAM_ID); Serial.print(",");
        Serial.print(current_data.MISSION_TIME); Serial.print(",");
        Serial.print(current_data.STATE); Serial.print(",");
        Serial.print(current_data.ALTITUDE); Serial.print(",");
        Serial.print(current_data.TEMPERATURE);Serial.print(",");
        Serial.print(","); Serial.print(current_data.GPS_LATITUDE, 6);
        Serial.print(","); Serial.print(current_data.GPS_LONGITUDE, 6);
        Serial.print(","); Serial.println(current_data.GPS_SATS);
       
    #endif
        if (is_MicroSD_ready()) {
            if (logTelemetry(current_data)) {
                flushLogFile();
                Serial.println("Dati di telemetria salvati sulla SD.");
            } else {
                Serial.println("ERRORE: Impossibile scrivere i dati sulla SD.");
            }
        } else {
            Serial.println("SD non pronta, impossibile salvare i dati.");
        }
    }
}
