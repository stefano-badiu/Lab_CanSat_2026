#include "MicroSD.hpp"
#include <Arduino.h>
#include <SD.h>  // Libreria per la gestione della scheda SD, che permette di leggere e scrivere file su di essa
#include <SPI.h> // Libreria per la comunicazione SPI, necessaria per interfacciarsi con la scheda SD

const int chipSelectPin = 10; // Pin CS (Chip Select) per la scheda SD, usato per attivare la comunicazione con essa
const char* filename = "telemetry.csv"; // Nome del file su cui verranno salvati i dati di telemetria
bool sdReady = false; // Variabile che indica se la scheda SD è stata inizializzata correttamente e pronta all'uso
bool headerWritten = false; // Serve a non riscrivere ogni volta la prima riga del CSV.
File logFile; // È il file della missione su cui verranno salvati i dati.

bool init_MicroSD() {
    if (sdReady) { // Se la SD è già stata inizializzata, non serve rifarlo
        return true;
    }

    if (SD.begin(chipSelectPin)) { // Prova a inizializzare la scheda SD usando il pin CS specificato
        sdReady = true; // Se tutto va bene, segna la SD come pronta
        headerWritten = false; // All'inizio di una nuova sessione considero l'header non ancora scritto
        return true; // Ritorna true per indicare che l'inizializzazione è riuscita
    } else { // Se fallisce
        Serial.println("ERRORE: MicroSD non trovata!"); // Stampa un messaggio di errore sulla seriale
        sdReady = false; // Segna la SD come non pronta
        return false; // Ritorna false per indicare che l'inizializzazione è fallita
    }
}

bool createLogFile() {
    if (!sdReady) { // Se la SD non è pronta, provo prima a inizializzarla
        if (!init_MicroSD()) {
            return false; // Se l'inizializzazione fallisce, non posso aprire il file
        }
    }

    if (logFile) { // Se il file è già aperto, non serve riaprirlo
        return true;
    }

    logFile = SD.open(filename, FILE_WRITE); // Apre il file in scrittura, oppure lo crea se non esiste

    if (!logFile) { // Se l'apertura fallisce
        Serial.println("ERRORE: Impossibile creare o aprire il file di log!");
        return false;
    }

    if (logFile.size() > 0) { // Se il file contiene già dati, considero l'header già presente
        headerWritten = true;
    } else { // Se il file è vuoto, l'header non è ancora stato scritto
        headerWritten = false;
    }

    return true; // Il file è pronto per essere usato
}

bool writeLogHeader(){
    if (!logFile) {
        if (!createLogFile()) { // Se il file non è aperto, provo prima a crearlo se non esiste
        Serial.println("ERRORE: Impossibile creare o aprire il file di log!");
        return false;
        }    
   }
   if (headerWritten) { // Se l'header è già stato scritto, non serve riscriverlo
        return true;
    
    }

    logFile.println("TEAM_ID,MISSION_TIME,STATE,ALTITUDE,TEMPERATURE,GPS_LATITUDE,GPS_LONGITUDE,GPS_SATS,TILT_X,TILT_Y,TILT_Z,ACC_X,ACC_Y,ACC_Z"); // Scrivo la prima riga del CSV con i nomi delle colonne
    headerWritten = true; // Segno che l'header è stato scritto
    return true; // L'header è stato scritto con successo
}

bool logTelemetry(const Telemetry& data) {
    if (!logFile) { // Se il file non è aperto, provo prima a crearlo se non esiste
        if (!createLogFile()) {
            Serial.println("ERRORE: Impossibile creare o aprire il file di log!");
            return false;
        }
    }

    if (!headerWritten) { // Se l'header non è stato scritto, lo scrivo prima di tutto
        if (!writeLogHeader()) {
            Serial.println("ERRORE: Impossibile scrivere l'header nel file di log!");
            return false;
        }
    }

    // Scrivo una riga con i dati attuali, separando i valori con virgole per formare un CSV
    logFile.print(data.TEAM_ID);
    logFile.print(",");
    logFile.print(data.MISSION_TIME);
    logFile.print(",");
    logFile.print(fsmToString(data.STATE)); // Converto lo stato FSM in stringa per salvarlo nel CSV
    logFile.print(",");
    logFile.print(data.ALTITUDE);
    logFile.print(",");
    logFile.print(data.TEMPERATURE);
    logFile.print(",");
    logFile.print(data.GPS_LATITUDE, 6); // Scrivo la latitudine con 6 cifre decimali
    logFile.print(",");
    logFile.print(data.GPS_LONGITUDE, 6); // Scrivo la longitudine con 6 cifre decimali
    logFile.print(",");
    logFile.print(data.GPS_SATS);
    logFile.print(",");
    logFile.print(data.TILT_X);
    logFile.print(",");
    logFile.print(data.TILT_Y);
    logFile.print(",");
    logFile.print(data.TILT_Z); 
    logFile.print(",");
    logFile.print(data.ACC_X);
    logFile.print(",");
    logFile.print(data.ACC_Y);
    logFile.print(",");
    logFile.println(data.ACC_Z);
    
    return true; // La riga è stata scritta con successo
}

void flushLogFile() {
    if (logFile) { // Se il file è aperto
        logFile.flush(); // Forza il salvataggio dei dati sulla SD
    }   
}

void closeLogFile() {
    if (logFile) { // Se il file è aperto
        logFile.close(); // Chiude il file per assicurarsi che tutti i dati siano salvati correttamente
    }
}
bool is_MicroSD_ready() {
    return sdReady; // Ritorna true se la SD è pronta da usare, false altrimenti
}
