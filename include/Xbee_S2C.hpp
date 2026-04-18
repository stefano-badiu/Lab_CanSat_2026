#pragma once
#include "Sputnik_Identity.hpp"
#include <Arduino.h>

class Xbee_S2C {
public: 
    
    Xbee_S2C(Stream& serialPort); // Inizializza passando il riferimento alla porta seriale (Hardware o Software)
    void sendTelemetry(const Telemetry& data); // Invia i dati di telemetria in formato CSV
    bool available(); // Verifica se ci sono comandi in arrivo
    String readData(); // Legge i dati in ingresso

private:
    Stream& serial;
};
