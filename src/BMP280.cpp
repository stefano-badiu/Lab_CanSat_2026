#include "BMP280.hpp"
#include <Wire.h> //Serve per attivare la comunicazione sui pin A4 (SDA) e A5 (SCL) dell'Arduino Nano, che sono dedicati al bus I2C. 
#include <Adafruit_Sensor.h>
#include <Adafruit_BMP280.h> //Contiene i comandi specifici per il chip BME280
extern Telemetry current_data;

Adafruit_BMP280 bmp;  // indica che ho creato un oggetto virtuale che rappresenta il chip fisico (l'ho capito così)

bool init_BMP280(){
    if (! bmp.begin(0x76)) {
        Serial.println("ERRORE: BMP280 non trovato!"); // Messaggio per il team 
        return(false); }
    else  return(true);  // questa è l'inizializzazione, avrò come risultato V/F.
}

void read_BMP280(){
    float pressure = bmp.readPressure()/100.0F; // nella formula mi serve la Pressione trovata dal sensore, essa viene misurata in Pa, a me serve in hPa, perciò /100
    float base = pressure/SEALEVELPRESSURE_HPA; //per ora usiamo questo dato(SEALEVELPRESSURE), magari il giorno della competizione possiamo pure mandare il dato(P0) misurato direttamente lì per renderlo più preciso
    float risultato_h = 44330.0*(1.0- pow(base,(1.0/5.255)));
 
    current_data.TEMPERATURE = bmp.readTemperature();
    current_data.ALTITUDE=risultato_h; //assegno il dato raccolto nella raccolta


}