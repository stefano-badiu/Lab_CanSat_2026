#include "BMP280.hpp"
#include <Wire.h> //Serve per attivare la comunicazione sui pin A4 (SDA) e A5 (SCL) dell'Arduino Nano, che sono dedicati al bus I2C. 
#include <Adafruit_Sensor.h>
#include <Adafruit_BMP280.h> //Contiene i comandi specifici per il chip BMP280
extern Telemetry current_data;

Adafruit_BMP280 bmp;  // indica che ho creato un oggetto virtuale che rappresenta il chip fisico (l'ho capito così)

// Variabile globale del file per la pressione di riferimento
float P_0 = 1013.25;

bool init_BMP280(){
    if (! bmp.begin(0x76)) {
        //Serial.println("ERRORE: BMP280 non trovato!"); // Messaggio per il team 
        return(false); }
    else  {
        // Configurazioni standard del sensore 
        bmp.setSampling(Adafruit_BMP280::MODE_NORMAL,     
                        Adafruit_BMP280::SAMPLING_X2,   //temperatura: campionamento standard  
                        Adafruit_BMP280::SAMPLING_X16, //il sensore fa 16 micro-letture interne e ne fa la media prima di dare il risultato  
                        Adafruit_BMP280::FILTER_X16,  //filtro matematico che "ammorbidisce" i cambiamenti bruschi.    
                        Adafruit_BMP280::STANDBY_MS_1); //tempo di riposo tra una misura automatica e l'altra
        return(true);  // questa è l'inizializzazione, avrò come risultato V/F.
}
}

bool calibration_BMP280(){
    // facciamo 15 letture e calcoliamo la media per avere un valore stabile
    float somma_P0 = 0;
    for(int i = 0; i < 15; i++) {
        // leggiamo in Pascal e convertiamo in hPa per coerenza con la formula di sotto (/100)
        somma_P0 += (bmp.readPressure() / 100.0F); 
        delay(50);
    }
    
    // calcolo della pressione media al suolo
    P_0 = somma_P0 / 15.0F; 

    return true;
}


void read_BMP280(){
    float current_pressure = bmp.readPressure()/100.0F; // nella formula mi serve la Pressione trovata dal sensore, essa viene misurata in Pa, a me serve in hPa, perciò /100
    float base = current_pressure/P_0; //calcoliamo il rapporto rispetto alla pressione di terra (P/P0)
    float altitude = 44330.0*(1.0- pow(base,(1.0/5.255)));
 
    // aggiornamento della telemetria globale
    current_data.TEMPERATURE = bmp.readTemperature();
    current_data.ALTITUDE= altitude; //assegno il dato raccolto nella raccolta


}