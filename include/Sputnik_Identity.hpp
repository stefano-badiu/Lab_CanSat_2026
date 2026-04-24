#pragma once  //serve a prendere il file una sola volta, così non lo legge più volte creando problemi

//VARIABILI
#define SEALEVELPRESSURE_HPA (1013.25)

enum FSM{
    STATE_IDLE, //esce quando rileva il lancio
    STATE_ASCENT,//esce quando abbiamo raggiunto l'apogeo
    STATE_DESCENT_FAST, // esce quando la quota<= 100m
    STATE_DESCENT_SLOW, // esce quando v di discesa è circa 0m/s per almeno due secondi
    STATE_LANDED // FINE MISSIONE
};

struct Telemetry{
    int TEAM_ID; //ID del team ho usato int perché sarà un numero intero
    unsigned long MISSION_TIME; // ho usato unsigned long perché mi serve un numero intero grande solo positivo, necessario per contenere i millisecondi di volo senza errori (fino a 50 giorni).
    FSM STATE; //Stato attuale della missione
    float ALTITUDE; //ovviamente servono i decimali per dati così, perciò userò float
    float TEMPERATURE; // come prima
    double GPS_LATITUDE; // double è per aggiungere accuratezza ad un numero float
    double GPS_LONGITUDE; // come prima
    int GPS_SATS; // il numero di satelliti è un numero intero
    double TILT_X;
    double TILT_Y;
    double TILT_Z;
    float ACC_X;
    float ACC_Y;
    float ACC_Z;
    float BATTERY_VOLTAGE_V; 
    float BATTERY_CURRENT_mA; 
    float BATTERY_POWER_mW; 
    float BATTERY_CONSUMED_mWH; 
    float BATTERY_REMAINING_PCT; 
    //potremmo voler includere altri dati, ne discuteremmo
};
const char* fsmToString(FSM state); // Converte uno stato della FSM in una stringa leggibile
