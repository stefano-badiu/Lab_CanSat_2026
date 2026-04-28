#include "MissionControl.hpp"
#include <Arduino.h>
#include <EEPROM.h>
#include <Servo.h>

bool manual_override = false;
unsigned long photoInterval = 0;
Servo servo_paracadute;


bool detect_launch() {
    static unsigned long inizio_decollo = 0;
    
    // Niente accelerometri: il drone sale dolcemente.
    // Usiamo una quota di sicurezza (es. 10 metri) per capire che il volo è iniziato.
    if (current_data.ALTITUDE > 10.0) {
        if (inizio_decollo == 0) {
            inizio_decollo = millis(); // Facciamo partire il cronometro
        } 
        // Se rimaniamo sopra i 10 metri per 2 secondi continui, siamo sicuramente in volo
        else if (millis() - inizio_decollo >= 2000) {
            return true; // Passa a STATE_ASCENT
        }
    } else {
        // Se era solo un falso allarme o il drone è riatterrato subito, azzera tutto
        inizio_decollo = 0;
    }
    
    return false;
}

int eps_a=2;
const unsigned long t_conferma_apogeo = 1000; // 1 secondo continuo di caduta

bool detect_apogee() {
    static float m = 0.0;
    static unsigned long inizio_caduta = 0;
    float new_altitude = current_data.ALTITUDE;
    
    if (new_altitude > m) { m = new_altitude; }
    
    if (m > (new_altitude + eps_a)) {
        if (inizio_caduta == 0) { 
            inizio_caduta = millis(); 
        }
        else if (millis() - inizio_caduta >= t_conferma_apogeo) { 
            return true; // APOGEO CONFERMATO
    } else {
        inizio_caduta = 0; 
    }
}
    
    return false; 
}

const float quota_target = 100.0;
const float compensazione_caduta = 10.0; 
const float soglia_sgancio = quota_target + compensazione_caduta; // 110 metri
// Aspettiamo 300ms. A 30 m/s, il CanSat perderà circa 9 metri prima dell'apertura.
// Aprirà quindi intorno ai 100 metri 
const unsigned long t_conferma_sgancio = 300;
bool detect_altitude() {
    static unsigned long inizio_sotto_quota = 0; 

    if (current_data.ALTITUDE <= soglia_sgancio) {
        if (inizio_sotto_quota == 0) {
            inizio_sotto_quota = millis(); // Start timer
        }
        
        if (millis() - inizio_sotto_quota >= t_conferma_sgancio) {
            return true; // OK: Time passed, confirm deploy
        }
    } 
    else {
        // RESET: Solo se la quota è tornata sopra la soglia
        inizio_sotto_quota = 0;
    }
    return false;   
}


const float eps_alt = 0.5; // Metri di tolleranza
const float eps_acc = 0.2; // Tolleranza accelerazione (m/s^2)
const unsigned long t_conferma = 5000; // 5 secondi 

bool detect_landing() {
    static unsigned long inizio = 0;
    static float altitudine_riferimento = 0;

    float acc_totale = sqrt((current_data.ACC_X* current_data.ACC_X) + (current_data.ACC_Y*current_data.ACC_Y) + (current_data.ACC_Z*current_data.ACC_Z));
    float diff_acc = abs(acc_totale - 1);

    float diff_alt = abs(current_data.ALTITUDE - altitudine_riferimento); // calcoliamo quanto l'altitudine è cambiata dall'ultimo controllo

    if (diff_alt < eps_alt && diff_acc < eps_acc) {
        // Se è la prima volta che lo vediamo fermo, facciamo partire il cronometro
        if (inizio == 0) {
            inizio = millis();
        }
        
        // Se è rimasto fermo per più di 5 secondi...
        if (millis() - inizio >= t_conferma) {
            return true; // ATTERRAGGIO CONFERMATO
        }
    } 
    else {
        // Se si muove, resetta tutto: il timer e la nuova altitudine di riferimento
        inizio = 0;
        altitudine_riferimento = current_data.ALTITUDE;
    }

    return false;
}


void init_mission_control() {
    servo_paracadute.attach(PIN_SERVO);
    servo_paracadute.write(0);
} 

void change_state(FSM state){
    current_data.STATE= state;
    EEPROM.put(0, current_data.STATE);
        //&&&&&&&&&&&&&&&&&stampa seriale per test&&&&&&&&&&&&&&&&&&&
        Serial.print("MISSION UPDATE: Passaggio allo stato ");
        Serial.println(state); 
    switch(state) {
        case STATE_IDLE:          photoInterval = 0;     break; // Ferma
        case STATE_ASCENT:        photoInterval = 5000; break; // 5 sec
        case STATE_DESCENT_FAST:  photoInterval = 2000;  break; // 2 sec
        case STATE_DESCENT_SLOW:  photoInterval = 1000;  break; // 1 sec
        case STATE_LANDED:        photoInterval = 0;     break; // Ferma
    }    


}


void update_mission_state(){
    if (manual_override == true) {
        return; 
    }
    switch(current_data.STATE) { 
        case STATE_IDLE:
        //azioni che dovrà eseguire in questo stato
            if(detect_launch()){
                change_state(STATE_ASCENT);
            }
        break;

        case STATE_ASCENT:
        //Azioni
            if(detect_apogee()){
                change_state(STATE_DESCENT_FAST);
            
            }
        break;

        case STATE_DESCENT_FAST:
        //azioni
            if(detect_altitude()){
                change_state(STATE_DESCENT_SLOW);
            }
        break;

        case STATE_DESCENT_SLOW:
        servo_paracadute.write(90);
        current_data.PARACHUTE_OPEN = true;
            if(detect_landing()){
                change_state(STATE_LANDED);
            }
        break;

        case STATE_LANDED:
        //azioni
        break;
    }

}


