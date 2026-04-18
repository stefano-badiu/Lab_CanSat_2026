#include <Arduino.h>   // Rende disponibili Serial, tipi Arduino e macro utili
#include <Wire.h>      // Libreria per la comunicazione I2C con il sensore
#include <math.h>      // Serve per funzioni matematiche come sqrtf() e acosf()
#include "MPU6050.hpp" // Header del modulo MPU6050



static constexpr uint8_t kMpuAddress = 0x68;     // Indirizzo I2C standard del MPU6050
static constexpr float kRadToDeg = 57.2957795F;  // Fattore di conversione da radianti a gradi
static bool mpuReady = false;                    // Variabile che indica se il sensore è stato inizializzato correttamente

static int16_t readAxisRegister(uint8_t registerAddress) {   // Funzione interna che legge 2 byte da un registro del sensore
    Wire.beginTransmission(kMpuAddress);                     // Avvia la comunicazione I2C con il MPU6050
    Wire.write(registerAddress);                             // Dice al sensore quale registro vogliamo leggere

    if (Wire.endTransmission(false) != 0) {                  // Invia il registro senza chiudere il bus I2C
        return 0;                                            // Se c'è errore nella trasmissione, restituisce 0
    }

    if (Wire.requestFrom(static_cast<int>(kMpuAddress), 2) != 2) {  // Chiede 2 byte consecutivi al sensore
        return 0;                                                    // Se non arrivano esattamente 2 byte, errore
    }

    const int16_t highByte = Wire.read();                    // Legge il byte alto del dato
    const int16_t lowByte = Wire.read();                     // Legge il byte basso del dato

    return static_cast<int16_t>((highByte << 8) | lowByte); // Ricompone i due byte in un solo valore signed a 16 bit
}

static float clampUnit(float value) {        // Funzione interna che limita il valore tra -1 e +1
    if (value > 1.0F) return 1.0F;           // Se il valore supera +1, lo forza a +1
    if (value < -1.0F) return -1.0F;         // Se il valore scende sotto -1, lo forza a -1
    return value;                            // Altrimenti restituisce il valore originale
}

bool init_MPU6050() {                        // Funzione di inizializzazione del sensore
    Wire.beginTransmission(kMpuAddress);     // Inizia una comunicazione con il MPU6050
    Wire.write(0x6B);                        // Seleziona il registro PWR_MGMT_1, usato per la gestione alimentazione
    Wire.write(0x00);                        // Scrive 0x00 per togliere il sensore dalla modalità sleep

    if (Wire.endTransmission() != 0) {       // Se la scrittura fallisce, il sensore non risponde correttamente
        Serial.println(F("ERRORE: MPU6050 non trovato.")); // Messaggio diagnostico sulla seriale
        mpuReady = false;                    // Segna il sensore come non pronto
        return false;                        // Ritorna false per indicare inizializzazione fallita
    }

    mpuReady = true;                         // Se tutto va bene, il sensore viene segnato come pronto
    return true;                             // Ritorna true per indicare inizializzazione riuscita
}

void read_MPU6050() {                        // Funzione che legge i dati del sensore e aggiorna la telemetria
    if (!mpuReady) return;                   // Se il sensore non è pronto, esce subito senza fare nulla

    float ax = static_cast<float>(readAxisRegister(0x3B)) / 16384.0F; // Legge accelerazione asse X e la converte in g
    float ay = static_cast<float>(readAxisRegister(0x3D)) / 16384.0F; // Legge accelerazione asse Y e la converte in g
    float az = static_cast<float>(readAxisRegister(0x3F)) / 16384.0F; // Legge accelerazione asse Z e la converte in g

    if (is_MPU6050_calibrated()) {              // Se il sensore è calibrato, applica i dati di calibrazione
        const MPU6050CalibrationData calib = get_MPU6050_calibration_data(); // Ottiene i dati di calibrazione
        ax = (ax - calib.biasX) * calib.scaleX; // Applica bias e scale all'asse X
        ay = (ay - calib.biasY) * calib.scaleY; // Applica bias e scale all'asse Y
        az = (az - calib.biasZ) * calib.scaleZ; // Applica bias e scale all'asse Z
    }

    const float magnitude = sqrtf((ax * ax) + (ay * ay) + (az * az)); // Calcola il modulo del vettore accelerazione nello spazio 3D

    if (magnitude <= 0.0F) return;           // Se il modulo è nullo o non valido, evita divisioni per zero

    current_data.TILT_X = acosf(clampUnit(ax / magnitude)) * kRadToDeg; // Calcola l'angolo rispetto all'asse X e lo converte in gradi
    current_data.TILT_Y = acosf(clampUnit(ay / magnitude)) * kRadToDeg; // Calcola l'angolo rispetto all'asse Y e lo converte in gradi
    current_data.TILT_Z = acosf(clampUnit(az / magnitude)) * kRadToDeg; // Calcola l'angolo rispetto all'asse Z e lo converte in gradi
    current_data.ACC_X = ax; // Aggiorna la telemetria con l'accelerazione grezza sull'asse X
    current_data.ACC_Y = ay; // Aggiorna la telemetria con l'accelerazione grezza sull'asse Y
    current_data.ACC_Z = az; // Aggiorna la telemetria con l'accelerazione grezza sull'asse Z

}

bool is_MPU6050_ready() {                    // Funzione che permette al resto del programma di sapere se il sensore è attivo
    return mpuReady;                         // Restituisce true se il sensore è inizializzato, false altrimenti
}

static MPU6050CalibrationData calibrationData = {0, 0, 0, 1, 1, 1, false}; // Dati di calibrazione predefiniti (non validi)

void reset_MPU6050_calibration() {                 // Funzione che resetta i dati di calibrazione ai valori di default
    calibrationData = {0, 0, 0, 1, 1, 1, false}; // Bias a 0, scale a 1, valid=false
}

bool is_MPU6050_calibrated() {                     // Funzione che indica se il sensore è stato calibrato
    return calibrationData.valid;              // Restituisce true se i dati di calibrazione sono validi, false altrimenti
}
MPU6050CalibrationData get_MPU6050_calibration_data() { // Funzione che restituisce i dati di calibrazione attuali
    return calibrationData;                   // Restituisce la struttura con i dati di calibrazione (bias, scale, valid)
}

void set_MPU6050_calibration_data(const MPU6050CalibrationData& calibData) { // Funzione che permette di impostare manualmente i dati di calibrazione
    if (!calibData.valid) return;            // Se i dati passati non sono validi, ignora la richiesta

    calibrationData = calibData;             // Aggiorna i dati di calibrazione con quelli forniti
}

bool calibrate_MPU6050_accel(uint16_t sampleCount, uint16_t sampleDelayMs) { // Funzione che esegue la calibrazione dell'accelerometro
    if (!mpuReady) return false;            // Se il sensore non è pronto, non può essere calibrato
    if (sampleCount == 0) return false;     // Se il numero di campioni è zero, non ha senso procedere

    float sumX = 0, sumY = 0, sumZ = 0;    // Variabili per accumulare le somme delle letture degli assi
    for (uint16_t i = 0; i < sampleCount; ++i) { // Ciclo per acquisire un certo numero di campioni
        sumX += static_cast<float>(readAxisRegister(0x3B)) / 16384.0F; // Legge e accumula l'accelerazione asse X
        sumY += static_cast<float>(readAxisRegister(0x3D)) / 16384.0F; // Legge e accumula l'accelerazione asse Y
        sumZ += static_cast<float>(readAxisRegister(0x3F)) / 16384.0F; // Legge e accumula l'accelerazione asse Z
        delay(sampleDelayMs); // Attende un breve intervallo tra le letture per stabilizzare i dati
    }

    calibrationData.biasX = sumX / sampleCount; // Calcola il bias medio sull'asse X
    calibrationData.biasY = sumY / sampleCount; // Calcola il bias medio sull'asse Y
    calibrationData.biasZ = (sumZ / sampleCount) - 1.0F; // Calcola il bias medio sull'asse Z, sottraendo 1g per compensare la gravità

    calibrationData.scaleX = 1.0F;
    calibrationData.scaleY = 1.0F;
    calibrationData.scaleZ = 1.0F;
    calibrationData.valid = true;


    return true;                             // Restituisce true per indicare che la calibrazione è stata completata con successo
    
}
