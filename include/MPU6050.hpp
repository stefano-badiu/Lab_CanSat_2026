#pragma once
#include "Sputnik_Identity.hpp"
#include <stdint.h>
extern Telemetry current_data;

bool init_MPU6050();
void read_MPU6050();
bool is_MPU6050_ready();

struct MPU6050CalibrationData {
    float biasX;
    float biasY;
    float biasZ;
    float scaleX;
    float scaleY;
    float scaleZ;
    bool valid;
    
};
bool calibrate_MPU6050_accel(uint16_t sampleCount, uint16_t sampleDelayMs);
bool is_MPU6050_calibrated();
void reset_MPU6050_calibration();
MPU6050CalibrationData get_MPU6050_calibration_data(); // Restituisce i dati di calibrazione attuali, se validi. Se non calibrato, restituisce valori di default con valid=false.
void set_MPU6050_calibration_data(const MPU6050CalibrationData& calibData); // Permette di impostare manualmente i dati di calibrazione, ad esempio da valori salvati in precedenza. Se calibData.valid è false, ignora i dati e non applica alcuna calibrazione.
