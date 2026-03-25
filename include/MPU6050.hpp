#pragma once

#include "Sputnik_Identity.hpp"

extern Telemetry current_data;

bool init_MPU6050();
void read_MPU6050();
bool is_MPU6050_ready();
