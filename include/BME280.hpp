#pragma once
#include "Sputnik_Identity.hpp"
extern Telemetry current_data; // Informa il driver che il posto dove inserire i dati esiste già (nel main) e non deve crearne uno.
bool init_BME280();  // con il comando bool ti aspetti che dalla funzione esca una condizione: V/F, essendo init_BME280 una funzione di inizializzazione, controlla solo se è tutto ok, mi basta sapere appunto se è ok:V, o meno:F
void read_BME280();  // con il comando void ... la funzione legge cosa succede dentro il sensore e ...
