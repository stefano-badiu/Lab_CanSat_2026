#pragma once
#include "Sputnik_Identity.hpp"

bool init_MicroSD();  //Serve a inizializzare la scheda. Ritorno: true se la SD risponde, false se fallisce.

bool createLogFile();  //Serve a creare o aprire il file della missione. Ritorno: true se il file è pronto, false se non riesce.

bool writeLogHeader();  //Serve a scrivere la prima riga del CSV con i nomi delle colonne. Ritorno: true se scrive bene, false se fallisce.

bool logTelemetry(const Telemetry& data);  //Serve a scrivere una riga con i dati attuali. Parametro: un riferimento costante alla struct Telemetry. Ritorno: true se la riga è stata scritta, false se no.

void flushLogFile();  //Serve a forzare il salvataggio sulla SD ogni tanto.

void closeLogFile();  //Serve a chiudere bene il file a fine missione.

bool is_MicroSD_ready();  //Serve al main per sapere se la SD è disponibile.
