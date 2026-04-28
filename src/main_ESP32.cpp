#include <Arduino.h>
#include "ESP32CAM.hpp" // La tua libreria personalizzata

// --- INTERRUTTORE DI MISSIONE ---
#define MOD_TEST // %%%%%%%%%%%%Commenta questa riga per il lancio reale%%%%%%%%%%%%%
// 1. Definiamo la nuova seriale hardware
// UART1 dell'ESP32: RX su pin 13, TX su pin 12 (opzionale)
HardwareSerial SerialNano(1); 
#define RX_NANO 13
#define TX_NANO 12

void setup() {
   // Porta di DEBUG (USB) - Alziamo la velocità per i log pesanti
    Serial.begin(115200);
    SerialNano.begin(38400, SERIAL_8N1, RX_NANO, TX_NANO);
    
    #ifdef MOD_TEST
        delay(1000); // Pausa per far stabilizzare il monitor seriale
        Serial.println("\n--- SPUTNIK-33cl: CAMERA SYSTEM CHECK (TEST) ---");
        
        if (init_camera_hardware()) {
            Serial.println("Hardware (Camera + SD): OK.");
            
            // ---> ATTIVAZIONE TEST VIDEO <---
            Serial.println("Avvio Test di Streaming per messa a fuoco...");
            setup_wifi_stream();
            // --------------------------------
            
            Serial.println("In attesa di ordini di scatto..."); // <- La tua riga originale
        } else {
            Serial.println("ERRORE CRITICO: Inizializzazione Hardware fallita.");
            Serial.println("Controllare la MicroSD e il montaggio della lente."); // <- La tua riga originale
        }
    #else
        // 2. Logica di avvio per il LANCIO (Modalità Silenziosa)
        delay(2000); // Pausa di sicurezza per far stabilizzare le tensioni della batteria
        init_camera_hardware(); 
    #endif
}

void loop() {
    // Buffer statico per la ricezione (addio classe String!)
    static char buffer[256];
    static int index = 0;
    
    while (SerialNano.available() > 0) {
        char c = SerialNano.read();

        // Se troviamo il carattere di fine riga (\n) o il buffer è pieno
        if (c == '\n' || index >= 127) {
            buffer[index] = '\0'; // Chiudiamo la stringa
            
            #ifdef MOD_TEST
                Serial.print("Dato ricevuto dal Nano: ");
                Serial.println(buffer);
            #endif
            
            // In volo o in test, il parser viene chiamato SEMPRE e solo UNA volta
            parse_incoming_data(buffer); 
            
            index = 0; // Resettiamo il buffer per il prossimo pacchetto
        } 
        else if (c != '\r') { // Ignoriamo il ritorno a capo di Windows
            buffer[index++] = c;
        }
    }
}