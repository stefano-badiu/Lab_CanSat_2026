#include "Sputnik_Identity.hpp"

const char* fsmToString(FSM state) { //per convertire lo stato della FSM in una stringa leggibile, così da poterla salvare sulla SD in modo comprensibile
    switch (state) {
        case STATE_IDLE: return "IDLE";
        case STATE_ASCENT: return "ASCENT";
        case STATE_DESCENT_FAST: return "DESCENT_FAST";
        case STATE_DESCENT_SLOW: return "DESCENT_SLOW";
        case STATE_LANDED: return "LANDED";
        default: return "UNKNOWN";
    }
}