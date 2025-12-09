#include <POP32.h>

struct Sensor {
    u_int8_t channel;
    unsigned short blackValue;
    unsigned short whiteValue;

    int get_value() {
        return analog(channel); 
    }

    double get_normalised() {
        return (double) (get_value() - whiteValue) / (double) (blackValue - whiteValue); 
    }
};