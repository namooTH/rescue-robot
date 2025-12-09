#include <POP32.h>

struct Sensor {
    u_int8_t channel;
    unsigned short blackValue;
    unsigned short whiteValue;

    double get_value() {
        (double) (analog(channel) - whiteValue) / (double) (blackValue - whiteValue); 
    }
};