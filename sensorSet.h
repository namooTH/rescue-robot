#include <POP32.h>
#include <sensor.h>

struct SensorSet {
    Sensor left;
    Sensor right;

    double get_value() {
        return (left.get_value() + right.get_value()) / 2.0f;
    }
};