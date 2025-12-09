#include <sensorSet.h>

class SensorSetPairController {
    public:
        SensorSet left;
        SensorSet right;

        double get_direction() {
            return (left.get_value() + right.get_value()) / 2.0f;
        }
};