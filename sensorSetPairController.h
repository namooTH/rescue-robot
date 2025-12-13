#pragma once
#include <sensorSet.h>

class SensorSetPairController {
    public:
        SensorSet *left;
        SensorSet *right;

        double get_direction() {
            return (left->get_normalised() - right->get_normalised()) / 2.0f;
        }
};