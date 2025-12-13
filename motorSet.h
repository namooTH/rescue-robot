#pragma once
#include <POP32.h>

struct MotorSet {
    const int left;
    const int right;

    int left_speed = 0;
    int right_speed = 0;

    const int LEFT_POLARITY  = 1;
    const int RIGHT_POLARITY = 1;

    void move(int pow, double direction) {
        direction = constrain(direction, -1.0, 1.0);

        int maxPow = abs(pow);

        int left_speed  = pow - (2.0 * pow * direction);
        int right_speed = pow + (2.0 * pow * direction);

        left_speed  = constrain(left_speed,  -maxPow, maxPow);
        right_speed = constrain(right_speed, -maxPow, maxPow);

        motor(left, left_speed);
        motor(right, right_speed);
    }


    void stop() {
        left_speed = 0;
        right_speed = 0;

        motor(left, 0);
        motor(right, 0);
    }
};