#include <POP32.h>

struct MotorSet {
    const int left;
    const int right;

    const int LEFT_POLARITY  = 1;
    const int RIGHT_POLARITY = 1;

    void move(int pow, double direction) {
        direction = constrain(direction, -1.0, 1.0);

        int left_speed  = pow + (2.0 * pow * direction);
        int right_speed = pow - (2.0 * pow * direction);

        left_speed  = constrain(left_speed,  -pow, pow);
        right_speed = constrain(right_speed, -pow, pow);

        motor(left, left_speed);
        motor(right, right_speed);
    }

    void stop() {
        motor(left, 0);
        motor(right, 0);
    }
};