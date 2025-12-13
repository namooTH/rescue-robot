#pragma once
#include <motorSet.h>
#include <sensorSetPairController.h>
#include <IMUSensor.h>

class MotorSetPairController {
    public:
        SensorSetPairController sensor_set_pair_controller;
        IMUSensor imu_sensor;

        SensorSet front_sensor;
        SensorSet back_sensor;

        MotorSet front;
        MotorSet back;

        bool backward = true;

        void move(int pow, double direction) {
            front.move(pow, direction);
            back.move(pow, direction);
            clear();
            drawTextFmt(0, 0, WHITE, "front: %d, %d", front.left_speed, front.right_speed);
            drawTextFmt(0, 10, WHITE, "back: %d, %d", back.left_speed, back.right_speed);
            flip();
        }
        
        void forward(int pow) {
            move(pow, 0.0);  
        }

        void stop() {
            front.stop();
            back.stop();
        }

        void run_until_black(bool back_up = true) {
            SensorSet* sensor = nullptr;
            sensor = backward ? &back_sensor : &front_sensor;

            while (sensor->get_normalised() < 1.0) {
                double direction = sensor_set_pair_controller.get_direction();
                move(40, -direction);
                clear();
                drawTextFmt(0, 0, WHITE, "%f", direction);
                flip();
            };
            if (back_up) {
                stop();
                move(-20, 0.0);
                delay(300);
                stop();
            }
        }
        
        void run_until_white() {
            SensorSet* sensor = nullptr;
            double (SensorSet::*sensor_func)() = &SensorSet::get_normalised;
            sensor = backward ? &back_sensor : &front_sensor;

            while (sensor->get_normalised() > 0.5) {
                double direction = sensor_set_pair_controller.get_direction();
                move(40, direction);
                clear();
                drawTextFmt(0, 0, WHITE, "%f", direction);
                flip();
            };
        }

        void run(int tile = 1) {
            move(40, 0.0);
            delay(1000 * tile);
            stop();
        }

        void align() {
            SensorSet* sensor = nullptr;
            sensor = backward ? &front_sensor : &back_sensor; 

            while (sensor->get_normalised() < 1.0) {
                double direction = back_sensor.get_direction();
                move(-40, -direction);
                clear();
                drawTextFmt(0, 0, WHITE, "%f", direction);
                flip();
            };

            stop();
        }

        float norm180(float a) {
            while (a > 180) a -= 360;
            while (a < -180) a += 360;
            return a;
        }
        
        void rotate_to(float targetDeg) {
            targetDeg = norm180(targetDeg);

            imu_sensor.getYaw();
            float current = norm180(imu_sensor.pvYaw);

            // signed difference (e.g. +60 means turn left 60 degrees)
            float diff = norm180(targetDeg - current);

            // direction: left = -1, right = +1
            float dir = (diff > 0) ? -1.0f : 1.0f;

            while (fabs(norm180(imu_sensor.pvYaw - targetDeg)) > 3.0f) {
                imu_sensor.getYaw();
                move(60, dir);
            }

            stop();
        }

};