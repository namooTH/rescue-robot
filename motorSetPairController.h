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

        bool backward = false;

        void move(int pow, double direction) {
            front.move(pow, direction);
            back.move(pow, direction);
            // clear();
            // drawTextFmt(0, 0, WHITE, "front: %d, %d", front.left_speed, front.right_speed);
            // drawTextFmt(0, 10, WHITE, "back: %d, %d", back.left_speed, back.right_speed);
            // flip();
        }
        
        void forward(int pow) {
            move(pow, 0.0);  
        }

        void stop() {
            front.stop();
            back.stop();
        }

        void run_until_black(bool back_up = true) {
            SensorSet* sensor = backward ? &back_sensor : &front_sensor;

            while (sensor->left->get_normalised() < 0.5 && sensor->right->get_normalised() < 0.5) {
                double direction = sensor_set_pair_controller.get_direction();
                move(40, -direction);
            };
            align(!backward);
            if (back_up) {
                move(-20, 0.0);
                delay(300);
                stop();
            }
        }
        
        void run_until_white() {
            SensorSet* sensor = nullptr;
            double (SensorSet::*sensor_func)() = &SensorSet::get_normalised;
            sensor = backward ? &back_sensor : &front_sensor;

            while (sensor->get_normalised() > 0.1) {
                double direction = sensor_set_pair_controller.get_direction();
                move(40, direction);
                clear();
                drawTextFmt(0, 0, WHITE, "%f", direction);
                flip();
            };
        }

        void run(float tile = 1.0) {
            move((signbit(tile) ? -1 : 1) * 40, 0.0);
            delay(1000 * fabs(tile));
            stop();
        }

        void align(bool backward = false) {
            SensorSet* sensor = backward ? &front_sensor : &back_sensor;
        
            int SEARCH_SPEED = backward ? -40 : 40;
            int ALIGN_SPEED  = backward ? -40 : 40;
            const double BLACK_MIN = 0.5;   // bar detection threshold
            const double CENTER_EPS = 0.2;  // balance tolerance
        
            while (sensor->get_normalised() < BLACK_MIN) {
                move(-SEARCH_SPEED, 0.0);   // straight
            }
            stop();
        
            while (true) {
                double strength = sensor->get_normalised();
                double dir = sensor->get_direction();
            
                if (fabs(dir) < CENTER_EPS && fabs(strength - BLACK_MIN) < CENTER_EPS) {
                    break;
                }

                clear();
                drawTextFmt(0, 0, WHITE, "%f %f", fabs(dir), fabs(strength - BLACK_MIN));
                flip();
               
                int fb = ALIGN_SPEED;
                if (strength < BLACK_MIN - CENTER_EPS) {
                    fb = -ALIGN_SPEED;
                } else if (strength > BLACK_MIN + CENTER_EPS) {
                    fb = ALIGN_SPEED;
                }
            
                move(fb, -dir);
            }
        
            stop();
        }
        
        float norm180(float a) {
            while (a > 180) a -= 360;
            while (a < -180) a += 360;
            return a;
        }

        float continuousYaw = 0.0f;
        float lastYaw = 0.0f;
        bool yawInit = false;
        
        float unwrapYaw(float rawYaw) {
            if (!yawInit) {
                lastYaw = rawYaw;
                continuousYaw = rawYaw;
                yawInit = true;
                return continuousYaw;
            }
        
            float delta = rawYaw - lastYaw;
        
            // detect wrap (your IMU jumps ~370 deg)
            if (delta > 180)  delta -= 360;
            if (delta < -180) delta += 360;
        
            continuousYaw += delta;
            lastYaw = rawYaw;
        
            return continuousYaw;
        }

        void rotate_to(float targetDeg) {
            imu_sensor.getYaw();
            float startYaw = unwrapYaw(imu_sensor.pvYaw);
        
            // target relative to current heading
            float targetYaw = startYaw + norm180(targetDeg - norm180(startYaw));
        
            while (true) {
                imu_sensor.getYaw();
                float yaw = unwrapYaw(imu_sensor.pvYaw);
            
                float error = targetYaw - yaw;
            
                if (fabs(error) < 3.0f)
                    break;
            
                // left = -1, right = +1
                float dir = (error > 0) ? 1.0f : -1.0f;
            
                move(50, dir);
            }
        
            stop();
        }


};