#pragma once
#include "../motor/motorSet.h"
#include "sensorSetPairController.h"
#include "../sensor/IMUSensor.h"
#include "../utils/PID.h"

class MotorSetPairController {
    public:
        SensorSetPairController sensor_set_pair_controller;
        IMUSensor *imu_sensor;

        SensorSet front_sensor;
        SensorSet back_sensor;
        
        MotorSet front;
        MotorSet back;

        SensorSet far_front_sensor;
        SensorSet far_back_sensor;
        
        MotorSetPairController(const SensorSetPairController& sspc,
                               IMUSensor* imu,
                               const SensorSet& front_s,
                               const SensorSet& back_s,
                               const MotorSet& front_m,
                               const MotorSet& back_m)
            : sensor_set_pair_controller(sspc),
              imu_sensor(imu),
              front_sensor(front_s),
              back_sensor(back_s),
              front(front_m),
              back(back_m),
              far_front_sensor{sensor_set_pair_controller.left->left,
                               sensor_set_pair_controller.right->left},
              far_back_sensor{sensor_set_pair_controller.left->right,
                              sensor_set_pair_controller.right->right}
        {}

        bool backward = false;

        void move(int pow, double direction, bool use_offset = true) {
            front.move(pow, direction, use_offset);
            back.move(pow, direction, use_offset);
        }
        
        void stop() {
            front.stop();
            back.stop();
        }
        
        void run(float tile = 1.0) {
            move((signbit(tile) ? -1 : 1) * 153, 0.0);
            delay(550 * fabs(tile));
            stop();
        }

        void run_until_black(float boost = 1.0f, bool back_up = true, int pow = 178) {
            SensorSet* sensor = backward ? &back_sensor : &front_sensor;

            int speed = pow;

            int now = millis();
            int start = now;
            int lastTime = now;
            
            int boostTime = 270.0f * fabs(boost);

            while (sensor->left->get_normalised() < 0.9 && sensor->right->get_normalised() < 0.9) {
                now = millis();
                float dt = (now - lastTime) / 1000.0f;
                lastTime = now;
                if (dt <= 0) dt = 0.001f;
                
                if (boost > 0.0) {
                    speed = lerp(230.0, (double) pow, constrain((now - start) / boostTime, 0.0, 1.0));
                }

                double ln = sensor_set_pair_controller.left->left->get_normalised();
                double rn = sensor_set_pair_controller.right->left->get_normalised();
                double direction = 0.0;

                if (!(ln < 0.9 && rn < 0.9)) {
                    double dir = ln - rn;
                    direction = constrain(dir, -1.0, 1.0);
                }
                
                move(speed, -direction);
            };

            move(-pow, 0.0);
            delay(200);
            stop();

            delay(100);
            align(!backward);
            
            if (back_up) {
                move(-102, 0.0);
                delay(130);
                stop();
            }
        }
        
        void run_until_white() {
            SensorSet* sensor = nullptr;
            double (SensorSet::*sensor_func)() = &SensorSet::get_normalised;
            sensor = backward ? &back_sensor : &front_sensor;

            while (sensor->get_normalised() > 0.1) {
                double direction = sensor_set_pair_controller.get_direction();
                move(127, direction);
            };
        }

        bool check_front(float timeout = 2.0, bool back_up = true) {
            int start = millis();
            float elapsed = 0;
            bool found = false;
            while (elapsed < timeout) {
                elapsed = (millis() - start) / 1000.0;
                if (front_sensor.get_normalised() > 0.8) {
                    found = true;
                    break;
                }
                move(40, 0.0);
            }
            if (found) {
                align(true);
            }

            if (back_up) {
                move(-40, 0.0);
                delay(elapsed * 1000);
                stop();
            }

            return found;
        }

        PID alignPID = {4.0, 0.0, 0.25};
        
        void align(bool backward = false) {
            alignPID.reset();

            SensorSet* nearSensor = backward ? &front_sensor : &back_sensor;
            SensorSet* farSensor = backward ? &far_front_sensor : &far_back_sensor;

            SensorSet* sensors[2] = {nearSensor, farSensor};

            int SEARCH_SPEED = backward ? -100 : 100;
            int ALIGN_DIR  = backward ? -1 : 1;
            const double BLACK_MIN = 0.5;    // bar detection threshold
            const double CENTER_EPS = 0.07;  // balance tolerance
        
            for (SensorSet* sensor : sensors) {
                bool unable = false;

                int now = millis();
                int start = now;
                int lastTime = now;

                while (sensor->get_normalised() >= BLACK_MIN) {
                    if (now - start > 200) {
                        unable = true;
                        break;   
                    }
                    now = millis();
                    move(SEARCH_SPEED, 0.0);
                    unable = false;
                }
                while (sensor->get_normalised() < BLACK_MIN) {
                    if (now - start > 200) {
                        unable = true;
                        break;
                    }
                    now = millis();
                    move(-SEARCH_SPEED, 0.0);
                    unable = false;
                }
                stop();

                now = millis();
                
                while (now - start < 2000 && !unable) {
                    now = millis();

                    float dt = (now - lastTime) / 1000.0f;
                    lastTime = now;
                    if (dt <= 0) dt = 0.001f;
                    
                    double strength = sensor->get_normalised();
                    double strength_error = 1.0 - strength;
                    double dir = sensor->get_direction();

                    float pidOut = alignPID.update(strength_error, dt);
                    
                    if (fabs(dir) < CENTER_EPS) {
                        break;
                    }
                
                    int fb = (strength > BLACK_MIN) ? ALIGN_DIR * speedFromPID(pidOut, 160, 200) : ALIGN_DIR * -speedFromPID(pidOut, 160, 200);
                
                    move(fb, -dir, false);
                }
            }

            stop();
            resetIMUToLastPerfect();
        }
        
        float getWorldYaw() {
            imu_sensor->getYaw();
            return norm180(worldYawOffset + imu_sensor->getYaw());
        }

        const float MAX_ERR = 90.0f;
        
        PID yawPID;
        
        void rotate_to(float targetDeg) {
            targetDeg = norm180(targetDeg);
        
            float error = norm180(targetDeg - getWorldYaw());
            float dir = (error > 0) ? 1.0f : -1.0f;
            
            float absError = fabs(error);
            if (absError >= 100) {
                yawPID = {4.0, 0.0, 0.5};
            } else {
                yawPID = {4.0, 0.0, 1.0};
            }
            
            yawPID.reset();
            
            int lastTime = millis();
            
            while (true) {
                int now = millis();
                float dt = (now - lastTime) / 1000.0f;
                lastTime = now;
                if (dt <= 0) dt = 0.001f;

                error = norm180(targetDeg - getWorldYaw());
                float pidOut = yawPID.update(error, dt);

                dir = (error > 0) ? 1.0f : -1.0f;
            
                if (fabs(error) < 0.15f) break;
                
                move(speedFromPID(pidOut), dir, false);
            }
        
            stop();
        
            lastPerfectYaw = targetDeg;
            resetIMUKeepWorld();
        }

    private:
        float worldYawOffset = 0.0f;
        float lastPerfectYaw = 0.0f;
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

        int speedFromPID(float pidOut, int stall_speed = 120, int speed_max = 250) {
            float mag = fabs(pidOut);
            
            if (mag < stall_speed)
                mag = stall_speed;
        
                if (mag > speed_max)
                mag = speed_max;
                
            return (int)mag;
        }
        
        float norm180(float a) {
            while (a > 180) a -= 360;
            while (a < -180) a += 360;
            return a;
        }

        void resetIMUKeepWorld() {
            imu_sensor->getYaw();
            worldYawOffset = norm180(lastPerfectYaw - imu_sensor->getYaw());    
            imu_sensor->Reset();
        }

        void resetIMUToLastPerfect() {
            worldYawOffset = norm180(lastPerfectYaw);
            imu_sensor->Reset();
        }
    };