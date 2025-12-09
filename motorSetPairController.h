#include <motorSet.h>

class MotorSetPairController {
    public:
        SensorSetPairController sensor_set_pair_controller;        
        SensorSet front_sensor;
        SensorSet back_sensor;

        MotorSet front;
        MotorSet back;

        void move(int pow, double direction) {
            front.move(pow, direction);
            back.move(pow, direction);
        }
        
        void forward(int pow) {
            move(pow, 0.0);  
        }

        void stop() {
            front.stop();
            back.stop();
        }

        void run_until_black() {
            double direction = sensor_set_pair_controller.get_direction();
            move(20, direction);
            while (front_sensor.get_value() < 0.9);
            stop();
        }
};