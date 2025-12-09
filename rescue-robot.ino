#include <POP32.h>

#include <sensorSetPairController.h>
#include <motorSetPairController.h>
#include <IMUSensor.h>

#define DEBUG

const MotorSet motorSets[2] = { {1, 2},   // Front
                                {3, 4} }; // Back

SensorSet sensorSets[4] = { {Sensor{0, 0, 0}, Sensor{1, 0, 0}},   // Front
                            {Sensor{2, 0, 0}, Sensor{3, 0, 0}},   // Back
                            {Sensor{4, 0, 0}, Sensor{6, 0, 0}},   // Left
                            {Sensor{5, 0, 0}, Sensor{7, 0, 0}} }; // Right

SensorSetPairController sensor_controller = {sensorSets[2], sensorSets[3]};
MotorSetPairController  motor_controller  = {sensor_controller, sensorSets[1], sensorSets[2], motorSets[0], motorSets[1]};

IMUSensor imu_sensor;

#ifdef DEBUG
#include <string>
String sensor_debug_names[4] = {
    "Front Sensor",
    "Back Sensor",
    "Left Sensor",
    "Right Sensor"
};
#endif

void cali_sensors() {
    size_t sensor_idx = 0;
    for (SensorSet sensor_set: sensorSets) {
        while (!SW_A()) {
            oled.clear();
            #ifdef DEBUG
            oled.text(1,0, "%s", sensor_debug_names[sensor_idx].c_str());
            #endif
            oled.text(2,0, "%d        %d", sensor_set.left.get_value(), sensor_set.right.get_value());
            oled.text(3,0, "%d", sensor_set.get_value());
            oled.show();

            sensor_idx++;
        }
        while (SW_A());
    }

    size_t sensor_idx = 0;
    while (!SW_A()) {
        oled.clear();
        for (SensorSet sensor_set: sensorSets) {
            oled.text(1,0, "%s", sensor_debug_names[sensor_idx].c_str());
        
            sensor_idx++;
            while (SW_A());
        oled.show();
    }
    }
}

void setup() {
    cali_sensors();
    while (!SW_A());
    motor_controller.forward(20);
    // delay(2000);
    // motor_controller.stop();
    // delay(1000);
    // motor_controller.move(50, 1.0);
    // delay(500);
    // motor_controller.stop();
    // delay(1000);
    // motor_controller.move(50, -1.0);
    // delay(500);
    // motor_controller.stop();
}

void loop() {
}