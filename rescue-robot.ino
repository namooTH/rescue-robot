#include <POP32.h>

#include <sensorSetPairController.h>
#include <motorSetPairController.h>
#include <IMUSensor.h>

#define DEBUG

const MotorSet motorSets[2] = { {1, 2},   // Front
                                {3, 4} }; // Back

SensorSet sensorSets[4] = { {Sensor{0, 1, 1}, Sensor{1, 1, 1}},   // Front
                            {Sensor{2, 1, 1}, Sensor{3, 1, 1}},   // Back
                            {Sensor{4, 1, 1}, Sensor{6, 1, 1}},   // Left
                            {Sensor{5, 1, 1}, Sensor{7, 1, 1}} }; // Right

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
            char *sensor_name = new char[ sensor_debug_names[sensor_idx].length() ];
            strcpy(sensor_name, sensor_debug_names[sensor_idx].c_str());
            oled.text(1,0, sensor_name);
            #endif
            oled.text(2,0, "%d        %d", sensor_set.left.get_value(), sensor_set.right.get_value());
            oled.show();
        }
        while (SW_A());
        sensor_idx++;
    }

    oled.clear();
    while (!SW_A()) {
        sensor_idx = 0;
        oled.text(1,0, "White");
        for (SensorSet sensor_set: sensorSets) {
            oled.text(sensor_idx+1, 0, "%d        %d", sensor_set.left.whiteValue, sensor_set.right.whiteValue);
            sensor_idx++;
            oled.show();
        }
    }
    oled.clear();
    while (!SW_A()) {
        sensor_idx = 0;
        oled.text(1,0, "Black");
        for (SensorSet sensor_set: sensorSets) {
            oled.text(sensor_idx+1, 0, "%d        %d", sensor_set.left.blackValue, sensor_set.right.blackValue);
            sensor_idx++;
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