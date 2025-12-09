#define RESCUE_CONFIG

#include <sensorSetPairController.h>
#include <motorSetPairController.h>
#include <IMUSensor.h>


const MotorSet motorSets[2] = { {1, 2},   // Front
                                {3, 4} }; // Back

// Sensor sensors[8] = {
//     {0, 693, 3342}, {1, 872, 3382},  // Front
//     {2, 656, 3468}, {3, 507, 2237},  // Back
//     {4, 529, 2292}, {6, 733, 2900},  // Left
//     {5, 693, 2841}, {7, 503, 2187}   // Right
// };

Sensor sensors[8] = {
    {0, 1, 1}, {1, 1, 1},  // Front
    {2, 1, 1}, {3, 1, 1},  // Back
    {4, 1, 1}, {6, 1, 1},  // Left
    {5, 1, 1}, {7, 1, 1}   // Right
};

SensorSet sensorSets[4] = { { &sensors[0], &sensors[1] },   // Front
                            { &sensors[2], &sensors[3] },   // Back
                            { &sensors[4], &sensors[5] },   // Left
                            { &sensors[6], &sensors[7] } }; // Right

SensorSetPairController sensor_controller = { &sensorSets[2], &sensorSets[3] };
MotorSetPairController  motor_controller  = { sensor_controller, sensorSets[1], sensorSets[2], motorSets[0], motorSets[1] };

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
    for (SensorSet &sensor_set: sensorSets) {
        while (!SW_A()) {
            oled.clear();
            oled.text(1,0, "White");
            #ifdef DEBUG
            char *sensor_name = new char[ sensor_debug_names[sensor_idx].length() ];
            strcpy(sensor_name, sensor_debug_names[sensor_idx].c_str());
            oled.text(2,0, sensor_name);
            #endif
            oled.text(3,0, "%d        %d", sensor_set.left->get_value(), sensor_set.right->get_value());
            oled.show();
        }
        sensor_set.set_white();
        while (SW_A());
        sensor_idx++;
    }

    sensor_idx = 0;
    oled.clear();
    for (SensorSet &sensor_set: sensorSets) {
        while (!SW_A()) {
            oled.clear();
            oled.text(1,0, "Black");
            #ifdef DEBUG
            char *sensor_name = new char[ sensor_debug_names[sensor_idx].length() ];
            strcpy(sensor_name, sensor_debug_names[sensor_idx].c_str());
            oled.text(2,0, sensor_name);
            #endif
            oled.text(3,0, "%d        %d", sensor_set.left->get_value(), sensor_set.right->get_value());
            oled.show();
        }
        sensor_set.set_black();
        while (SW_A());
        sensor_idx++;
    }

    oled.clear();
    sensor_idx = 0;
    oled.text(1,0, "White");
    for (SensorSet &sensor_set: sensorSets) {
        oled.text(sensor_idx+2, 0, "%d        %d", sensor_set.left->whiteValue, sensor_set.right->whiteValue);
        sensor_idx++;
        oled.show();
    }
    while (!SW_A());
    while (SW_A());
    oled.clear();
    sensor_idx = 0;
    oled.text(1,0, "Black");
    for (SensorSet &sensor_set: sensorSets) {
        oled.text(sensor_idx+2, 0, "%d        %d", sensor_set.left->blackValue, sensor_set.right->blackValue);
        sensor_idx++;
        oled.show();
    }
    while (!SW_A());
}
