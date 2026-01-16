#define RESCUE_CONFIG

#include <POP32.h>
#include <string>
#include "controller/sensorSetPairController.h"
#include "controller/motorSetPairController.h"
#include "draw/draw.hpp"
#include "sensor/IMUSensor.h"


const MotorSet motorSets[2] = { {1, 2, 0, 0, 0, 3},   // Front
                                {3, 4, 0, 0, 0, 3}};  // Back

Sensor sensors[8] = {
    {0, 444, 2093}, {1, 710, 3732},  // Front
    {2, 719, 3101}, {3, 235, 1168},  // Back
    {4, 618, 3516}, {5, 403, 1990},  // Left
    {6, 692, 2835}, {7, 601, 2693}   // Right
};


void parse_sensor_data(String data) {

}



SensorSet sensorSets[4] = { { &sensors[0], &sensors[1] },   // Front
                            { &sensors[2], &sensors[3] },   // Back
                            { &sensors[4], &sensors[5] },   // Left
                            { &sensors[6], &sensors[7] } }; // Right

IMUSensor imu_sensor;
SensorSetPairController sensor_controller = { &sensorSets[2], &sensorSets[3] };
MotorSetPairController  motor_controller  = { sensor_controller,
                                              &imu_sensor,
                                              
                                              sensorSets[0],
                                              sensorSets[1],
  
                                              motorSets[0], 
                                              motorSets[1] };


String sensor_debug_names[4] = {
    "Front Sensor",
    "Back Sensor",
    "Left Sensor",
    "Right Sensor"
};

void cali_sensors() {
    size_t sensor_idx = 0;
    for (SensorSet &sensor_set: sensorSets) {
        while (!SW_A()) {
            clear();
            drawTextFmt(0, 0, WHITE, "White");
            drawTextFmt(0, 10, WHITE, sensor_debug_names[sensor_idx].c_str());
            drawTextFmt(0, 30, WHITE, "%d        %d", sensor_set.left->get_value(), sensor_set.right->get_value());
            flip();
        }
        sensor_set.set_white();
        while (SW_A());
        sensor_idx++;
    }

    sensor_idx = 0;
    for (SensorSet &sensor_set: sensorSets) {
        while (!SW_A()) {
            clear();
            drawTextFmt(0, 0, WHITE, "Black");
            drawTextFmt(0, 10, WHITE, sensor_debug_names[sensor_idx].c_str());
            drawTextFmt(0, 30, WHITE, "%d        %d", sensor_set.left->get_value(), sensor_set.right->get_value());
            flip();
        }
        sensor_set.set_black();
        while (SW_A());
        sensor_idx++;
    }

    clear();
    drawTextFmt(0, 0, WHITE, "Plug in USB to");
    drawTextFmt(0, 10, WHITE, "get values");
    flip();
    while (!SW_A());
    while (SW_A());

    for (SensorSet &sensor_set: sensorSets) {
        SerialUSB.printf("%d %d %d %d %d %d ", sensor_set.left->channel, sensor_set.left->whiteValue, sensor_set.left->blackValue, sensor_set.right->channel, sensor_set.right->whiteValue, sensor_set.right->blackValue);
    }

    clear();
    sensor_idx = 0;
    drawTextFmt(0, 0, WHITE, "White");
    for (SensorSet &sensor_set: sensorSets) {
        drawTextFmt(0, 10+(10*sensor_idx), WHITE, "%d        %d", sensor_set.left->whiteValue, sensor_set.right->whiteValue);
        sensor_idx++;
    }
    SerialUSB.print("\n");
    flip();
    while (!SW_A());
    while (SW_A());

    clear();
    sensor_idx = 0;
    drawTextFmt(0, 0, WHITE, "Black");
    for (SensorSet &sensor_set: sensorSets) {
        drawTextFmt(0, 10+(10*sensor_idx), WHITE, "%d        %d", sensor_set.left->blackValue, sensor_set.right->blackValue);
        sensor_idx++;
    }
    flip();
    while (!SW_A());
}

struct RescueInit {
    RescueInit() {
        imu_sensor.Init();
    };
};

static RescueInit rescue_init;