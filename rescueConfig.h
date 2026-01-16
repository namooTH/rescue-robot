#define RESCUE_CONFIG

#include <POP32.h>
#include "controller/sensorSetPairController.h"
#include "controller/motorSetPairController.h"
#include "draw/draw.hpp"
#include "sensor/IMUSensor.h"


const MotorSet motorSets[2] = { {1, 2, 0, 0, 0, 3},   // Front
                                {3, 4, 0, 0, 0, 3}};  // Back

Sensor sensors[8] = {
    {0}, {1},  // Front
    {2}, {3},  // Back
    {4}, {5},  // Left
    {6}, {7}   // Right
};

const String sensor_data = "0 2079 447 1 3769 704 2 2945 603 3 1159 226 4 3567 567 5 1983 371 6 2950 600 7 2631 523";

// not using sstream beacuse it cannot be used with the arduino's string implementation
void parse_sensor_data(String raw_data) {
    int pos = 0;
    while (pos < raw_data.length()) {
        int space1 = raw_data.indexOf(' ', pos);
        if (space1 == -1) break;
        String portStr = raw_data.substring(pos, space1);
        pos = space1 + 1;
        
        int space2 = raw_data.indexOf(' ', pos);
        if (space2 == -1) break;
        String whiteStr = raw_data.substring(pos, space2);
        pos = space2 + 1;
        
        int space3 = raw_data.indexOf(' ', pos);
        String blackStr;
        if (space3 == -1) {
            blackStr = raw_data.substring(pos);
            pos = raw_data.length();
        } else {
            blackStr = raw_data.substring(pos, space3);
            pos = space3 + 1;
        }
        
        int port = portStr.toInt();
        int white = whiteStr.toInt();
        int black = blackStr.toInt();
        
        for (auto &sensor : sensors) {
            if (sensor.channel == port) {
                sensor.whiteValue = white;
                sensor.blackValue = black;
                break;
            }
        };
    }
}

SensorSet sensorSets[4] = { { &sensors[0], &sensors[1] },   // Front
                            { &sensors[2], &sensors[3] },   // Back
                            { &sensors[4], &sensors[5] },   // Left
                            { &sensors[6], &sensors[7] } }; // Right

IMUSensor imu_sensor;
SensorSetPairController sensor_controller = { &sensorSets[2], &sensorSets[3] };
MotorSetPairController  motor_controller  = { &sensor_controller,
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

void debug_sensor() {
    size_t sensor_idx = 0;

    clear();
    sensor_idx = 0;
    drawTextFmt(0, 0, WHITE, "White");
    for (SensorSet &sensor_set: sensorSets) {
        drawTextFmt(0, 10+(10*sensor_idx), WHITE, "%d        %d", sensor_set.left->whiteValue, sensor_set.right->whiteValue);
        sensor_idx++;
    }
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
        SerialUSB.printf(" %d %d %d %d %d %d", sensor_set.left->channel, sensor_set.left->whiteValue, sensor_set.left->blackValue, sensor_set.right->channel, sensor_set.right->whiteValue, sensor_set.right->blackValue);
    }

    debug_sensor();
}

struct RescueInit {
    RescueInit() {
        parse_sensor_data(sensor_data);
        imu_sensor.Init();
    };
};

static RescueInit rescue_init;