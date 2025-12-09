#define DEBUG

#include <POP32.h>
#include <rescueConfig.h>

void setup() {
    cali_sensors();
    motor_controller.run_until_black();
    motor_controller.stop();
}

void loop() {
    oled.clear();
    oled.text(1,0, "%f", sensor_controller.left->get_normalised());
    oled.show();
}