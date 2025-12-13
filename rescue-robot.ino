#define DEBUG
#include <draw/draw.hpp>
#include <draw/menu.hpp>
#include <rescueConfig.h>

void setup() {
    asm(".global _printf_float");
    Wire.setClock(1000000);
    OLED_DMA_Init(); 
}

void run() {
    // motor_controller.rotate_to(180);
    // delay(1000);
    // motor_controller.rotate_to(90);
    motor_controller.run_until_white();
    motor_controller.run_until_black();
    motor_controller.run_until_white();
    motor_controller.run_until_black();
    motor_controller.rotate_to(90);
}

Menu menu = { {{"Calibrate Sensors", cali_sensors}, {"Run", run}} };

void loop() {
    menu.menu();
}