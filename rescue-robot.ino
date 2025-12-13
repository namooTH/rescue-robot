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
    motor_controller.backward = false;
    motor_controller.run_until_black(false); motor_controller.run_until_white();
    motor_controller.run_until_black();
    motor_controller.rotate_to(-90);
    motor_controller.run_until_black();
    motor_controller.rotate_to(180);
    motor_controller.align();
    motor_controller.run_until_black();
    motor_controller.rotate_to(90);
    motor_controller.run_until_black();
    motor_controller.rotate_to(180);
    motor_controller.run_until_black();
    // motor_controller.deploy_dice_front();    
}

Menu menu = { {{"Calibrate Sensors", cali_sensors}, {"Run", run}} };

void loop() {
    menu.menu();
}