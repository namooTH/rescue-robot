#define DEBUG
#include <draw/draw.hpp>
#include <draw/menu.hpp>
#include <rescueConfig.h>

void setup() {
    asm(".global _printf_float");
    SerialUSB.begin(9600);
    Wire.setClock(1000000);
    OLED_DMA_Init(); 
}

void deploy_dice() {
    servo(1, 0);
    delay(500);
    servo(1, 90);
}

void run() {
    clear();
    drawTextFmt(0,0,WHITE,"%d", 1);
    flip();
    motor_controller.backward = false;
    motor_controller.run_until_white();
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
    motor_controller.run_until_black(false);
    deploy_dice();

    motor_controller.run(-1.5);
    motor_controller.run(0.5);
    motor_controller.rotate_to(-90);
    motor_controller.run_until_black();

    motor_controller.rotate_to(0);
    motor_controller.run_until_black();
    motor_controller.rotate_to(90);
    motor_controller.run_until_black();
    deploy_dice();

    motor_controller.run(-1.0);
    motor_controller.rotate_to(180);
    motor_controller.run_until_black();
    motor_controller.rotate_to(90);
    motor_controller.run_until_black();

    motor_controller.rotate_to(-90);
    motor_controller.run_until_black();
    motor_controller.rotate_to(0);
    motor_controller.run_until_black();
    motor_controller.rotate_to(-90);
    motor_controller.run_until_black();
    motor_controller.rotate_to(180);
    motor_controller.run_until_black();
    deploy_dice();

    motor_controller.rotate_to(0);
    motor_controller.run_until_black();
    motor_controller.rotate_to(90);
    motor_controller.run_until_black();
    motor_controller.rotate_to(180);
    motor_controller.run_until_black();
    deploy_dice();

    // motor_controller.deploy_dice_front();    
}

void test_motor() {
    while (1) {
        motor_controller.move(40, 0.0);
    }
}

void test_imu() {
    while (1) {
        clear();
        drawTextFmt(0,0,WHITE,"%f", imu_sensor.getYaw());
        flip();
    }
}

Menu menu = { {
    {"Calibrate Sensors", cali_sensors},
    {"Run", run},
    {"Align", [](){motor_controller.align(true);}},
    {"Run Until Black", [](){motor_controller.run_until_black();}},
    {"Test Motor", test_motor},
    {"Test IMU", test_imu},
    {"Test Cube", deploy_dice}
} };

void loop() {
    menu.menu();
}