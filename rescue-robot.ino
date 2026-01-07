#define DEBUG
#include "draw/draw.hpp"
#include "draw/menu.hpp"
#include "rescueConfig.h"
#include "furelise.h"

void setup() {
    asm(".global _printf_float");
    SerialUSB.begin(9600);
    Wire.setClock(1000000);
    OLED_DMA_Init(); 

    servo(1, 175);
    deflag();
}

void control_mode() {
    double dir = 0.0;
    while (true) {
        if (SerialUSB.available()) {
            char usb = SerialUSB.read();
            switch (usb)
            {
            case 'w':
                motor_controller.move(50, dir);
                break;
            case 'a':
                dir = 0.5;
                break;
            case 's':
                motor_controller.move(-50, dir);
                break;
            case 'd':
                dir = -0.5;
                break;
            case ' ':
                break;
            default:
                break;
            }
        }
        motor_controller.move(0, dir);
        
    }
}

void deploy_dice() {
    servo(1, 55);
    delay(300);
    servo(1, 180);
    delay(300);
    servo(1, 150);
}

void deflag() {
    servo(2, 80);
}

void flag() {
    servo(2, 180);
}

void run_auto() {
    motor_controller.check_front();
    // motor_controller.run(1.0);
    // while (1) {
    //     motor_controller.rotate_to(90);
    //     motor_controller.run();
    // }
}

void run() {
    motor_controller.run(1.2);
    motor_controller.rotate_to(90.0);
    motor_controller.run_until_black(0.00); //checkpoint 1
    
    motor_controller.run_until_black(0.00 , true, true);
    motor_controller.rotate_to(0.0);
    motor_controller.run(1.0);
    motor_controller.rotate_to(90.0);
    motor_controller.run_until_black(0.00);
    motor_controller.rotate_to(0.0);
    motor_controller.run_until_black(0.00);
    motor_controller.rotate_to(90.0);
    motor_controller.run_until_black(0.00, true, false, 130, 0.5);
    deploy_dice(); //1st
    
    motor_controller.run(-0.5);
    motor_controller.rotate_to(-90.0);
    motor_controller.run_until_black(0.00);
    motor_controller.rotate_to(180.0);
    motor_controller.run_until_black(0.00);
    motor_controller.rotate_to(-90.0);
    motor_controller.run_until_black(0.00);
    motor_controller.rotate_to(0.0);
    motor_controller.run_until_black(0.00);
    motor_controller.rotate_to(90.0);
    motor_controller.run_until_black(0.00);
    motor_controller.rotate_to(0.0);
    motor_controller.run_until_black(0.00);
    motor_controller.rotate_to(-90.0);

    motor_controller.stop();
}   

void test_motor() {
    while (1) {
        motor_controller.move(80, 0.0);
    }
}

void test_imu() {
    while (1) {
        clear();
        drawTextFmt(0,0,WHITE,"%f", imu_sensor.getYaw());
        drawTextFmt(0,10,WHITE,"SW_OK to Reset");
        flip();
        if (SW_OK()) {
            imu_sensor.Reset();
        }

    }
}

Menu tests = { {
    {"Test Motor", test_motor},
    {"Test IMU", test_imu},
    {"Test Dice", deploy_dice},
    {"Test Flag", flag},
    {"Test DeFlag", deflag},
    {"Align", [](){motor_controller.align(true);}},
    {"Test Rotate 90", [](){motor_controller.resetIMU(); motor_controller.rotate_to(-90.0);}},
    {"Test Rotate 180", [](){motor_controller.resetIMU(); motor_controller.rotate_to(180.0);}},
    {"Check Front", [](){motor_controller.check_front();}},
    {"Run Until Black", [](){motor_controller.run_until_black();}},
    {"Run Until Black Backward", [](){motor_controller.run_until_black(0.0, true, true);}},
    {"Run Rotate Left", [](){motor_controller.move(160, -1.0), delay(2000); motor_controller.stop();}},
    {"Run Rotate Right", [](){motor_controller.move(160, 1.0), delay(2000); motor_controller.stop();}},
    {"Run Until White", [](){motor_controller.run_until_white();}}
}};

Menu menu = { {
    {"Run", run},
    {"Tests", [](){tests.menu();}},
    {"Control Mode", control_mode},
    {"Run Without Fixed", run_auto},
    {"Calibrate Sensors", cali_sensors}
} };

void loop() {
    menu.menu();
}