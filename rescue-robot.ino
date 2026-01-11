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

void deploy_dice() {
    servo(1, 55);
    delay(300);
    servo(1, 180);
    delay(300);
    servo(1, 150);
}

void deflag() {
    servo(2, 180);
}

void flag() {
    servo(2, 80);
}

void run() {
    motor_controller.run_until_white();
    motor_controller.run_until_black();
    motor_controller.rotate_to(90.0);
    motor_controller.run_until_black(0.00, true, false, 140);
    motor_controller.rotate_to(180.0);
    motor_controller.run(0.05);
    motor_controller.run_until_black(0.0, true, false, 140);
    motor_controller.rotate_to(90.0);
    
    motor_controller.run_until_black(1.0);
    deploy_dice(); // red
    motor_controller.run(-0.7);

    motor_controller.rotate_to(0.0);
    motor_controller.run_until_black(0.0, true, true);
    motor_controller.run_until_black();
    motor_controller.rotate_to(90.0);
    motor_controller.run_until_black();
    motor_controller.run(-0.05);
    motor_controller.rotate_to(180.0);

    motor_controller.run_until_black(0.0, false);
    motor_controller.run_until_white();
    motor_controller.run_until_black(0.0, false);

    motor_controller.run_until_black(0.0, false, true);
    motor_controller.run_until_white(true);
    motor_controller.run_until_black(0.0, false, true);
    motor_controller.rotate_to(-90.0);

    motor_controller.run(1.6);
    motor_controller.rotate_to(180.0);
    motor_controller.run_until_black(0.0, true, true);
    motor_controller.run_until_black();
    motor_controller.run(-0.1);
    deploy_dice(); // green

    motor_controller.run_until_black(0.0, true, true);
    motor_controller.rotate_to(-90.0);
    motor_controller.run_until_black();
    motor_controller.rotate_to(0.0);
    motor_controller.run_until_black(0.0, true, true); // checkpoint 2
    motor_controller.run(1.75);
    motor_controller.rotate_to(90.0);
    motor_controller.run_until_black(0.0, true, true);

    motor_controller.run_until_black(0.0);
    deploy_dice(); // blue
    motor_controller.run_until_black(0.0, true, true);

    motor_controller.rotate_to(0.0);
    motor_controller.run_until_black(0.0, true, false, 140);
    
    motor_controller.rotate_to(90.0);
    motor_controller.run_until_black();
    motor_controller.rotate_to(180.0);
    motor_controller.run_until_black();
    motor_controller.rotate_to(90.0);
    motor_controller.run_until_black();
    motor_controller.rotate_to(0.0);
    motor_controller.run_until_black();
    motor_controller.rotate_to(90.0);
    motor_controller.run_until_black();
    motor_controller.rotate_to(180.0);
    motor_controller.run_until_black(1.0);
    motor_controller.run(-0.25);
    deploy_dice();
    flag();
    
    motor_controller.stop();
}   


Menu tests = { {
    {"Test Motor", [](){
        while (1) motor_controller.move(100, 0.0);
    }},
    {"Test IMU", [](){
        while (1) {
            clear();
            drawTextFmt(0,0,WHITE,"%f", imu_sensor.getYaw());
            drawTextFmt(0,10,WHITE,"SW_OK to Reset");
            flip();
            if (SW_OK()) {
                imu_sensor.Reset();
            }
        }
    }},
    {"Test Dice", deploy_dice},
    {"Test Flag", flag},
    {"Test DeFlag", deflag},
    {"Align", [](){motor_controller.align(true);}},
    {"Test Rotate 90", [](){motor_controller.imu_sensor->Reset(); motor_controller.rotate_to(-90.0);}},
    {"Test Rotate 180", [](){motor_controller.imu_sensor->Reset(); motor_controller.rotate_to(180.0);}},
    {"Run Until Black", [](){motor_controller.run_until_black();}},
    {"Run Until Black Backward", [](){motor_controller.run_until_black(0.0, true, true);}}
}};

Menu menu = { {
    {"Run", run},
    {"Tests", [](){tests.menu();}},
    {"Calibrate Sensors", cali_sensors}
} };

void loop() {
    menu.menu();
}