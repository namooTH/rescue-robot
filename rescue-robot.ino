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
    servo(3, 55);
    delay(300);
    servo(3, 180);
    delay(300);
    servo(3, 150);
}

void deflag() {
    servo(2, 180);
}

void flag() {
    servo(2, 80);
}

int N = 0;// ==>
int S = 180;// <==
int E = 90;// ^
int West = -90;// v

void run() {
    motor_controller.run(0.75);
    motor_controller.rotate_to(E);
    motor_controller.run_until_black(0.0);
    motor_controller.run_until_black(0.0, true,true);
    motor_controller.rotate_to(N);
    motor_controller.run(1);
    motor_controller.rotate_to(E);
    motor_controller.run_until_black(0.0);
    motor_controller.rotate_to(N);
    motor_controller.run_until_black(0.0,true,true);
    motor_controller.run_until_black(0.0);
    motor_controller.rotate_to(E);
    motor_controller.run_until_black(0.0, true, false, 140, 0.5);
    deploy_dice();
    motor_controller.run_until_black(0.0,true,true);
    motor_controller.rotate_to(N);
    motor_controller.run_until_black(0.0,true,true);
    motor_controller.rotate_to(West);
    motor_controller.run_until_black(0.0);
    motor_controller.rotate_to(West);
    motor_controller.run_until_black(0.0);
    motor_controller.rotate_to(N);
    motor_controller.run(1.5);
    motor_controller.run_until_black(0.0);
    motor_controller.rotate_to(E);
    motor_controller.run_until_black(0.0);
    motor_controller.rotate_to(N);
    motor_controller.run_until_black(0.0);
    motor_controller.rotate_to(West);
    motor_controller.run_until_black(0.0);
    motor_controller.rotate_to(N);
    motor_controller.run_until_black(0.0, true, false, 140, 0.5);
    deploy_dice();
    motor_controller.run_until_black(0.0,true,true);
    motor_controller.rotate_to(E);
    motor_controller.run_until_black(0.0, true, false, 140, 0.5);
    deploy_dice();
    motor_controller.run(-0.87);
    motor_controller.rotate_to(N);
    motor_controller.run_until_black(0.0);
    motor_controller.rotate_to(West);
    motor_controller.run_until_black(0.0, true, false, 140, 0.5);
    deploy_dice();
    motor_controller.run_until_black(0.0,true,true);
    motor_controller.rotate_to(N);
    motor_controller.run_until_black(0.0,true,true);
    motor_controller.rotate_to(West);
    motor_controller.run(-1);
    motor_controller.rotate_to(N);
    motor_controller.run_until_black(0.0);
    motor_controller.run(-1.0);
    motor_controller.rotate_to(E);
    motor_controller.run_until_black(0.0);
    motor_controller.run(1);
    motor_controller.run_until_black(0.0);
    motor_controller.rotate_to(N);
    motor_controller.run_until_black(0.0,true,true);
    motor_controller.rotate_to(West);
    motor_controller.run_until_black(0.0);
    motor_controller.rotate_to(N);
    motor_controller.run_until_black(0.0,true,true);
    motor_controller.rotate_to(E);
    motor_controller.run_until_black(0.0);
    motor_controller.rotate_to(N);
    motor_controller.run_until_black(0.0,true,true);
    motor_controller.rotate_to(West);
    motor_controller.run_until_black(0.0);
    motor_controller.run_until_black(0.0,true,true);
    motor_controller.rotate_to(N);
    motor_controller.run_until_black(0.0);
    motor_controller.rotate_to(West);
    motor_controller.run_until_black(0.0);
    motor_controller.rotate_to(N);
    motor_controller.run_until_black(0.0);
    motor_controller.rotate_to(E);
    motor_controller.run_until_black(0.0);
    motor_controller.rotate_to(N);
    motor_controller.run_until_black(0.0);
    motor_controller.rotate_to(West);
    motor_controller.run_until_black(0.0);
    motor_controller.run(1);
    motor_controller.rotate_to(N);
    motor_controller.run(2);
    motor_controller.run_until_black(0.0,true,true);
    motor_controller.run(-0.75);

 






  
   play_fur_elise();
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
    {"Run Until Black Backward", [](){motor_controller.run_until_black(0.0, true, true);}},
    {"Run Rotate Left", [](){motor_controller.move(160, -1.0), delay(2000); motor_controller.stop();}},
    {"Run Rotate Right", [](){motor_controller.move(160, 1.0), delay(2000); motor_controller.stop();}},
    {"Run Until White", [](){motor_controller.run_until_white();}}

}};

Menu menu = { {
    {"Run", run},
    {"Tests", [](){tests.menu();}},
    {"Calibrate Sensors", cali_sensors}
} };

void loop() {
    menu.menu();
}