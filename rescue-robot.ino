#define DEBUG
#include <draw/draw.hpp>
#include <draw/menu.hpp>
#include <rescueConfig.h>
#include <furelise.h>

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
    delay(200);
    servo(1, 180);
    delay(200);
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
    motor_controller.run_until_white();
    motor_controller.run_until_black();
    motor_controller.rotate_to(-90.0);
    motor_controller.align();
    motor_controller.run_until_black();
    motor_controller.rotate_to(0.0);
    motor_controller.run_until_black();
    motor_controller.rotate_to(90.0);
    motor_controller.run_until_black(0.0, false);
    deploy_dice();
    motor_controller.run(-0.75);

    motor_controller.rotate_to(-90.0);
    motor_controller.run_until_black();
    motor_controller.rotate_to(180.0);
    motor_controller.run_until_black();
    motor_controller.rotate_to(90.0);
    motor_controller.align();
    motor_controller.run(1.0);
    motor_controller.rotate_to(180.0); //bridge
    motor_controller.align();
    motor_controller.run_until_black();
    motor_controller.rotate_to(-90.0);
    motor_controller.align();
    motor_controller.run_until_black(); //off bridge
    motor_controller.rotate_to(0);
    motor_controller.align();
    motor_controller.run(1.0);
    motor_controller.rotate_to(90);
    motor_controller.run_until_black(0.0, true, 140);//checkpoint
    motor_controller.rotate_to(-90);
    motor_controller.run_until_black();
    motor_controller.rotate_to(0);
    motor_controller.run_until_black();
    motor_controller.rotate_to(-90);
    motor_controller.align();
    motor_controller.run(1);
    motor_controller.rotate_to(180.0);
    motor_controller.align();
    motor_controller.run_until_black(0.0); //checkpoint
    motor_controller.rotate_to(0);
    motor_controller.align();
    motor_controller.run_until_black();
    motor_controller.rotate_to(-90);
    motor_controller.run_until_black();
    motor_controller.rotate_to(180);
    motor_controller.run_until_black();
    deploy_dice(); //red
    motor_controller.run(-0.5);
    motor_controller.rotate_to(-90);
    motor_controller.align();
    motor_controller.run_until_black();
    motor_controller.rotate_to(0);
    motor_controller.align();
    motor_controller.run_until_black();
    motor_controller.rotate_to(-90);
    motor_controller.run_until_black();
    motor_controller.rotate_to(180);
    motor_controller.run_until_black();
    motor_controller.run(1);
    motor_controller.run_until_black();
    motor_controller.rotate_to(90);
    motor_controller.align();
    motor_controller.run_until_black();
    deploy_dice(); //blue
    motor_controller.rotate_to(-90);
    motor_controller.run_until_black();
    motor_controller.rotate_to(0);
    motor_controller.align();
    motor_controller.run_until_black();
    motor_controller.run(1);
    motor_controller.align();
    motor_controller.run_until_black();
    motor_controller.rotate_to(90);
    motor_controller.align();
    motor_controller.run(2.0);
    motor_controller.run_until_black(0.0, true, 100);
    motor_controller.run(-0.25);
    deploy_dice();
    flag();
    beep();
    beep();
    beep();
    beep();
    beep();
    beep();
    beep();
    play_fur_elise();
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
    {"Test Rotate 90", [](){motor_controller.imu_sensor->Reset(); motor_controller.rotate_to(-90.0);}},
    {"Test Rotate 180", [](){motor_controller.imu_sensor->Reset(); motor_controller.rotate_to(180.0);}},
    {"Check Front", [](){motor_controller.check_front();}},
    {"Run Until Black", [](){motor_controller.run_until_black();}}
} };

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