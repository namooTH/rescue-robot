#include "draw/draw.hpp"
#include "draw/menu.hpp"
#include "rescueConfig.h"
#include "badapple.h"

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
  motor_controller.rotate_to(90);
  motor_controller.align();
  motor_controller.run_until_black();
  motor_controller.rotate_to(180);
  motor_controller.run_until_black();
  motor_controller.rotate_to(-90);
  motor_controller.run_until_black();
  motor_controller.rotate_to(180);
  motor_controller.run_until_black();
  motor_controller.rotate_to(-90);
  motor_controller.run_until_black();
  motor_controller.rotate_to(0);
  motor_controller.run_until_black();
  deploy_dice();  //blue

  motor_controller.run_until_black(0.0, true, true);
  motor_controller.rotate_to(-90);
  motor_controller.align(false);
  motor_controller.run_until_black(0.0, true, true);
  motor_controller.rotate_to(180);
  motor_controller.run_until_black();
  motor_controller.rotate_to(-90);
  motor_controller.run_until_black(0.0, true, false, 120); // checkpoint 2
  motor_controller.run_until_black(0.0, true, true, 120);
  motor_controller.rotate_to(0);
  motor_controller.run_until_black();
  motor_controller.rotate_to(-90);
  motor_controller.run(0.9);
  motor_controller.rotate_to(0);
  motor_controller.run_until_black();
  motor_controller.rotate_to(90);
  motor_controller.run_until_black();
  motor_controller.rotate_to(0);
  motor_controller.run_until_black();
  motor_controller.rotate_to(-90);
  motor_controller.run(0.9);
  motor_controller.rotate_to(0);
  motor_controller.run_until_black();
  motor_controller.rotate_to(-90);
  motor_controller.run_until_black();
  motor_controller.rotate_to(180);
  motor_controller.run_until_black();
  motor_controller.rotate_to(90);
  motor_controller.run_until_black();
  deploy_dice();  //red

  motor_controller.run_until_black(0.0, true, true);
  motor_controller.rotate_to(0);
  motor_controller.run_until_black(0.0);
  motor_controller.rotate_to(90);
  motor_controller.run_until_black(0.0);
  motor_controller.rotate_to(0);
  motor_controller.run_until_black(0.0);
  motor_controller.rotate_to(-90);
  motor_controller.run_until_black(0.0, false);
  motor_controller.run_until_white();
  motor_controller.run_until_black(0.0, true, false, 178, 0.5);
  deploy_dice();  //green
  motor_controller.run_until_black(0.0, false, true);
  motor_controller.run_until_white(true);
  motor_controller.run_until_black(0.0, true, true);
  motor_controller.rotate_to(180);
  motor_controller.align();
  motor_controller.run_until_black();
  motor_controller.run(-0.35);
  deploy_dice();  //yellow
  flag();

  motor_controller.stop();
  badapple();
}


Menu tests = { { { "Test Motor", []() {
                    while (1) motor_controller.move(100, 0.0);
                  } },
                 { "Test IMU", []() {
                    while (1) {
                      clear();
                      drawTextFmt(0, 0, WHITE, "%f", imu_sensor.getYaw());
                      drawTextFmt(0, 10, WHITE, "SW_OK to Reset");
                      flip();
                      if (SW_OK()) {
                        imu_sensor.Reset();
                      }
                    }
                  } },
                 { "Test Dice", deploy_dice },
                 { "Test Flag", flag },
                 { "Test DeFlag", deflag },
                 { "Align", []() {
                    motor_controller.align(true);
                  } },
                 { "Test Rotate 90", []() {
                    motor_controller.imu_sensor->Reset();
                    motor_controller.rotate_to(-90.0);
                  } },
                 { "Test Rotate 180", []() {
                    motor_controller.imu_sensor->Reset();
                    motor_controller.rotate_to(180.0);
                  } },
                 { "Run Until Black", []() {
                    motor_controller.run_until_black();
                  } },
                 { "Run Until Black Backward", []() {
                    motor_controller.run_until_black(0.0, true, true);
                  } } } };

Menu menu = { { { "Run", run },
                { "Tests", []() {
                   tests.menu();
                 } },
                { "Calibrate Sensors", cali_sensors } } };

void loop() {
  menu.menu();
}