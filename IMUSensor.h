#include <POP32.h>

// ZX-IMU Sensor, for 'YAW' value, use Serial1 to communicate (static).
// MUST use .Init() to initialize the sensor before use.
class IMUSensor{
    public:
        float pvYaw = 99.0f; // Current yaw value
        bool pvStatus = false; // Status of IMU sensor
        uint8_t rxBuf[8];

        IMUSensor(){}

        // Initialize the IMU sensor with 115200 baud rate.
        // WARNING: This function will take about 3050ms to complete.
        void Init(){
            Serial1.begin(115200);
            delay(1000);
            zeroYaw();

            // Set Mode to 'Automatic'
            Serial1.write(0xA5);
            Serial1.write(0x52);
            delay(100);
        }

        // Set Pitch, Roll and Yaw to zero
        // WARNING: delay upto 1950ms
        inline void zeroYaw() {
            // Set Pitch & Roll to zero
            Serial1.write(0xA5);
            Serial1.write(0x54);
            delay(1800);       
            // Set Yaw to zero
            Serial1.write(0xA5);
            Serial1.write(0x55);
            delay(150);
        }

        // Repeated yaw-zero until near zero
        //
        // 'precision' : the precision to stop zeroing, default is 0.02f
        // 'time_out' : the maximum time to wait for zeroing, default is 10'000ms
        void AutoZero(float_t precision = 0.02f,int32_t time_out = 10000) {
            zeroYaw();
            unsigned long start = HAL_GetTick();
            while (true) {
                if (fetchIMU() && fabs(pvYaw) <= precision) break;
                if (HAL_GetTick() - start > time_out) {
                    zeroYaw();
                    break;
                }
            }
        }

        // Fetch data from IMU sensor, return true if successful
        // pvYaw will be updated with the latest yaw value
        inline bool fetchIMU() {
            static uint8_t idx = 0;
            while (Serial1.available()) {
                uint8_t b = Serial1.read();
                if (idx == 0 && b != 0xAA) continue;
                rxBuf[idx++] = b;
                if (idx == 8) {
                    idx = 0;
                    if (rxBuf[7] == 0x55) {
                        pvYaw = (int16_t)(rxBuf[1] << 8 | rxBuf[2]) / 100.0f;
                        pvStatus = true;
                        return true;
                    }
                }
            }
            pvStatus = false;
            return false;
        }

        // Get the current yaw value
        //
        // 'do_fetch' : true - fetching new data, flase - using previous value
        inline float_t getYaw(bool do_fetch = true){
            if (do_fetch) {
                if (!fetchIMU()) return pvYaw; // If fetch failed, return previous value
            }
            return pvYaw;
        }

        // Log the current yaw value, status of IMU sensor, to OLED screen. 
        // this function DO NOT update, fetch the IMU sensor.
        //
        // 'doBeep' : true - beep when logging, false - no beep
        inline void LogYaw(bool doBeep = false){
            if(doBeep)beep();
            oled.clear();
            oled.text(0,0,"Yaw=%f", pvYaw);
            oled.text(1,0,"Status: %s", pvStatus ? "OK" : "Error");
            oled.show();
        }
        
};