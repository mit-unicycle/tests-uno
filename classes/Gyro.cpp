#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"

#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
    #include "Wire.h"
#endif

volatile bool mpuInterrupt = false;     // indicates whether MPU interrupt pin has gone high
void dmpDataReady() {
    mpuInterrupt = true;
}

/**
 * Class that approximates the absolute angle using the data from an mpu6050, 
 * and uses the integrated DMP ship for accurate measurements
 * 
 * Very important: do not import mpu6050 library before this point as it will cause conflicts
 * 
 * Libraries used: 
 *  - MPU6050 by Electronic Cats
 * 
 *   board   Lolin         Description
    ======= ==========    ====================================================
    VCC     VU (5V USB)   Not available on all boards so use 3.3V if needed.
    GND     G             Ground
    SCL     D1 (GPIO05)   I2C clock
    SDA     D2 (GPIO04)   I2C data
    XDA     not connected
    XCL     not connected
    AD0     not connected
    INT     D8 (GPIO15)   Interrupt pin
*/
class Gyro {
    private:
        MPU6050 mpu;
        //MPU6050 mpu(0x69); // <-- use for AD0 high
        
        // MPU control/status vars
        bool dmpReady = false;  // set true if DMP init was successful
        uint8_t mpuIntStatus;   // holds actual interrupt status byte from MPU
        uint8_t devStatus;      // return status after each device operation (0 = success, !0 = error)
        uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)
        uint16_t fifoCount;     // count of all bytes currently in FIFO
        uint8_t fifoBuffer[64]; // FIFO storage buffer

        // orientation/motion vars
        Quaternion q;           // [w, x, y, z]         quaternion container
        VectorInt16 aa;         // [x, y, z]            accel sensor measurements
        VectorInt16 aaReal;     // [x, y, z]            gravity-free accel sensor measurements
        VectorInt16 aaWorld;    // [x, y, z]            world-frame accel sensor measurements
        VectorFloat gravity;    // [x, y, z]            gravity vector

        float ypr[3];           // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector

        #define INTERRUPT_PIN 2

        void mpu_setup() {
            // join I2C bus (I2Cdev library doesn't do this automatically)
            #if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
                Wire.begin();
                Wire.setClock(200000); // 400kHz I2C clock. Comment this line if having compilation difficulties
                Wire.setWireTimeout(3000, true);
            #elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
                Fastwire::setup(400, true);
            #endif

            // initialize device
            Serial.println(F("Initializing I2C devices..."));
            mpu.initialize();
            pinMode(INTERRUPT_PIN, INPUT);

            // verify connection
            Serial.println(F("Testing device connections..."));
            Serial.println(mpu.testConnection() ? F("MPU6050 connection successful") : F("MPU6050 connection failed"));

            // load and configure the DMP
            Serial.println(F("Initializing DMP..."));
            devStatus = mpu.dmpInitialize();

            // At home: Your offsets:   -2209   2119    1327    -68     -78     22
            // mounted -1375   -3760   1042    -49     54      81
            // supply your own gyro offsets here, scaled for min sensitivity
            mpu.setXAccelOffset(-2209);
            mpu.setYAccelOffset(2119);
            mpu.setZAccelOffset(1327);
            mpu.setXGyroOffset(-68);
            mpu.setYGyroOffset(-78);
            mpu.setZGyroOffset(22);

            // make sure it worked (returns 0 if so)
            if (devStatus == 0) {
                // turn on the DMP, now that it's ready
                Serial.println(F("Enabling DMP..."));
                mpu.setDMPEnabled(true);

                // enable Arduino interrupt detection
                Serial.println(F("Enabling interrupt detection (Arduino external interrupt 0)..."));
                attachInterrupt(digitalPinToInterrupt(INTERRUPT_PIN), dmpDataReady, RISING);
                mpuIntStatus = mpu.getIntStatus();

                // set our DMP Ready flag so the main loop() function knows it's okay to use it
                Serial.println(F("DMP ready! Waiting for first interrupt..."));
                dmpReady = true;

                // get expected DMP packet size for later comparison
                packetSize = mpu.dmpGetFIFOPacketSize();
            } else {
                // ERROR!
                // 1 = initial memory load failed
                // 2 = DMP configuration updates failed
                // (if it's going to break, usually the code will be 1)
                Serial.print(F("DMP Initialization failed (code "));
                Serial.print(devStatus);
                Serial.println(F(")"));
            }
        }

        void mpu_loop() {
            // if programming failed, don't try to do anything
            if (!dmpReady) return;

            // wait for MPU interrupt or extra packet(s) available
            if (!mpuInterrupt && fifoCount < packetSize) return;

            // reset interrupt flag and get INT_STATUS byte
            mpuInterrupt = false;
            mpuIntStatus = mpu.getIntStatus();

            // get current FIFO count
            fifoCount = mpu.getFIFOCount();
            
            //Serial.println(fifoCount);
            // check for overflow (this should never happen unless our code is too inefficient) 
            // I changed from 1024 to 1000 because I was having erroronous values
            if ((mpuIntStatus & 0x10) || fifoCount == 1024) {
                // reset so we can continue cleanly
                mpu.resetFIFO();
                Serial.println(F("FIFO overflow!"));

            // otherwise, check for DMP data ready interrupt (this should happen frequently)
            } else if (mpuIntStatus & 0x02) {
                // wait for correct available data length, should be a VERY short wait
                while (fifoCount < packetSize) fifoCount = mpu.getFIFOCount();

                // read a packet from FIFO
                mpu.getFIFOBytes(fifoBuffer, packetSize);

                // track FIFO count here in case there is > 1 packet available
                // (this lets us immediately read more without waiting for an interrupt)
                fifoCount -= packetSize;

                // display Euler angles in degrees
                mpu.dmpGetQuaternion(&q, fifoBuffer);
                mpu.dmpGetGravity(&gravity, &q);
                mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
                
                /*
                Serial.print(ypr[0] * 180/M_PI);
                Serial.print("\t");
                Serial.print(ypr[1] * 180/M_PI);
                Serial.print("\t");
                Serial.println(ypr[2] * 180/M_PI);
                */
            }
        }

    public:
        void setup() {
            mpu_setup();
        }

        void loop() {
            mpu_loop();
        }

        /**
         * Return Yaw angle in deg (least accurate)
        */
        float getYaw() {
            return ypr[0] * 180/M_PI;
        }

        /**
         * Return Pitch angle in deg
        */
        float getPitch() {
            return ypr[1] * 180/M_PI;
        }

        /**
         * Return Roll angle in deg
        */
        float getRoll() {
            return ypr[2] * 180/M_PI;
        }
};
