#include <mcp_can.h>
#include <SPI.h>
#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
    #include "Wire.h"
#endif

MPU6050 mpu;

#define INTERRUPT_PIN 2  // use pin 2 on Arduino Uno & most boards
#define LED_PIN 13 // (Arduino is 13, Teensy is 11, Teensy++ is 6)
bool blinkState = false;

bool dmpReady = false;  // set true if DMP init was successful
uint8_t mpuIntStatus;   // holds actual interrupt status byte from MPU
uint8_t devStatus;      // return status after each device operation (0 = success, !0 = error)
uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)
uint16_t fifoCount;     // count of all bytes currently in FIFO
uint8_t fifoBuffer[64]; // FIFO storage buffer
int16_t gyro[3];

Quaternion q;           // [w, x, y, z]         quaternion container
VectorInt16 aa;         // [x, y, z]            accel sensor measurements
VectorInt16 aaReal;     // [x, y, z]            gravity-free accel sensor measurements
VectorInt16 aaWorld;    // [x, y, z]            world-frame accel sensor measurements
VectorFloat gravity;    // [x, y, z]            gravity vector
float euler[3];         // [psi, theta, phi]    Euler angle container
float ypr[3];           // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector

uint8_t teapotPacket[14] = { '$', 0x02, 0,0, 0,0, 0,0, 0,0, 0x00, 0x00, '\r', '\n' };

const int SPI_CS_PIN = 10;
int flag = 0;
int canID1 = (0x001 << 5) + 0x00D;
int canID2 = (0x003 << 5) + 0x00D;
byte stmp1[8];
byte stmp2[8];
int CH1 = 3;
int CH2 = 5;
float ang_vel_1;
float ang_vel_2;
float v_x;
float v_y;

MCP_CAN CAN(SPI_CS_PIN);   // Set CS pin

volatile bool mpuInterrupt = false;     // indicates whether MPU interrupt pin has gone high
void dmpDataReady() {
    mpuInterrupt = true;
}

void setup()
{
    #if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
        Wire.begin();
        Wire.setClock(400000); // 400kHz I2C clock. Comment this line if having compilation difficulties
    #elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
        Fastwire::setup(400, true);
    #endif

    Serial.begin(115200);


    mpu.initialize();
    pinMode(INTERRUPT_PIN, INPUT);
    devStatus = mpu.dmpInitialize();
    mpu.setXGyroOffset(220);
    mpu.setYGyroOffset(76);
    mpu.setZGyroOffset(-85);
    mpu.setZAccelOffset(1788);

    if (devStatus == 0) {
        // Calibration Time: generate offsets and calibrate our MPU6050
        mpu.CalibrateAccel(6);
        mpu.CalibrateGyro(6);
        mpu.PrintActiveOffsets();
        // turn on the DMP, now that it's ready
        mpu.setDMPEnabled(true);
        attachInterrupt(digitalPinToInterrupt(INTERRUPT_PIN), dmpDataReady, RISING);
        mpuIntStatus = mpu.getIntStatus();

                dmpReady = true;

        // get expected DMP packet size for later comparison
        packetSize = mpu.dmpGetFIFOPacketSize();
    }
    
    else {
    }

    // configure LED for output
    pinMode(LED_PIN, OUTPUT);

    CAN.begin(CAN_500KBPS);}

void loop()
{
    unsigned long ch1 = pulseIn(CH1, HIGH);
    unsigned long ch2 = pulseIn(CH2, HIGH);

    ch2 = map(ch2,980,2100,2100,980);

    if (abs(ch1-1500)<100){
      ch1 = 1540;
    }

    if (abs(ch2-1500)<100){
      ch2 = 1540;
    }

    v_x = -3.0f+6.0f*(ch1-980)/(2100-980);
    v_y = -3.0f+6.0f*(ch2-980)/(2100-980);


//0.12f*gyro[2]*M_PI/180.0f
    if (!dmpReady) return;
    // read a packet from FIFO
    if (mpu.dmpGetCurrentFIFOPacket(fifoBuffer)) {
            mpu.dmpGetGyro(gyro, fifoBuffer);
            mpu.dmpGetQuaternion(&q, fifoBuffer);
            mpu.dmpGetGravity(&gravity, &q);
            mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
            Serial.println(ypr[0]);
            
    ang_vel_1 = (v_x*(cos(ypr[0])+sin(ypr[0]))+v_y*(sin(ypr[0])-cos(ypr[0]))+0.12f*gyro[2]*M_PI/180.0f)/(0.06f*M_PI);
    ang_vel_2 = (v_x*(cos(ypr[0])-sin(ypr[0]))+v_y*(sin(ypr[0])+cos(ypr[0]))+0.12f*gyro[2]*M_PI/180.0f)/(0.06f*M_PI);
    }
    
    //Serial.println(ang_vel_1);
    //Serial.println(ang_vel_2);


    float set_point1 = ang_vel_1;
    float set_point2 = -ang_vel_2;

    memset(stmp1, 0, sizeof(float));  // Clear the float space
    memcpy(stmp1, &set_point1, sizeof(float)); // Copy the position into the data

    memset(stmp2, 0, sizeof(float));  // Clear the float space
    memcpy(stmp2, &set_point2, sizeof(float)); // Copy the position into the data

    if (flag ==0){
      CAN.sendMsgBuf(canID1, 0, 8, stmp1);
      flag = 1;
    }
    else {
      CAN.sendMsgBuf(canID2, 0, 8, stmp2);
      flag = 0;
    }

}
