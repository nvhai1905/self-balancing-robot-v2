#include <PID_v1.h>
#include <LMotorController.h>
#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"

#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
#include "Wire.h"
#endif

#define MIN_ABS_SPEED 30

MPU6050 mpu;

// MPU control/status vars
bool dmpReady = false;
uint8_t mpuIntStatus;
uint8_t devStatus;
uint16_t packetSize;
uint16_t fifoCount;
uint8_t fifoBuffer[64];

// orientation/motion vars
Quaternion q;
VectorFloat gravity;
float ypr[3];

// PID
double originalSetpoint = 178.0;
double setpoint = originalSetpoint;
double movingAngleOffset = 1;
double input, output;

// Adjust these values to fit your own design
double Kp = 40;
double Kd = 1.3;
double Ki = 150;
PID pid(&input, &output, &setpoint, Kp, Ki, Kd, DIRECT);

double motorSpeedFactorLeft = 0.5;
double motorSpeedFactorRight = 0.5;

// MOTOR CONTROLLER
int ENA = 11;
int IN1 = 7;
int IN2 = 6;
int IN3 = 5;
int IN4 = 4;
int ENB = 10;
LMotorController motorController(ENA, IN1, IN2, ENB, IN3, IN4, motorSpeedFactorLeft, motorSpeedFactorRight);

// Biến để lưu trạng thái điều khiển
char currentCommand = 'S';  // 'S' nghĩa là dừng và ưu tiên cân bằng

volatile bool mpuInterrupt = false;
void dmpDataReady() {
    mpuInterrupt = true;
}

void setup() {
    Serial.begin(115200);

    #if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
    Wire.begin();
    TWBR = 24; // 400kHz I2C clock
    #endif

    mpu.initialize();
    devStatus = mpu.dmpInitialize();

    mpu.setXGyroOffset(0);
    mpu.setYGyroOffset(0);
    mpu.setZGyroOffset(0);
    mpu.setZAccelOffset(0);

    if (devStatus == 0) {
        mpu.setDMPEnabled(true);
        attachInterrupt(0, dmpDataReady, RISING);
        mpuIntStatus = mpu.getIntStatus();
        dmpReady = true;
        packetSize = mpu.dmpGetFIFOPacketSize();

        pid.SetMode(AUTOMATIC);
        pid.SetSampleTime(10);
        pid.SetOutputLimits(-255, 255);
    } else {
        Serial.print(F("DMP Initialization failed (code "));
        Serial.print(devStatus);
        Serial.println(F(")"));
    }
}

// Các hàm điều khiển
void moveForward() {
    setpoint = originalSetpoint - movingAngleOffset;
    Serial.println("Moving forward");
}

void moveBackward() {
    setpoint = originalSetpoint + movingAngleOffset;
    Serial.println("Moving backward");
}

void turnLeft() {
    setpoint = originalSetpoint;  // giữ cân bằng góc
    // Khi quay trái, giảm tốc độ động cơ phải và tăng hoặc giữ tốc độ động cơ trái
    // Hoặc đảo chiều quay động cơ bên trái so với phải để quay tại chỗ
    motorController.move(output * 0.5, -output * 0.5);  
    Serial.println("Turning left");
}

void turnRight() {
    setpoint = originalSetpoint;  
    motorController.move(-output * 0.5, output * 0.5);
    Serial.println("Turning right");
}

void stopMotors() {
    setpoint = originalSetpoint;
    Serial.println("Stopping");
}

void loop() {
    // Đọc lệnh điều khiển từ Serial
    if (Serial.available() > 0) {
        char command = Serial.read();
        if (command == 'F' || command == 'B' || command == 'L' || command == 'R' || command == 'S') {
            currentCommand = command;
        }
    }

    if (!dmpReady) return;

    // Đọc dữ liệu MPU
    while (!mpuInterrupt && fifoCount < packetSize) {
        pid.Compute();
        int currentSpeed = output;

        if (currentCommand == 'L') {
            // Quay trái
            motorController.move(currentSpeed * 0.7, MIN_ABS_SPEED);
        } else if (currentCommand == 'R') {
            // Quay phải
            motorController.move(currentSpeed * 0.7, MIN_ABS_SPEED);
        } else if (currentCommand == 'F' || currentCommand == 'B') {
            motorController.move(currentSpeed, MIN_ABS_SPEED);
        } else {
            motorController.move(currentSpeed, MIN_ABS_SPEED);  // dừng và cân bằng
        }
    }

    mpuInterrupt = false;
    mpuIntStatus = mpu.getIntStatus();
    fifoCount = mpu.getFIFOCount();

    if ((mpuIntStatus & 0x10) || fifoCount == 1024) {
        mpu.resetFIFO();
        Serial.println(F("FIFO overflow!"));
    } else if (mpuIntStatus & 0x02) {
        while (fifoCount < packetSize)
            fifoCount = mpu.getFIFOCount();

        mpu.getFIFOBytes(fifoBuffer, packetSize);
        fifoCount -= packetSize;

        mpu.dmpGetQuaternion(&q, fifoBuffer);
        mpu.dmpGetGravity(&gravity, &q);
        mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
        input = ypr[1] * 180 / M_PI + 180;

        Serial.print("ANGLE:");
        Serial.println(input, 2);

        // Giới hạn góc để quyết định có cho điều khiển không
        if (currentCommand == 'S' || currentCommand == 0) {
            stopMotors(); // Không lệnh: giữ cân bằng
        } else if (input > 172 && input < 176) {
            // Góc an toàn, cho phép tất cả lệnh
            switch (currentCommand) {
                case 'F': moveForward(); break;
                case 'L': turnLeft(); break;
                case 'R': turnRight(); break;
                case 'S': stopMotors(); break;
            }
        } else if (input > 180 && input < 184) {
            // Góc nghiêng sau, chỉ cho phép lùi
            if (currentCommand == 'B') moveBackward();
            else stopMotors();
        } else {
            // Ngoài vùng an toàn, dừng và cân bằng
            stopMotors();
            currentCommand = 'S';
        }
    }
}
