#include <AccelStepper.h>

#define PID_OPTIMIZED_I

#include <Arduino.h>
#include <SoftwareSerial.h>
#include <Wire.h>
#include <TroykaIMU.h>
#include "MeOrion.h"
#include <math.h>
#include "GyverTimer.h"
#include "GyverPID.h"

#define K 0.99
#define BALANCING_LIMITS 40 // Границы балансая
#define BLACK_LEFT_LINE_S 480 // Сырые значения с датчика линии
#define WHITE_LEFT_LINE_S 45
#define BLACK_RIGHT_LINE_S 480
#define WHITE_RIGHT_LINE_S 45
#define MAX_VAL_POT 976
#define N_MEASURE_SPEED 15
#define N_MEASURE_ULTRASONIC 15

#define STEPPER_X_DIR_PIN mePort[PORT_1].s1
#define STEPPER_X_STP_PIN mePort[PORT_1].s2
#define STEPPER_Y_DIR_PIN mePort[PORT_2].s1
#define STEPPER_Y_STP_PIN mePort[PORT_2].s2
AccelStepper stepperL(AccelStepper::DRIVER, STEPPER_X_STP_PIN, STEPPER_X_DIR_PIN);
AccelStepper stepperR(AccelStepper::DRIVER, STEPPER_Y_STP_PIN, STEPPER_Y_DIR_PIN);

MeUltrasonicSensor ultrasonic(3);
MePort lineSensors(PORT_7);
Me7SegmentDisplay seg7(4);
//MeStepper stepperL(PORT_1);
//MeStepper stepperR(PORT_2); 
Accelerometer accel;
Gyroscope gyro;

float accelY, accelZ, accAngle;
float gyroX, gyroAngle;
float cfAngle, cfAngle_old;
float vKp = 0, vKi = 0, vKd = 0;
float bKp = 75, bKi = 0, bKd = 0;
float lKp = 0, lKi = 0, lKd = 0;

unsigned long currTime, prevTime, loopTime;
float balanceSetPoint = 1;
int targetSpeed = 30;
int u_left, u_right;
float u_speed, u_balance, u_lineFollower;
int currentUltasonicDist;
float angleFixrate = 0.0008;

GTimer_ms myTimer1(10), myTimer2(100);
GyverPID regulator_b(bKp, bKi, bKd, 10);
GyverPID regulator_v(vKp, vKi, vKd, 10);
GyverPID regulator_l(lKp, lKi, lKd, 10);

void setup(){
  Serial.begin(115200);
  Serial.println();
  Serial.println("Initialization...");
  accel.begin();
  gyro.begin();
  stepperL.setAcceleration(20000); stepperR.setAcceleration(20000);
  stepperL.setMaxSpeed(2000); stepperR.setMaxSpeed(2000);
  regulator_b.setDirection(REVERSE);
  regulator_b.setLimits(-10000, 10000);
  Serial.println("Initialization completed");
  buzzerOff();
}

void loop()
{
  currTime = millis();
  loopTime = currTime - prevTime;
  prevTime = currTime;
  SerialData();
  if (myTimer1.isReady()) {
    accelY = accel.readAY();
    accelZ = accel.readAZ();
    accAngle = atan2(accelY, accelZ) * RAD_TO_DEG;
    gyroX = gyro.readDegPerSecX();
    gyroAngle = (float)gyroX * loopTime / 1000;
    cfAngle = K * (cfAngle_old + gyroAngle) + (1 - K) * accAngle; // Комплементарный фильтр от двух датчиковx 
//    Serial.print("angle: "); Serial.println(cfAngle);
//    Serial.println(currTime);
    if (myTimer2.isReady()) seg7.display(cfAngle);
    if (abs(cfAngle) < BALANCING_LIMITS) {
      /*
      if (cfAngle < balanceSetPoint) {
        balanceSetPoint += angleFixrate * loopTime;
      } else {
        balanceSetPoint -= angleFixrate * loopTime;
      }
      */
      /*
      if (balanceSetPoint - 0.5 <= cfAngle and  <= balanceSetPoint + 0.5){
        regulator_b.integral = 0;
      }
      */
      // Speed
      int lMotSpeed = stepperL.speed(), rMotSpeed = stepperR.speed();
      Serial.print("lMotSpeed: "); Serial.print(lMotSpeed); Serial.print(" "); Serial.print("rMotSpeed: "); Serial.print(rMotSpeed); Serial.print("\t");
      int currentSpeed = (lMotSpeed + rMotSpeed) / 2;
      //Serial.print("speed: "); Serial.print(currentSpeed); Serial.print("\t");

      // Speed PID
      targetSpeed = 0;
      float errorSpeed = currentSpeed - targetSpeed;
      regulator_v.setpoint = errorSpeed;
      u_speed = regulator_v.getResultTimer();
      //int u_speed = 0;

      // Balance PID
      float targetAngle = balanceSetPoint - u_speed;
      float errorAngle = cfAngle - targetAngle;
      regulator_b.setpoint = errorAngle;
//      regulator_b.setDt(loopTime);
      u_balance = regulator_b.getResultTimer();

      // Line Sensors
      //int leftLineS = lineSensors.aRead1(); //Serial.print("l: "); Serial.print(leftLineS); Serial.print("\t"); // Для вывода сырых значений левого
      //int rightLineS = lineSensors.aRead2(); //Serial.print("r: "); Serial.print(rightLineS); Serial.print("\t"); // Для вывода сырых значений правого
      //leftLineS = map(leftLineS, BLACK_LEFT_LINE_S, WHITE_LEFT_LINE_S, 0, 255);
      //leftLineS = constrain(leftLineS, 0, 255); //Serial.print("leftLineS: "); Serial.print(leftLineS); Serial.print("\t");
      //rightLineS = map(rightLineS, BLACK_RIGHT_LINE_S, WHITE_RIGHT_LINE_S, 0, 255);
      //rightLineS = constrain(rightLineS, 0, 255); //Serial.print("rightLineS: "); Serial.print(rightLineS); Serial.print("\t");

      // LineFollower PID
      //float errorLineFollower = leftLineS - rightLineS;
      //u_lineFollower = PID_Control(2, errorLineFollower, 0.05, 0, 0, loopTime, false);
      //u_lineFollower = constrain(u_lineFollower, -25, 25);
      
      // Ultrasonic distance
      //currentUltasonicDist = ultrasonic.distanceCm();

      u_left = u_balance; u_right = -u_balance;
//      u_left = constrain(u_left, -255, 255); u_right = constrain(u_right, -255, 255);
      Serial.println(cfAngle);
      stepperL.setSpeed(u_left); stepperR.setSpeed(u_right);
    }
    else {
      stepperL.setSpeed(0); stepperR.setSpeed(0);
    }
    cfAngle_old = cfAngle;
  }  
  
  stepperL.runSpeed(); stepperR.runSpeed();
}

void SerialData(){
  if(Serial.available() > 2) {
    String command = Serial.readStringUntil('\n');
    command.trim();
    command.replace(" ", ""); // Убрать возможные пробелы между символами
    byte strIndex = command.length(); // Переменая для хронения индекса вхождения цифры в входной строке
    // Поиск первого вхождения цифры от 0 по 9 в подстроку
    for (byte i = 0; i < 10; i++) {
      byte index = command.indexOf(String(i));
      if (index < strIndex && index != 255) strIndex = index;
    }
    String incoming = command.substring(0, strIndex);
    String valueStr = command.substring(strIndex, command.length());
    float value = valueStr.toFloat();
    if (incoming == "bp") {
      bKp = value;
      Serial.print("bp  ");
      Serial.println(value);
      regulator_b.Kp = bKp;
    } else if (incoming == "bi") {
      bKi = value;
      Serial.print("bi  ");
      Serial.println(value);
      regulator_b.Ki = bKi;
      regulator_b.integral = 0;
    } else if (incoming == "bd") {
      bKd = value;
      Serial.print("bd  ");
      Serial.println(value);
      regulator_b.Kd = bKd;
    } else if (incoming == "bsp") {
      balanceSetPoint = value;
      Serial.print("bsp  ");
      Serial.println(value);
      regulator_b.integral = 0;
    } else if (incoming == "af") {
      angleFixrate = value;
      Serial.print("af  ");
      Serial.println(value);
    }
  }
}
