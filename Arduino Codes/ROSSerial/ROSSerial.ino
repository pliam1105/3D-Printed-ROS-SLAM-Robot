#include <Servo.h>
#include <PID_v1.h>
#include "SerialTransfer.h"

//servos
#define servo1 22
#define servo2 23
#define servo3 24
#define servo4 25

//left motor
#define leftRPWM 4
#define leftLPWM 5
#define leftENR 8
#define leftENL 9

//right motor
#define rightRPWM 6
#define rightLPWM 7
#define rightENR 10
#define rightENL 11

//left encoder
#define leftA 2
#define leftB 3

//right encoder
#define rightA 19
#define rightB 18

//servo variables
Servo Servo1, Servo2, Servo3, Servo4;

const float valueRange1[] = {30, 80};
const float angleRange1[] = {-20, 30};

const float valueRange2[] = {40, 110};
const float angleRange2[] = {10, 80};

const float valueRange3[] = {90, 160};
const float angleRange3[] = {-50, 20};
const float angleConstrain3[] = {50, 120}; //wrt the previous link, this is where angle3 lies

const float valueRange4[] = {50, 0};
const float angleRange4[] = {25, 75};

const double metersToPulses = 1670.0; //for meter -> pulses conversion
const double wheelDistance = 0.25; //for angular velocity computation

//encoder variables
volatile long leftCounter = 0, rightCounter = 0;
const long encoderInterval = 10;
unsigned long lastTime, currentTime, startTime;
volatile bool changeLeft = false, changeRight = false;
double leftEncoderSpeed, rightEncoderSpeed;//measured speeds

//PID variables
const double KP = 1, KI = 1.5, KD = 0.05;
double leftMotorSpeed, rightMotorSpeed;
double leftOutput, rightOutput;
double leftPWM, rightPWM;

//ROS Twist linear and angular components
double linearX, angularZ;
int stop = 1;

PID leftSpeedPID(&leftEncoderSpeed, &leftOutput, &leftMotorSpeed, KP, KI, KD, DIRECT);
PID rightSpeedPID(&rightEncoderSpeed, &rightOutput, &rightMotorSpeed, KP, KI, KD, DIRECT);

//Serial communication
SerialTransfer ros_transfer;

struct __attribute__((packed)) vel_struct{
  float linear_x, angular_z;
  int32_t stop;
};

struct __attribute((packed)) real_speed_struct{
  float speed_left, speed_right;
};

uint16_t packet_size = 0;

void setup() {
  //initialize servos
  float initialAngles[4] = {0, 10, 50, 25};
  moveServos(initialAngles);

  //attach servos
  Servo1.attach(servo1);
  Servo2.attach(servo2);
  Servo3.attach(servo3);
  Servo4.attach(servo4);

  //define motor pin modes
  pinMode(leftRPWM, OUTPUT);
  pinMode(leftLPWM, OUTPUT);
  pinMode(rightRPWM, OUTPUT);
  pinMode(rightLPWM, OUTPUT);
  pinMode(leftENR, OUTPUT);
  pinMode(leftENL, OUTPUT);
  pinMode(rightENR, OUTPUT);
  pinMode(rightENL, OUTPUT);

  //enable motor movement in both directions
  digitalWrite(leftENL, HIGH);
  digitalWrite(leftENR, HIGH);
  digitalWrite(rightENL, HIGH);
  digitalWrite(rightENR, HIGH);

  //Serial communication
  Serial.begin(115200);
  ros_transfer.begin(Serial);

  //set encoder pin modes
  pinMode(leftA, INPUT_PULLUP);
  pinMode(leftB, INPUT_PULLUP);
  pinMode(rightA, INPUT_PULLUP);
  pinMode(rightB, INPUT_PULLUP);

  attachInterrupt(0, changeLeftA, RISING);
  attachInterrupt(4, changeRightA, RISING);

  lastTime = startTime = millis();

  //PID setup
  leftSpeedPID.SetOutputLimits(-100, 100);
  rightSpeedPID.SetOutputLimits(-100, 100);
  leftSpeedPID.SetSampleTime(10);
  rightSpeedPID.SetSampleTime(10);
  leftSpeedPID.SetMode(AUTOMATIC);
  rightSpeedPID.SetMode(AUTOMATIC);

  //wait for handshake call
  while(true){
    if(ros_transfer.available()){
      packet_size = 0;
      int32_t ok;
      packet_size = ros_transfer.rxObj(ok, packet_size);//should be 100
      if(ok == 100){
        //send response
        packet_size = 0;
        packet_size = ros_transfer.txObj((int32_t)200, packet_size);
        ros_transfer.sendData(packet_size);
        break;
      }
    }
  }
}

void loop() {
  //receiving commands
  if(ros_transfer.available()){
    //receive packet
    packet_size = 0;
    vel_struct new_vel = vel_struct();
    packet_size = ros_transfer.rxObj(new_vel, packet_size);
    float servoAngles[4];
    packet_size = ros_transfer.rxObj(servoAngles, packet_size);

    //take action
    stop = new_vel.stop;
    linearX = new_vel.linear_x;
    angularZ = new_vel.angular_z;
    moveServos(servoAngles);

    //send encoder speeds
    real_speed_struct real_speed = real_speed_struct();
    real_speed.speed_left = leftEncoderSpeed;
    real_speed.speed_right = rightEncoderSpeed;

    packet_size = 0;
    packet_size = ros_transfer.txObj(real_speed.speed_left, packet_size);
    packet_size = ros_transfer.txObj(real_speed.speed_right, packet_size);
    ros_transfer.sendData(packet_size);
  }

  //compute encoder speeds
  currentTime = millis();
  if(currentTime - lastTime >= encoderInterval){
    //compute speeds in pulses/millisecond
    leftEncoderSpeed = ((double)leftCounter)/((double)currentTime - lastTime);
    rightEncoderSpeed = ((double)rightCounter)/((double)currentTime - lastTime);
    //convert to meters/second
    leftEncoderSpeed = leftEncoderSpeed * 1000.0 / metersToPulses;
    rightEncoderSpeed = rightEncoderSpeed * 1000.0 / metersToPulses;
    leftCounter = rightCounter = 0;
    lastTime += encoderInterval;
  }
  
  //compute wheel velocities from linear and angular velocity components
  leftMotorSpeed = linearX - angularZ * wheelDistance / 2.0;
  rightMotorSpeed = linearX + angularZ * wheelDistance / 2.0;
  //apply PID
  leftSpeedPID.Compute();
  rightSpeedPID.Compute();
  leftPWM += leftOutput;
  rightPWM += rightOutput;
  // Serial.print(leftPWM);
  // Serial.print(", ");
  // Serial.println(rightPWM);
  if(leftPWM >=0){
    analogWrite(leftRPWM, leftPWM);
    analogWrite(leftLPWM, 0);
  }else{
    analogWrite(leftRPWM, 0);
    analogWrite(leftLPWM, -leftPWM);
  }
  if(rightPWM >=0){
    analogWrite(rightRPWM, rightPWM);
    analogWrite(rightLPWM, 0);
  }else{
    analogWrite(rightRPWM, 0);
    analogWrite(rightLPWM, -rightPWM);
  }

  if(stop != 1){
    // if(leftSpeedPID.GetMode() == MANUAL) leftSpeedPID.SetMode(AUTOMATIC);
    // if(rightSpeedPID.GetMode() == MANUAL) rightSpeedPID.SetMode(AUTOMATIC);
    digitalWrite(leftENL, HIGH);
    digitalWrite(leftENR, HIGH);
    digitalWrite(rightENL, HIGH);
    digitalWrite(rightENR, HIGH);
  }else{
    // leftSpeedPID.SetMode(MANUAL);
    // rightSpeedPID.SetMode(MANUAL);
    analogWrite(leftLPWM, 0);
    analogWrite(leftRPWM, 0);
    analogWrite(rightLPWM, 0);
    analogWrite(rightRPWM, 0);
    digitalWrite(leftENL, LOW);
    digitalWrite(leftENR, LOW);
    digitalWrite(rightENL, LOW);
    digitalWrite(rightENR, LOW);
    leftPWM = rightPWM = 0;
  }
}

void changeLeftA(){
  if(digitalRead(leftA)!=digitalRead(leftB)){
    //clockwise but reversed -> forward is +
    leftCounter--;
  }else{
    //counterclockwise but reversed
    leftCounter++;
  }
  changeLeft = true;
}

void changeRightA(){
  if(digitalRead(rightA)!=digitalRead(rightB)){
    //clockwise
    rightCounter++;
  }else{
    //counterclockwise
    rightCounter--;
  }
  changeRight = true;
}

void moveServos(float angles[4]){
  //calculate values from angles with map
  float values[4];
  values[0] = map(constrain(angles[0], angleRange1[0], angleRange1[1]), angleRange1[0], angleRange1[1], valueRange1[0], valueRange1[1]);
  values[1] = map(constrain(angles[1], angleRange2[0], angleRange2[1]), angleRange2[0], angleRange2[1], valueRange2[0], valueRange2[1]);
  values[2] = constrain(map(constrain(angles[2], angleConstrain3[0], angleConstrain3[1])-90-angles[1], angleRange3[0], angleRange3[1], valueRange3[0], valueRange3[1]), 0, 180);
  values[3] = map(constrain(angles[3], angleRange4[0], angleRange4[1]), angleRange4[0], angleRange4[1], valueRange4[0], valueRange4[1]);
  
  //initialize servo values
  Servo1.write(values[0]);
  Servo2.write(values[1]);
  Servo3.write(values[2]);
  Servo4.write(values[3]);
}
