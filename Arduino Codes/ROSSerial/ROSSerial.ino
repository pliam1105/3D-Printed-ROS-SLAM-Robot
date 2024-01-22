#include <PID_v1.h>
#include "SerialTransfer.h"

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

void setup() {
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

  // //motor speeds
  // leftMotorSpeed = 0.2;
  // rightMotorSpeed = 0.2;

  //ROS Twist components
  // linearX = 0.2;//in m/s
  // angularZ = 0.78;//in rad/s
}

void loop() {
  uint16_t packet_size = 0;
  
  //receiving commands
  if(ros_transfer.available()){
    //receive packet
    packet_size = 0;
    vel_struct new_vel = vel_struct();
    packet_size = ros_transfer.rxObj(new_vel, packet_size);

    //take action
    stop = new_vel.stop;
    linearX = new_vel.linear_x;
    angularZ = new_vel.angular_z;

    //send response
    packet_size = 0;
    packet_size = ros_transfer.txObj("OK", packet_size);
    ros_transfer.sendData(packet_size);
  }

  //compute and send encoder speed
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

    //sending encoder speeds
    real_speed_struct real_speed = real_speed_struct();
    real_speed.speed_left = leftEncoderSpeed;
    real_speed.speed_right = rightEncoderSpeed;

    packet_size = 0;
    packet_size = ros_transfer.txObj(real_speed.speed_left, packet_size);
    packet_size = ros_transfer.txObj(real_speed.speed_right, packet_size);
    ros_transfer.sendData(packet_size);
    //wait for response
    while(!ros_transfer.available()){
      //wait
    }
    packet_size = 0;
    char ok[2];
    packet_size = ros_transfer.rxObj(ok, packet_size, 2);
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
