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
// const double wheelDistance = 0.25; //for angular velocity computation

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
// double linearX, angularZ;
int stop = 1;

PID leftSpeedPID(&leftEncoderSpeed, &leftOutput, &leftMotorSpeed, KP, KI, KD, DIRECT);
PID rightSpeedPID(&rightEncoderSpeed, &rightOutput, &rightMotorSpeed, KP, KI, KD, DIRECT);

//Serial communication
SerialTransfer ros_transfer;

struct __attribute__((packed)) data_struct{
  float vel_left, vel_right;
  int32_t stop;
  float angle1, angle2, angle3, angle4;
};

uint16_t packet_size = 0;

void setup() {
  //initialize servos
  float initialAngles[4] = {0, 10, 50, 25};
  moveServos(initialAngles[0], initialAngles[1], initialAngles[2], initialAngles[3]);

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
    packet_size = 0;
    data_struct new_cmd = data_struct();
    packet_size = ros_transfer.rxObj(new_cmd, packet_size);

    //take action
    stop = new_cmd.stop;
    // linearX = new_cmd.linear_x;
    // angularZ = new_cmd.angular_z;
    leftMotorSpeed = new_cmd.vel_left;
    rightMotorSpeed = new_cmd.vel_right;

    data_struct real_data = new_cmd;//to copy & constrain servo values
    moveServos(real_data.angle1, real_data.angle2, real_data.angle3, real_data.angle4);

    //send encoder speeds
    real_data.vel_left = leftEncoderSpeed;
    real_data.vel_right = rightEncoderSpeed;

    packet_size = 0;
    packet_size = ros_transfer.txObj(real_data, packet_size);
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
  // leftMotorSpeed = linearX - angularZ * wheelDistance / 2.0;
  // rightMotorSpeed = linearX + angularZ * wheelDistance / 2.0;
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

void moveServos(float &angle1, float &angle2, float &angle3, float &angle4){
  //constraint angles
  angle1 = constrain(angle1, angleRange1[0], angleRange1[1]);
  angle2 = constrain(angle2, angleRange2[0], angleRange2[1]);
  angle3 = constrain(angle3, angleConstrain3[0], angleConstrain3[1]);
  angle4 = constrain(angle4, angleRange4[0], angleRange4[1]);

  //calculate values from angles with map
  float values[4];
  values[0] = map(angle1, angleRange1[0], angleRange1[1], valueRange1[0], valueRange1[1]);
  values[1] = map(angle2, angleRange2[0], angleRange2[1], valueRange2[0], valueRange2[1]);
  values[2] = constrain(map(angle3-90-angle2, angleRange3[0], angleRange3[1], valueRange3[0], valueRange3[1]), 0, 180);
  values[3] = map(angle4, angleRange4[0], angleRange4[1], valueRange4[0], valueRange4[1]);
  
  //apply servo values
  Servo1.write(values[0]);
  Servo2.write(values[1]);
  Servo3.write(values[2]);
  Servo4.write(values[3]);
}
