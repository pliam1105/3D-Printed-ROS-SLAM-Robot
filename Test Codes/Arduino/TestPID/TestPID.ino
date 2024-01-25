#include <PID_v1.h>

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

PID leftSpeedPID(&leftEncoderSpeed, &leftOutput, &leftMotorSpeed, KP, KI, KD, DIRECT);
PID rightSpeedPID(&rightEncoderSpeed, &rightOutput, &rightMotorSpeed, KP, KI, KD, DIRECT);

void setup() {
  delay(2000);
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

  Serial.begin(115200);
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
  linearX = 0.2;//in m/s
  angularZ = 0;//in rad/s

  // delay(1000);
}

void loop() {
  //compute encoder speed
  currentTime = millis();
  if(currentTime - lastTime >= encoderInterval){
    //compute speeds in pulses/millisecond
    leftEncoderSpeed = ((double)leftCounter)/((double)currentTime - lastTime);
    rightEncoderSpeed = ((double)rightCounter)/((double)currentTime - lastTime);
    //convert to meters/second
    leftEncoderSpeed = leftEncoderSpeed * 1000.0 / metersToPulses;
    rightEncoderSpeed = rightEncoderSpeed * 1000.0 / metersToPulses;
    Serial.print("0, ");
    Serial.print(leftMotorSpeed);
    Serial.print(", ");
    Serial.print(leftEncoderSpeed);
    Serial.print(", ");
    Serial.print(rightMotorSpeed);
    Serial.print(", ");
    Serial.println(rightEncoderSpeed);
    leftCounter = rightCounter = 0;
    lastTime += encoderInterval;
  }

  if(currentTime - startTime >= 5000){
    stop_moving();
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
}

void stop_moving(){
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
