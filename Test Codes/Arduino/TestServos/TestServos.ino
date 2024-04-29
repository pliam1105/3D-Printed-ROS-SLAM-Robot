#include <Servo.h>

#define servo1 22
#define servo2 23
#define servo3 24
#define servo4 25

Servo Servo1, Servo2, Servo3, Servo4;

const float valueRange1[] = {30, 80};
const float angleRange1[] = {-20, 30};

const float valueRange2[] = {40, 110};
const float angleRange2[] = {10, 80};

const float valueRange3[] = {90, 160};
const float angleRange3[] = {-50, 20};
const float angleConstrain3[] = {50, 120}; //wrt the previous link, this is where angle3 lies

// const float maxSum23 = 200; //SOS: must be value2 + value3 <= maxSum23

const float valueRange4[] = {50, 0};
const float angleRange4[] = {25, 75};

float value1, value2, value3, value4;
float angle1, angle2, angle3, angle4;

void setup() {
  //initial angles
  angle1 = 0, angle2 = 10, angle3 = 50, angle4 = 25;
  
  //calculate values from angles with map
  value1 = map(constrain(angle1, angleRange1[0], angleRange1[1]), angleRange1[0], angleRange1[1], valueRange1[0], valueRange1[1]);
  value2 = map(constrain(angle2, angleRange2[0], angleRange2[1]), angleRange2[0], angleRange2[1], valueRange2[0], valueRange2[1]);
  value3 = constrain(map(constrain(angle3, angleConstrain3[0], angleConstrain3[1])-90-angle2, angleRange3[0], angleRange3[1], valueRange3[0], valueRange3[1]), 0, 180);
  value4 = map(constrain(angle4, angleRange4[0], angleRange4[1]), angleRange4[0], angleRange4[1], valueRange4[0], valueRange4[1]);
  
  //initialize servo values
  Servo1.write(value1);
  Servo2.write(value2);
  Servo3.write(value3);
  Servo4.write(value4);
  //attach servos
  Servo1.attach(servo1);
  Servo2.attach(servo2);
  Servo3.attach(servo3);
  Servo4.attach(servo4);
}

void loop() {

}
