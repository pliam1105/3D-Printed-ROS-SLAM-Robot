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

volatile long leftCounter = 0, rightCounter = 0;
const long interval = 10;
unsigned long lastTime, currentTime;
volatile bool changeLeft = false, changeRight = false;

unsigned long startTime;

void setup() {
  Serial.begin(115200);
  //set pin modes
  pinMode(leftA, INPUT_PULLUP);
  pinMode(leftB, INPUT_PULLUP);
  pinMode(rightA, INPUT_PULLUP);
  pinMode(rightB, INPUT_PULLUP);

  attachInterrupt(0, changeLeftA, RISING);
  attachInterrupt(4, changeRightA, RISING);

  lastTime = millis();
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

  analogWrite(leftRPWM, 0);
  analogWrite(leftLPWM, 150);
  analogWrite(rightRPWM, 0);
  analogWrite(rightLPWM, 150);

  startTime = millis();
}

void loop() {
  unsigned long endTime = millis();
  if(endTime - startTime > 5000){
    analogWrite(leftRPWM, 0);
    analogWrite(leftLPWM, 0);
    analogWrite(rightRPWM, 0);
    analogWrite(rightLPWM, 0);
    //disable movement
    digitalWrite(leftENL, LOW);
    digitalWrite(leftENR, LOW);
    digitalWrite(rightENL, LOW);
    digitalWrite(rightENR, LOW);
  }
  // if(endTime - startTime > 5000){
  //   analogWrite(leftRPWM, 150);
  //   analogWrite(leftLPWM, 0);
  //   analogWrite(rightRPWM, 150);
  //   analogWrite(rightLPWM, 0);
  // }
  currentTime = millis();
  if(currentTime - lastTime >= interval){
    //compute speeds in pulses/millisecond
    float leftSpeed = ((float)leftCounter)/((float)currentTime - lastTime);
    float rightSpeed = ((float)rightCounter)/((float)currentTime - lastTime);
    Serial.print("0, ");
    Serial.print(leftSpeed);
    Serial.print(", ");
    Serial.println(rightSpeed);
    leftCounter = rightCounter = 0;
    lastTime += interval;
  }
  // else if(endTime - startTime > 2000){
  //   //reverse
  //   analogWrite(leftRPWM, 0);
  //   analogWrite(leftLPWM, 63);
  //   analogWrite(rightRPWM, 0);
  //   analogWrite(rightLPWM, 63);
  // }
  delay(10);
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
