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

unsigned long startTime;

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

  //forward
  analogWrite(leftRPWM, 63);
  analogWrite(leftLPWM, 0);
  analogWrite(rightRPWM, 63);
  analogWrite(rightLPWM, 0);

  startTime = millis();
}

void loop() {
  //put both motors in straight rotation for 2 seconds
  unsigned long endTime = millis();
  if(endTime - startTime > 4000){
    analogWrite(leftRPWM, 0);
    analogWrite(leftLPWM, 0);
    analogWrite(rightRPWM, 0);
    analogWrite(rightLPWM, 0);
    //disable movement
    digitalWrite(leftENL, LOW);
    digitalWrite(leftENR, LOW);
    digitalWrite(rightENL, LOW);
    digitalWrite(rightENR, LOW);
  }else if(endTime - startTime > 2000){
    //reverse
    analogWrite(leftRPWM, 0);
    analogWrite(leftLPWM, 63);
    analogWrite(rightRPWM, 0);
    analogWrite(rightLPWM, 63);
  }
  delay(100);
}
