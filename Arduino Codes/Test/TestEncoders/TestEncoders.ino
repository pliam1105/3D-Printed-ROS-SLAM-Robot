//left encoder
#define leftA 2
#define leftB 3

//right encoder
#define rightA 19
#define rightB 18

volatile long leftCounter = 0, rightCounter = 0;
const long interval = 1000;
unsigned long lastTime, currentTime;
volatile bool changeLeft = false, changeRight = false;

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
}

void loop() {
  // if(changeLeft || changeRight){
  //   Serial.print(leftCounter);
  //   Serial.print(", ");
  //   Serial.println(rightCounter);
  //   changeLeft = changeRight = false;
  // }
  
  currentTime = millis();
  if(currentTime - lastTime >= interval){
    //compute speeds in pulses/millisecond
    float leftSpeed = ((float)leftCounter)/((float)currentTime - lastTime);
    float rightSpeed = ((float)rightCounter)/((float)currentTime - lastTime);
    Serial.print(leftSpeed);
    Serial.print(", ");
    Serial.println(rightSpeed);
    leftCounter = rightCounter = 0;
    lastTime += interval;
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
