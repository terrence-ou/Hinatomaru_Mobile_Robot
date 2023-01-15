#define ENCA 19
#define ENCB 43

#define PWM_PIN 4
#define IN1 51
#define IN2 49

int pos = 0;

void setup() {
  Serial.begin(9600);
  pinMode(ENCA, INPUT);
  pinMode(ENCB, INPUT);
  attachInterrupt(digitalPinToInterrupt(ENCA), readEncoder, RISING);
}

void loop() {
//  setMotor(1, 50, PWM_PIN, IN1, IN2);
//  delay(1000);
//  Serial.println(pos);
//  setMotor(-1, 50, PWM_PIN, IN1, IN2);
//  delay(1000);
//  Serial.println(pos);
//  setMotor(0, 50, PWM_PIN, IN1, IN2);
//  delay(1000);
  Serial.println(pos);
  
}


void readEncoder(){
  int b = digitalRead(ENCB);
  if (b > 0) ++pos;
  else --pos;
}

void setMotor(int dir, int pwmVal, int pwmPin, int in1, int in2){
  analogWrite(pwmPin, pwmVal);
  if (dir == 1){
    digitalWrite(in1, HIGH);
    digitalWrite(in2, LOW);
  } else if (dir == -1){
    digitalWrite(in1, LOW);
    digitalWrite(in2, HIGH);
  } else {
    digitalWrite(in1, LOW);
    digitalWrite(in2, LOW);
  }
}
