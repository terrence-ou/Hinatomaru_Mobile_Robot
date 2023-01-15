#include <util/atomic.h>

// The number of motors
#define NMOTORS 4

/*
####################################################
#            Classes and Data Structures           #
####################################################
*/

// Motor structure
struct Motor {
   int enca, encb, in1, in2, pwm_pin, pulses;
};


/*
####################################################
#                  Helper Functions                #
####################################################
*/

Motor initMotor(int enca, int encb, int in1, int in2, int pwm_pin, int pulses){
  Motor m;
  m.enca = enca;
  m.encb = encb;
  m.in1 = in1;
  m.in2 = in2;
  m.pwm_pin = pwm_pin,
  m.pulses = pulses;
  return m; 
}



//Set motor rotation
void setMotor(int dir, int pwmVal, int pwmPin, int in1, int in2) {
  analogWrite(pwmPin, pwmVal);
  if (dir == 1) {
    digitalWrite(in1, HIGH);
    digitalWrite(in2, LOW);
  } else if (dir == -1) {
    digitalWrite(in1, LOW);
    digitalWrite(in2, HIGH);
  } else {
    digitalWrite(in1, LOW);
    digitalWrite(in2, LOW);
  }
}


/*
####################################################
#                  Global Variables                #s
####################################################
*/

Motor LF = initMotor(21, 28, 41, 50, 8, 700);
Motor LR = initMotor(20, 29, 39, 48, 9, 700);
Motor RF = initMotor(19, 30, 37, 46, 10, 700);
Motor RR = initMotor(18, 31, 35, 44, 11, 700);


// Pins
const int enca[] = {LF.enca, LR.enca, RF.enca, RR.enca};
const int encb[] = {LF.encb, LR.encb, RF.encb, RR.encb};
const int pwm_pin[] = {LF.pwm_pin, LR.pwm_pin, RF.pwm_pin, RR.pwm_pin};
const int in1[] = {LF.in1, LR.in1, RF.in1, RR.in1};
const int in2[] = {LF.in2, LR.in2, RF.in2, RR.in2};

// Global variables                     
long prevT = 0;
int pulses[] = {LF.pulses, LR.pulses, RF.pulses, RR.pulses};
volatile int posi[] = {0, 0, 0, 0};
int motions[][4] = {{-1, -1, 1, 1}, //Forward
                    {1, 1, -1, -1}, //Backward
                    {-1, 1, -1, 1}, //Left shift
                    {1, -1, 1, -1}, // Right shift
                    {-1, 0, 0, 1}, // Diag forward
                    {0, 1, -1, 0}, // Diag backward
                    {0, 0, -1, -1}, // Around a bend forward
                    {-1, -1, 0, 0}, // Around a bend backward
                    {1, 0, 1, 0}, // Central point rot
                    {1, 1, 1, 1}
                    };


/*
####################################################
#                    Main Program                  #
####################################################
*/


void setup() {
  for(int i = 0; i < NMOTORS; ++i){
    pinMode(enca[i], INPUT);
    pinMode(encb[i], INPUT);
    pinMode(pwm_pin[i], OUTPUT);
    pinMode(in1[i], OUTPUT);
    pinMode(in2[i], OUTPUT);
  }
}


void loop() {
  int num_motions = sizeof(motions) / sizeof(motions[0]);
  //loop through the motors
  for (int i = 0; i < num_motions; ++i){
    for(int j = 0; j < NMOTORS; ++j){
      int dir = motions[i][j];
      setMotor(dir, 250, pwm_pin[j], in1[j], in2[j]);
    }
    delay(1500);
    for(int j = 0; j < NMOTORS; ++j){
      int dir = 0;
      setMotor(dir, 250, pwm_pin[j], in1[j], in2[j]);
    }
    delay(500);
  }
}
