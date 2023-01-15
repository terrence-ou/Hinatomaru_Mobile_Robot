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


// PID Control Class
class PIDControl {
  private:
    float kp, ki, kd, umax; // Parameters
    float eprev, eintegral; // Storage

  public:
    //Constructor
    PIDControl() : kp(1), ki(0), kd(0), umax(255), eprev(0.0), eintegral(0.0) {};

    // Set PID control parameters
    void setParams(float kpIn, float kdIn, float kiIn, float umaxIn) {
      kp = kpIn; kd = kdIn; ki = kiIn; umax = umaxIn;
    }

    // Compute control signal
    void evalu(int value, int target, float deltaT, int &pwr, int &dir) {
      float e = target - value;                         // compute error
      float dedt = (e - eprev) / deltaT;                // compute derivative
      eintegral = eintegral + e * deltaT;               // compute integral
      float u = kp * e + ki * eintegral + kd * dedt;    // get control signal

      // get motor power
      pwr = (int) fabs(u);
      if (pwr > umax) pwr = umax;

      // get motor direction
      dir = 1;
      if (u < 0) dir = -1;

      // update previous error
      eprev = e;
    }

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


// Setting motor targets
void setTarget(float t, float deltat, float *target_f, long *target, int pulses[]){
  float positionChange[4] = {0.0, 0.0, 0.0, 0.0};
  
//  float pulsesPerTurn = 220.0; //This value is for hinata's motor
//  float tire_circum = 3.14 * 0.08;

//  float pulsesPerMeter = pulsesPerTurn * (1 / tire_circum);

  float velocity = 1.0; //One rotation persecond

  for(int i = 0; i < NMOTORS; i++){
    positionChange[i] = velocity * deltat * pulses[i];
  }
  
  
  for(int i = 0; i < NMOTORS; i++){
    target_f[i] = target_f[i] + positionChange[i];
  }

  target[0] = -(long) target_f[0];
  target[1] = -(long) target_f[1];
  target[2] = (long) target_f[2];
  target[3] = (long) target_f[3];
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

Motor LF = initMotor(21, 43, 36, 38, 8, 700);
Motor LR = initMotor(18, 34, 37, 39, 9, 700);
Motor RF = initMotor(3, 53, 51, 49, 10, 700);
Motor RR = initMotor(19, 24, 46, 48, 11, 700);


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



//PID controller instances
PIDControl pid[NMOTORS];


//Targets
float target_f[] = {0.0, 0.0, 0.0, 0.0};
long target[] = {0, 0, 0, 0};


//Function with template input for encoder
template <int j>
void readEncoder() {
  int b = digitalRead(encb[j]);
  if (b > 0) posi[j]++;
  else posi[j]--;
}

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

  //Setting up PID parameters for motors
  pid[0].setParams(1.0, 0, 0, 255);
  pid[1].setParams(1.0, 0, 0, 255);
  pid[2].setParams(1.0, 0, 0, 255);
  pid[3].setParams(1.0, 0, 0, 255);

  //Setting up encoder readings
  attachInterrupt(digitalPinToInterrupt(enca[0]), readEncoder<0>, RISING);
  attachInterrupt(digitalPinToInterrupt(enca[1]), readEncoder<1>, RISING);
  attachInterrupt(digitalPinToInterrupt(enca[2]), readEncoder<2>, RISING);
  attachInterrupt(digitalPinToInterrupt(enca[3]), readEncoder<3>, RISING);
}


void loop() {
  //time difference
  long currT = micros();
  float deltaT = ((float) (currT - prevT) / (1.0e6));
  prevT = currT;

  // Set targets
  setTarget(currT/1.0e6, deltaT, target_f, target, pulses);

  //read position in an atomic block to avoid a potential misread
  int pos[NMOTORS];
  ATOMIC_BLOCK(ATOMIC_RESTORESTATE){
    for (int i = 0; i < NMOTORS; ++i){
      pos[i] = posi[i];
    }
  }

  //loop through the motors
  for(int i = 0; i < NMOTORS; ++i){
    int pwr, dir;
//    pid[i].evalu(pos[i], target[i], deltaT, pwr, dir);
    if (i <= 1) dir = -1;
    else dir = 1;
    setMotor(dir, 200, pwm_pin[i], in1[i], in2[i]);
  }
}
