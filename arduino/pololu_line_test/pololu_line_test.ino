#include <PololuMenu.h>
#include <Pololu3piPlus32U4.h>

using namespace Pololu3piPlus32U4;

// Device modules
OLED display;
Buzzer buzzer;
LineSensors lineSensors;
Motors motors;
ButtonA buttonA;
ButtonB buttonB;
ButtonC buttonC;

// Recording Errors
int16_t lastError = 0;

// Array for line-following sensors' readings
#define NUM_SENSORS 5
unsigned int lineSensorValues[NUM_SENSORS];

// Speed controller
uint16_t maxSpeed;
int16_t minSpeed;

uint16_t baseSpeed;
uint16_t calibrationSpeed;


// PID variables
uint16_t proportional;  // coefficient of the P term * 256
uint16_t derivative;    // coefficient of the D term * 256


void slowSpeed() {
  maxSpeed = 60;
  minSpeed = 0;
  baseSpeed = maxSpeed;
  calibrationSpeed = 50;
  proportional = 64;  // P term = 1/4
  derivative = 256;   // D term = 1
}

void mediumSpeed() {
  maxSpeed = 150;
  minSpeed = 0;
  baseSpeed = maxSpeed;
  calibrationSpeed = 50;
  proportional = 64;  // P term = 1/4
  derivative = 256;   // D term = 1
}

void fastSpeed() {
  maxSpeed = 250;
  minSpeed = 0;
  baseSpeed = maxSpeed;
  calibrationSpeed = 50;
  proportional = 64;  // P term = 1/4
  derivative = 256;   // D term = 1
}


// OLED menu
PololuMenu<typeof(display)> menu;

void selectEdition() {
  display.clear();
  display.print(F("Select"));
  display.gotoXY(0, 1);
  display.print(F("edition"));
  delay(1000);

  static const PololuMenuItem items[] = {
    { F("Slow"), slowSpeed },
    { F("Medium"), mediumSpeed },
    { F("Fast"), fastSpeed },
  };

  menu.setItems(items, 3);
  menu.setDisplay(display);
  menu.setBuzzer(buzzer);
  menu.setButtons(buttonA, buttonB, buttonC);

  while (!menu.select());

  display.gotoXY(0, 1);
  display.print("OK! ...");
}

// Setup special characters in the LCD to display the bar graphs
void loadCustomCharacters() {
  static const char levels[] PROGMEM = {
    0, 0, 0, 0, 0, 0, 0, 63, 63, 63, 63, 63, 63, 63
  };

  display.loadCustomCharacter(levels + 0, 0);  // 1 bar
  display.loadCustomCharacter(levels + 1, 1);  // 2 bars
  display.loadCustomCharacter(levels + 2, 2);  // 3 bars
  display.loadCustomCharacter(levels + 3, 3);  // 4 bars
  display.loadCustomCharacter(levels + 4, 4);  // 5 bars
  display.loadCustomCharacter(levels + 5, 5);  // 6 bars
  display.loadCustomCharacter(levels + 6, 6);  // 7 bars
}

void printBar(uint8_t height) {
  if (height > 8) { height = 8; }
  const char barChars[] = { ' ', 0, 1, 2, 3, 4, 5, 6, (char)255 };
  display.print(barChars[height]);
}


// Calibrating line following sensors
void calibrateSensors(){
  display.clear();
  // wait 1 second then begin automatic sensor calibration
  // by rotating in place to sweep the sensors over the line
  delay(1000);
  for(uint16_t i = 0; i < 80; i++){
    if (i > 20 && i <= 60) motors.setSpeeds(-(int16_t)calibrationSpeed, calibrationSpeed);
    else motors.setSpeeds(calibrationSpeed, -(int16_t)calibrationSpeed);

    lineSensors.calibrate();
  }
  motors.setSpeeds(0, 0);
}

// Displays the estimated line position and a bar graph of sensor
// reading on the OLED. Returns after the user press b.
void showReadings(){
  display.clear();
  while(!buttonB.getSingleDebouncedPress()){
    uint16_t position = lineSensors.readLineBlack(lineSensorValues);

    display.gotoXY(0, 0);
    display.print(position);
    display.print("    ");
    display.gotoXY(0, 1);
    for (uint8_t i = 0; i < NUM_SENSORS; i++){
      uint8_t barHeight = map(lineSensorValues[i], 0, 1000, 0, 8);
      printBar(barHeight);
      // Serial.print(lineSensorValues[i]);
      // Serial.print(',');
    }
    // Serial.println();
    // Serial.print(checkTerminate());
    // Serial.println();
    delay(50);
  }
}

void showReadingsOnMove(uint16_t position){
    display.gotoXY(0, 0);
    display.print(position);
    display.print("    ");
    display.gotoXY(0, 1);
    for (uint8_t i = 0; i < NUM_SENSORS; i++){
      uint8_t barHeight = map(lineSensorValues[i], 0, 1000, 0, 8);
      printBar(barHeight);
    }
}

//Check terminate condition
bool checkTerminate(){
  bool allBlack = true;
  for(int i = 1; i < NUM_SENSORS; i++){
    if(lineSensorValues[i] < 600) allBlack = false;
  }
  return allBlack;
}

void setup() {
  Serial.begin(9600);
  // put your setup code here, to run once:
  loadCustomCharacters();
  selectEdition();

  display.clear();
  display.print(F("Press B"));
  display.gotoXY(0, 1);
  display.print(F("to calib"));

  while(!buttonB.getSingleDebouncedPress());

  calibrateSensors();
  showReadings();

  display.clear();
  display.print(F("Go!"));
}


void loop() {
  // put your main code here, to run repeatedly:
  int16_t position = lineSensors.readLineBlack(lineSensorValues);

  if(checkTerminate()){
    motors.setSpeeds(0, 0);
    return;
  }

  showReadingsOnMove((uint16_t)position);

  int16_t error = position - 2000;

  int16_t speedDifference = error * (int32_t)proportional / 256 + (error - lastError) * (int32_t)derivative / 256;

  lastError = error;

  int16_t leftSpeed = (int16_t)baseSpeed + speedDifference;
  int16_t rightSpeed = (int16_t)baseSpeed - speedDifference;

  leftSpeed = constrain(leftSpeed, minSpeed, (int16_t)maxSpeed);
  rightSpeed = constrain(rightSpeed, minSpeed, (int16_t)maxSpeed);
  // Serial.println(leftSpeed);
  // Serial.println(rightSpeed);
  motors.setSpeeds(leftSpeed, rightSpeed);

}
