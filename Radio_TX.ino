#include <SPI.h>
#include <NRFLite.h>
#include <LiquidCrystal_I2C.h>

//===== LCD =====//
LiquidCrystal_I2C lcd(0x27,16,2);

//===== Radio =====//
const static byte RADIO_TX_ID = 0;
const static byte RADIO_RX_ID = 1;
const static byte RADIO_PIN_CE = 9;
const static byte RADIO_PIN_CSN = 10;

struct DataPacket
{
  byte WalkerDir;
  byte WalkerSpd;
  bool WalkerMode;
};

NRFLite _radio;
DataPacket _radioData;

//===== Joystick =====//
const static byte JOYSTICK_PIN[2][3] = {{6, 7, 3}, {3, 2, 5}};  // The numbers correspond to {VRx, VRy, SW}; Index 0 = Left Joystick

char dirLegend[5] = {'F', 'R', 'B', 'L', 'I'};
int idlePos[2][2] = {{0,0} , {0,0}};
int currPos[2][2] = {{0,0}, {0,0}};

byte directionIndex;
bool changeMode = false;

//===== Potentiometer =====//
const static byte POT_PIN[2] = {0, 1}; // Index 0 = Left Potentiometer

float smoothingConst = 0.15;
int rawPotValue[2];
int smoothPotValue[2];

int speedSetting;

//===== Timer Variables =====//
unsigned long currT;
unsigned long prevTX;
unsigned long btnTimeStamp = 0;
unsigned long txInterval = 5.0e5;
unsigned long sampleInterval = 1.0e4;

//===== Main Program =====//
void setup() {
  Serial.begin(115200);
  while (!Serial) {}

  lcd.init();
  lcd.backlight();
  lcd.setCursor(1,0);
  lcd.print("MechWalker V.3");
  lcd.setCursor(1,1);
  lcd.print("Initializing..");

  delay(1000);

  if (!_radio.init(RADIO_TX_ID, RADIO_PIN_CE, RADIO_PIN_CSN)) {
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("Check radio!");
    lcd.setCursor(0,1);
    lcd.print("Press RESET!");
    while (1);
  }
  
  for (byte i = 0; i < 2; i++) {
    idlePos[0][i] = map(analogRead(JOYSTICK_PIN[0][i]), 0, 1023, 0, 255);
    idlePos[1][i] = map(analogRead(JOYSTICK_PIN[1][i]), 0, 1023, 0, 255);
  }

  pinMode(JOYSTICK_PIN[0][2], INPUT_PULLUP);
  pinMode(JOYSTICK_PIN[0][2], INPUT_PULLUP);

  lcd.clear();
}

void loop() {
  currT = micros();
 
  readJoystick(0);
  readPotentiometer(1);

  speedSetting = map(smoothPotValue[1], 0, 805, 0 ,100);

  if (!digitalRead(JOYSTICK_PIN[0][2])) {
    if (btnTimeStamp == 0) {
      btnTimeStamp = currT;
    }
    else if (currT - btnTimeStamp > 1.5e5) { 
      changeMode = !changeMode;
      btnTimeStamp = 0; 
    } 
  }

  if (currT - prevTX >= txInterval) {
    sendData();
    prevTX = currT;
  }

  drawMainScreen(); 
}

//===== Functions =====//
void drawMainScreen() {
  lcd.setCursor(3,0);
  lcd.print("Mode:");
  lcd.setCursor(8,0);
  if (changeMode) {
    lcd.print("Steer");
  }
  else {
    lcd.print("Drive");
  }
  
  lcd.setCursor(1,1);
  lcd.print("SPD:");
  
  lcd.setCursor(5,1);
  if (speedSetting / 10 == 0) {
    lcd.print("0" + String(speedSetting));
  }
  else {
    lcd.print(speedSetting);
  }
  
  lcd.setCursor(7,1);
  lcd.print("%  DIR:");
  
  lcd.setCursor(14,1);
  lcd.print(dirLegend[directionIndex]);
}


void readJoystick(byte joystickIndex) {
  for (byte i = 0; i < 2; i++) {
    currPos[joystickIndex][i] = map(analogRead(JOYSTICK_PIN[joystickIndex][i]), 0, 1023, 0, 255);
  }
  
  /*Each direction has a corresponding number:
    Up - 0
    Right - 1
    Down - 2
    Left - 3
    Idle - 4
  */

  //Idle
  if (abs(currPos[joystickIndex][0] - idlePos[joystickIndex][0]) < 30 && abs(currPos[joystickIndex][1] - idlePos[joystickIndex][1]) < 30) {
    directionIndex = 4;
  }

  //Left & Right Movement
  if (currPos[joystickIndex][0] < 25 && abs(currPos[joystickIndex][1] - idlePos[joystickIndex][1]) < 30) {
    directionIndex = 2 + pow(-1,joystickIndex+1);
  }
  
  if (currPos[joystickIndex][0] > 180 && abs(currPos[joystickIndex][1] - idlePos[joystickIndex][1]) < 30) {
    directionIndex = 2 + pow(-1,joystickIndex);
  }

  //Backward & Forward
  if (currPos[joystickIndex][1] > 180 && abs(currPos[joystickIndex][0] - idlePos[joystickIndex][0]) < 30) {
    directionIndex = 1 + pow(-1, joystickIndex+1);
  }
  
  if (currPos[joystickIndex][1] < 55 && abs(currPos[joystickIndex][0] - idlePos[joystickIndex][0]) < 30) {
    directionIndex = 1 + pow(-1, joystickIndex);
  }
}

void readPotentiometer(byte potIndex) {
  rawPotValue[potIndex] = analogRead(1);
  smoothPotValue[potIndex] = smoothingConst * rawPotValue[potIndex] + (1 - smoothingConst) * smoothPotValue[potIndex];
}


void sendData() {
  _radioData.WalkerDir = directionIndex;
  _radioData.WalkerSpd = speedSetting;
  _radioData.WalkerMode = changeMode;

  Serial.println("Sending data...");
  if (_radio.send(RADIO_RX_ID, &_radioData, sizeof(_radioData))) {
    Serial.println("... Success!");
  }
  else {
    Serial.println("... Failed!");
  }
}
