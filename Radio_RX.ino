#include <SPI.h>
#include <NRFLite.h>
#include <util/atomic.h>

//===== Walker Settings =====//
byte instrucQueue[2] = {4, 4};
bool modeSelection[2] = {0 , 0};

//===== Radio =====//
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

//===== Encoder Pins & Variables =====//
const byte ENCODER_PIN[2] = {2, 3};

volatile byte encoderCount[2];
byte prevEncoderCount;
byte deltaCount;

//===== Motor Pins & Variables =====//
const byte MOTOR_PIN[2][2] = { {4, 5}, {7, 6} }; // {DIR, EN}
const byte MOTOR_DIR[2][4] = { {1, 1, 0, 0}, {1, 0, 0, 1} };

volatile unsigned long pulseWidth;

bool changeLegPhase = false;
bool instrucComplete;

//===== PID Variables =====//
const float k_p = 5;
const float k_i = 27.5;

float PID_Integral;
float prevError;
unsigned long prevIntegralT;
const unsigned long integralInterval = 2.0e5;

//===== Timer Variables =====//
unsigned long currT; // Timer for entire program

unsigned long prevPulseT; // Used for pulse width caluclation

unsigned long prevDeltaT; // Timing variable to measure specified sampling period
const long deltaT = 1.5e5; // Sampling period for change in count value

//===== Main Program =====//
void setup() {
  Serial.begin(115200);
  while (!Serial) {}

  if (!_radio.init(RADIO_RX_ID, RADIO_PIN_CE, RADIO_PIN_CSN)) {
    Serial.println("Radio not responding!");
    Serial.println("Check connections and press RESET on Arduino!");
    while (1);
  }

  for (byte i = 0; i < 2; i++) {
    // Motors
    pinMode(MOTOR_PIN[i][0], OUTPUT);
    pinMode(MOTOR_PIN[i][1], OUTPUT);

    //Encoders
    pinMode(ENCODER_PIN[i], INPUT);
  }

  attachInterrupt(digitalPinToInterrupt(ENCODER_PIN[0]), ENCODER_LEFT_ISR, RISING);
  attachInterrupt(digitalPinToInterrupt(ENCODER_PIN[1]), ENCODER_RIGHT_ISR, RISING);
}

void loop() {
  currT = micros();

  readData();
  float MOTOR_SPEED = measureSpeed();

  if (modeSelection[0] != modeSelection[1]) {
    changeMode();
  }

  if (instrucQueue[1] == 4) { // Walker is idle and ready for next command
    instrucQueue[1] = instrucQueue[0];
    instrucComplete = false;
  }
  else if (instrucQueue[1] < 4) { // Walker is currently busy executing command
    PID_MOTOR_CONTROL(28, MOTOR_SPEED);
  }

  if (instrucComplete == true && MOTOR_SPEED == 0) {
    instrucQueue[1] = 4;
    Serial.println("Instruc complete!");

    // Reset variables
    encoderCount[0] = 0;
    PID_Integral = 0;
    prevError = 0;
  }

  Serial.print(modeSelection[0] != modeSelection[1]);
  Serial.print(" ");
  Serial.print(instrucQueue[0]);
  Serial.print(" ");
  Serial.print(instrucQueue[1]);
  Serial.print(" ");
  Serial.print(MOTOR_SPEED);
  Serial.print(" ");
  Serial.print(instrucComplete);
  Serial.print(" ");
  Serial.println(encoderCount[0]);
}

void changeMode() {
  instrucComplete = false;
  instrucQueue[1] = 5; // Flag walker is not idle

  if (encoderCount[0] >= 92) {
    analogWrite(MOTOR_PIN[0][1], 0);

    modeSelection[1] = modeSelection[0];
    instrucComplete = true;
  }
  else {
    digitalWrite(MOTOR_PIN[0][0], 1);
    analogWrite(MOTOR_PIN[0][1], 180);
  }
}

void readData() {
  while (_radio.hasData()) {
    _radio.readData(&_radioData);

    instrucQueue[0] = _radioData.WalkerDir;
    modeSelection[0] = _radioData.WalkerMode;
  }
}

void ENCODER_LEFT_ISR() {
  encoderCount[0] += 1;

  pulseWidth = (currT - prevPulseT);
  prevPulseT = currT;
}

void ENCODER_RIGHT_ISR() {
  encoderCount[1] += 1;
}

float measureSpeed() {
  unsigned long currPulseWidth;
  ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
    currPulseWidth = pulseWidth;
  }

  if ( (currT - prevDeltaT) >= deltaT) {
    deltaCount = encoderCount[0] - prevEncoderCount;
    prevEncoderCount = encoderCount[0];
    prevDeltaT = currT;
  }

  float countsPerSecond;
  if (deltaCount == 0) {
    countsPerSecond = 0;
  }
  else {
    countsPerSecond = ((float) 1 / currPulseWidth) * 1.0e6;
  }

  return countsPerSecond;
}

void PID_MOTOR_CONTROL(byte targetSpeed, float motorSpeed) {
  float PID_Error = ((float) targetSpeed - motorSpeed);

  if ( (currT - prevIntegralT) >= integralInterval) {
    PID_Integral += (PID_Error + prevError) * 0.5 * integralInterval * 1.0e-6;
    prevIntegralT = currT;
    prevError = PID_Error;
  }

  float controlSignal = k_p * PID_Error + k_i * PID_Integral;

  // If controlSingal is negative, we need to slow down motor => Run motor in opposite direction
  /*
    if (controlSignal < 0) {
      if ( (instrucQueue[1] - 1.5) < 0) {
        instrucQueue[1] += 2;
      }
      else {
        instrucQueue[1] -= 2;
      }
    }
  */

  int motorPWM = (int) fabs(controlSignal);
  if (motorPWM > 255) {
    motorPWM = 255;
  }

  // Insert logic for detecting completion of current instruction
  if (encoderCount[0] >= 189) {
    motorPWM = 0;
    instrucComplete = true;
  }

  digitalWrite(MOTOR_PIN[0][0], MOTOR_DIR[0][instrucQueue[1]]);
  digitalWrite(MOTOR_PIN[1][0], MOTOR_DIR[1][instrucQueue[1]]);

  analogWrite(MOTOR_PIN[0][1], motorPWM);
  analogWrite(MOTOR_PIN[1][1], motorPWM);
}
