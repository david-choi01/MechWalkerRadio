#include <util/atomic.h>

//===== Encoder Pins & Variables =====//
const byte encoderPin = 3;

volatile int encoderCount; // Ticks up every time photo-interrupter triggers
int prevEncoderCount; // Used for calculating change in count
unsigned int deltaCount; // Change in (encoderCount) value over (deltaT) microseconds

//===== Motor Pins & Variables =====//
const byte DIR1 = 4;
const byte EN1 = 5;

const byte DIR2 = 7;
const byte EN2 = 6;

volatile unsigned long pulseWidth; // Stores value of encoder pulse width calculated in interrupt

byte motorTestIndex = 1; // Used for dealing with PWM value overflow for linear PWM generator
byte motorCommand = 1;

//===== PID Variables =====//
const float k_p = 6;
const float k_i = 0;

float targetSpeed_Smoothed;
float targetSpeed_Prev;

float PID_Integral;
float prevError;
unsigned long integralInterval = 2.5e5;
unsigned long prevIntegralT;

//===== Timer Variables =====//
unsigned long currT; // Timer for entire program

unsigned long prevPulseT; // Used for pulse width caluclation

unsigned long prevIndexTimer; // Timing variable used for linear PWM generator

const long deltaT = 2.0e5; // Sampling period for change in count value
unsigned long prevDeltaT; // Timing variable to measure specified sampling period

void setup() {
  Serial.begin(115200);
  
  pinMode(encoderPin, INPUT);
  attachInterrupt(digitalPinToInterrupt(encoderPin), encoderPulse, RISING);

  pinMode(DIR1, OUTPUT);
  pinMode(EN1, OUTPUT);

  pinMode(DIR2, OUTPUT);
  pinMode(EN2, OUTPUT);
}

void loop() {
  currT = micros();

  float motorSpeed = measureRPM();

  digitalWrite(DIR2, HIGH);
  analogWrite(EN2, 245);

  /*float PID_Error;
  if (motorCommand == 1) {
    unsigned int currCount;
    ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
      currCount = encoderCount;
    }

    byte targetSpeed = 20;
    targetSpeed_Smoothed = (targetSpeed * 0.01) + (targetSpeed_Prev * 0.99);
    targetSpeed_Prev = targetSpeed_Smoothed; 

    PID_Error = targetSpeed_Smoothed - motorSpeed;

    if ((currT - prevIntegralT) >= integralInterval) {
      PID_Integral += (PID_Error + prevError) * 0.5 * integralInterval * 1.0e-6;
      prevIntegralT = currT;
      prevError = PID_Error;
    }
  
    float controlSignal = k_p * PID_Error + k_i * PID_Integral;
  
    digitalWrite(DIR2, HIGH);
    if (controlSignal < 0) {
      digitalWrite(DIR2, LOW);
    }
  
    int motorPWM = (int) fabs(controlSignal);
    if (motorPWM > 255) {
      motorPWM = 255;
    }

    if (currCount >= 188) {
      targetSpeed = 0;
      motorPWM = 0;
    }
  
    analogWrite(EN2, motorPWM);

    if (targetSpeed == 0 && deltaCount == 0) {
      encoderCount = 0;
      motorCommand = 0;
    }
  }
  */

  Serial.print(encoderCount);
  Serial.print(" ");
  Serial.print(prevEncoderCount);
  Serial.print(" ");
  Serial.print(motorSpeed);
  Serial.println();

}

void encoderPulse() {
  // Interrupt function for photo-interrupter. Track how many times interrupt triggers and measure period between triggers
  encoderCount++;
  
  pulseWidth = (currT - prevPulseT);
  prevPulseT = currT;
}

void PWM_Generator(int PWM_Value, int testTime, String genMode) {
  // Generates PWM values for motor testing
  digitalWrite(DIR1, HIGH);
  int motorPWM;
  
  if (genMode == "linear") {
    // Ramps up the PWM from given inital value to 255 over specified time period
    int basePWM = PWM_Value;
    int testInterval = testTime; // Time interval in seconds
  
    // testIndex variable is used to make sure the initial PWM value is maintained every time the test cycles
    if (currT - prevIndexTimer >= (testTime * 1.0e6)) {
      motorTestIndex++;
      prevIndexTimer = currT;
    }
  
    motorPWM = basePWM * motorTestIndex + ((255 - basePWM) * (micros() / 1.0e6)) / testInterval;
  
    analogWrite(EN1, motorPWM);
  }
  else if (genMode == "sine") {
    // Creates a square wave of specified amplitude; the testTime argument is not used here
    motorPWM = PWM_Value*(sin((currT*1.0e-6))>0);
    analogWrite(EN1, motorPWM);
  }
}

float measureRPM() {
  // Takes data from encoder to measure the RPM of the motor. 
  
  // Copy value of pulse width from interrupt function
  int currCount;
  unsigned long currPulseWidth;
  ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
    currCount = encoderCount;
    currPulseWidth = pulseWidth;
  }

  // Calculate change in pulse width every (deltaInterval) microseconds
  // unsigned int deltaCount;
  if ((currT - prevDeltaT) >= deltaT) {
    deltaCount = (currCount - prevEncoderCount);
    prevEncoderCount = currCount;
    prevDeltaT = currT;
  }

  // Detecting 0 RPM - the minimum detectable RPM is defined by the value of the sampling period 
  // for measuring change in the value of encoderCount.
  float countsPerSecond;
  if (deltaCount == 0) {
    countsPerSecond = 0;
  }
  else {
    countsPerSecond = ((float) 1 / currPulseWidth) * 1.0e6;
  }

  return countsPerSecond;
}
