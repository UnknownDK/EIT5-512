///////////////////////////////////////////////////////
////////////////////////DC/////////////////////////////
#define Encoder_output_A 5 // pin 2 of the Arduino
#define Encoder_output_B 19 // pin 4 of the Arduino

// Pins til PWM
const int pinPWMR = 16;  // 16 corresponds to GPIO16 men hvad vil vi vælge ???
const int pinPWML = 17;  // blå ledning

// setting PWM properties
const int freq = 10000; // PWM frekvens
const int pwmChannelR = 0; // Ved jeg ikke helt hvad gør
const int pwmChannelL = 1; // Ved jeg ikke helt hvad gør
const int resolution = 13; // bit resolution tror jeg (så 0 - 8k)

double gearRatio = 6.1838;
double angPrStep = 360.0 / (2000.0 * gearRatio);

double Kd = 2.5; //18.2;  // Differential gain
double Kp = 309.6;  // Proportional gain 0.0005, 5, 200
double Ki = 1350.6;  // Integrator gain
double P, I, D = 0;
double samplingPeriod = 12;   // 1/frekvens (enhed er millis)

double angle = 0;         // y(t) (DER VI ER - LÆSES MED ENCODER)
double controlSignal[2] = {0, 0}; // u(t)
double errorSignal[3] = {0, 0, 0};   // e(t) til e(t-2)
//double * setPoint = 0;      // r(t) (DER VI VIL HEN)
double setPoint[3] = {0, 0, 0};

unsigned long DCcurrentTime, someDelay; //previousTime,
//double elapsedTime;
///////////////////////////////////////////////////////
////////////////////////STEPPER////////////////////////
int motorPinA1 = 26; //26 B
int motorPinB1 = 32; //32 C
int motorPinA2 = 33; //33 D
int motorPinB2 = 25; //25 A

unsigned long StepCurrentTime;

int motorPos = 1;
long totalSteps = 0;
///////////////////////////////////////////////////////
////////////////////////ANGLE//////////////////////////
#include <stdio.h>
#include <math.h>
#include <stdlib.h>
double * findPlanes(double alphaA, double alphaB, double alphaC);
///////////////////////////////////////////////////////

void setup() {
  ///////////DC
  // configure LED PWM functionalitites
  ledcSetup(pwmChannelR, freq, resolution);
  ledcSetup(pwmChannelL, freq, resolution);
  pinMode(2, OUTPUT);
  // attach the channel to the GPIO to be controlled
  ledcAttachPin(pinPWMR, pwmChannelR);
  ledcAttachPin(pinPWML, pwmChannelL);
  Serial.begin(115200); // activates the serial communication
  pinMode(Encoder_output_A, INPUT); // sets the Encoder_output_A pin as the input
  pinMode(Encoder_output_B, INPUT); // sets the Encoder_output_B pin as the input
  attachInterrupt(Encoder_output_A, isrA, CHANGE);
  attachInterrupt(Encoder_output_B, isrB, CHANGE);


  /////////////////STEPPER
  pinMode(motorPinA1, OUTPUT);
  pinMode(motorPinB1, OUTPUT);
  pinMode(motorPinA2, OUTPUT);
  pinMode(motorPinB2, OUTPUT);
}

int sekvens[6] = {0, 0, 0, 0, 0, 0};
int nyVinkel = 1;
double t = 0;
unsigned long timerLast = 0;
void loop() {
  /*
    if (nyVinkel == 1) {
    //double *setPoint;
    setPoint = findPlanes(1.0472, 1.22173, 2.44346);
    for (int i = 0; i < 3; i++) {
      setPoint[i] *= (180 / 3.1415926);
      //Serial.println(setPoint[i]);
    }
    //free(setPoint);
    }
  */

  if (millis() > 1000 and sekvens[0] == 0) {
    setPoint[0] = -25.78;
    setPoint[1] = 0;
    setPoint[2] = 0;
    sekvens[0] = 1;
  }

  if (millis() > 3000 and sekvens[1] == 0) {
    t += 0.012;
    setPoint[0] = 2.0 * sin(t) - 25.78;
    setPoint[1] = 2.1 * cos(t);
    setPoint[2] = 0;
  }

  if (millis() > 7000 and sekvens[2] == 0) {
    sekvens[1] = 1;
    setPoint[0] = 0;
    setPoint[1] = -14.78;
    setPoint[2] = 0;
    sekvens[2] = 1;
  }

  if (millis() > 9000 and sekvens[3] == 0) {
    setPoint[0] = -25.78;
    setPoint[1] = -12.6;
    setPoint[2] = 0;
    sekvens[3] = 1;
  }

  if (millis() > 11000 and sekvens[4] == 0) {
    setPoint[0] = 0;
    setPoint[1] = 0;
    setPoint[2] = 0;
    sekvens[4] = 1;
  }

  if (millis() > 14500 and sekvens[5] == 0) {
    t += 0.012;
    setPoint[0] = 8.0 * sin(t);
    setPoint[1] = 8.18 * cos(t);
    setPoint[2] = 0;
  }

  if (millis() > 25000) {
    sekvens[5] = 1;
    setPoint[0] = 0;
    setPoint[1] = 0;
    setPoint[2] = 0;
  }


  /*
    if (millis() > 7500 and sekvens[1] == 0) {
      //setPoint = findPlanes(1.0472, 1.22173, 2.44346);
      setPoint[0] = -25.78;
      setPoint[1] = -12.6;
      setPoint[2] = 0;
      sekvens[1] = 1;
    }

    if (millis() > 12500 and sekvens[2] == 0) {
      //setPoint = findPlanes(1.0472, 1.22173, 2.44346);
      setPoint[0] = 0;
      setPoint[1] = -14.78;
      setPoint[2] = 0;
      sekvens[2] = 1;
    }

    if (millis() > 17500 and sekvens[3] == 0) {
      //setPoint = findPlanes(1.0472, 1.22173, 2.44346);
      setPoint[0] = 0;
      setPoint[1] = 0;
      setPoint[2] = 0;
      sekvens[3] = 1;
    }

    if (millis() > 22500 and (millis() > (timerLast + 10)) and sekvens[4] == 0) {
      timerLast = millis();
      setPoint[0] += 1;
      setPoint[1] = 0;
      setPoint[2] = 0;
    }

    if (millis() > 30000 and sekvens[5] == 0) {
      sekvens[4] = 1;
      setPoint[0] = 0;
      setPoint[1] = 0;
      setPoint[2] = 0;
      sekvens[5] = 1;
    }
  */


  if (millis() > DCcurrentTime + samplingPeriod) {
    DCcurrentTime = millis();
    DCCONTROL();
    Serial.print("DC setpoint"); Serial.println(setPoint[0]);
    Serial.print("DC-vinkel: "); Serial.println(angle);
  }
  if (millis() > StepCurrentTime + 2) {
    StepCurrentTime = millis();
    runSteps(&motorPos, setPoint[1]);
  }
  nyVinkel = 0;
}
