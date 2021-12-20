//////////////////////////////////////DCvariabler
#define Encoder_output_A 5 // pin 2 of the Arduino
#define Encoder_output_B 19 // pin 4 of the Arduino
//#define Encoder_output_I 18 // pin 3 of the Arduino

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
double *setPoint;      // r(t) (DER VI VIL HEN)

unsigned long currentTime, previousTime, someDelay;
double elapsedTime;

////////////////////////////////Steppervariabler
int motorPinA1 = 26; //26 B
int motorPinB1 = 32; //32 C
int motorPinA2 = 33; //33 D
int motorPinB2 = 25; //25 A

int motorPos = 1;
long totalSteps = 0;

///////////////////////////////Vinkelberegner
#include <stdio.h>
#include <math.h>
#include <stdlib.h>
double * findPlanes(double alphaA, double alphaB, double alphaC);


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
  
  ///////////////////VINKELBEREGNER
  
}

int nyVinkel = 1;
void loop() {
  if (nyVinkel == 1) {
    //double *setPoint;
    setPoint = findPlanes(1.0472, 1.22173, 2.44346);
    for (int i = 0; i < 3; i++) {
      setPoint[i] *= (180 / 3.1415926);
      Serial.println(setPoint[i]);
    }
    //free(setPoint);
  }

  if (millis() > currentTime - samplingPeriod) {
    DCCONTROL();
    Serial.print("DC-vinkel: "); Serial.println(angle);
  }
  runSteps(&motorPos, setPoint[1]);
  nyVinkel=0;
}


/////////////////////////////////FUNKTIONER
//DCkontrol

void DCCONTROL() {
  currentTime = millis();
  elapsedTime = (double)(currentTime - previousTime);

  errorSignal[0] = setPoint[0] - angle;

  // Controllerdelen
  P = Kp * (errorSignal[0] - errorSignal[1]);
  I = (samplingPeriod / 1000) * Ki * errorSignal[0];
  D = (Kd / (samplingPeriod / 1000)) * (errorSignal[0] - 2 * errorSignal[1] + errorSignal[2]);
  controlSignal[0] = 8.191 * (P + I + D) + controlSignal[1]; // burde ganges med 81.91, men det bliver for meget
  if (controlSignal[0] > 8191) {
    controlSignal[0] = 8191;
  } else if (controlSignal[0] < -8191) {
    controlSignal[0] = -8191;
  }
  controlSignal[1] = controlSignal[0];

  if (controlSignal[0] > 0) {
    ledcWrite(pwmChannelR, 0); //4095/controlsignal[0]
    ledcWrite(pwmChannelL, abs(controlSignal[0]));
  } else {
    ledcWrite(pwmChannelR, abs(controlSignal[0]));
    ledcWrite(pwmChannelL, 0);
  }

  previousTime = currentTime;
  //setPoint += 0.011;
  someDelay = millis();
  errorSignal[2] = errorSignal[1];
  errorSignal[1] = errorSignal[0];
}

void isrA() {
  if ((digitalRead(Encoder_output_A) == LOW) and (digitalRead(Encoder_output_B) == HIGH)) {
    angle += angPrStep;
  } else if ((digitalRead(Encoder_output_A) == LOW) and (digitalRead(Encoder_output_B) == LOW)) {
    angle -= angPrStep;
  } else if ((digitalRead(Encoder_output_A) == HIGH) and (digitalRead(Encoder_output_B) == LOW)) {
    angle += angPrStep;
  } else if ((digitalRead(Encoder_output_A) == HIGH) and (digitalRead(Encoder_output_B)) == HIGH) {
    angle -= angPrStep;
  }
  digitalWrite(2, HIGH);
}

void isrB() {
  if ((digitalRead(Encoder_output_B) == LOW) and (digitalRead(Encoder_output_A) == LOW)) {
    angle += angPrStep;
  } else if ((digitalRead(Encoder_output_B) == LOW) and (digitalRead(Encoder_output_A) == HIGH)) {
    angle -= angPrStep;
  } else if ((digitalRead(Encoder_output_B) == HIGH) and (digitalRead(Encoder_output_A) == HIGH)) {
    angle += angPrStep;
  } else if ((digitalRead(Encoder_output_B) == HIGH) and (digitalRead(Encoder_output_A) == LOW)) {
    angle -= angPrStep;
  }
  digitalWrite(2, LOW);
}


//////////////////////////////////////////Stepperkontrol
void stepOne() {
  digitalWrite(motorPinA1, HIGH);
  digitalWrite(motorPinB1, LOW);
  digitalWrite(motorPinA2, LOW);
  digitalWrite(motorPinB2, LOW);
  delay(5);
}
void stepTwo() {
  digitalWrite(motorPinA1, HIGH);
  digitalWrite(motorPinB1, HIGH);
  digitalWrite(motorPinA2, LOW);
  digitalWrite(motorPinB2, LOW);
  delay(5);
}
void stepThree() {
  digitalWrite(motorPinA1, LOW);
  digitalWrite(motorPinB1, HIGH);
  digitalWrite(motorPinA2, LOW);
  digitalWrite(motorPinB2, LOW);
  delay(5);
}
void stepFour() {
  digitalWrite(motorPinA1, LOW);
  digitalWrite(motorPinB1, HIGH);
  digitalWrite(motorPinA2, HIGH);
  digitalWrite(motorPinB2, LOW);
  delay(5);
}
void stepFive() {
  digitalWrite(motorPinA1, LOW);
  digitalWrite(motorPinB1, LOW);
  digitalWrite(motorPinA2, HIGH);
  digitalWrite(motorPinB2, LOW);
  delay(5);
}
void stepSix() {
  digitalWrite(motorPinA1, LOW);
  digitalWrite(motorPinB1, LOW);
  digitalWrite(motorPinA2, HIGH);
  digitalWrite(motorPinB2, HIGH);
  delay(5);
}
void stepSeven() {
  digitalWrite(motorPinA1, LOW);
  digitalWrite(motorPinB1, LOW);
  digitalWrite(motorPinA2, LOW);
  digitalWrite(motorPinB2, HIGH);
  delay(5);
}
void stepEight() {
  digitalWrite(motorPinA1, HIGH);
  digitalWrite(motorPinB1, LOW);
  digitalWrite(motorPinA2, LOW);
  digitalWrite(motorPinB2, HIGH);
  delay(5);
}


void singleStep(int *motorPos, byte forward) {
  int8_t dir = -1;
  if (forward > 0 ) {
    dir = 1;
  }

  totalSteps += dir;

  switch (*motorPos) {
    case 1:
      stepOne();
      if (dir == -1) {
        *motorPos = 8;
      }
      else {
        *motorPos  += dir;
      }
      break;

    case 2:
      stepTwo();
      *motorPos  += dir;
      break;
    case 3:
      stepThree();
      *motorPos += dir;
      break;

    case 4:
      stepFour();
      *motorPos  += dir;
      break;

    case 5:
      stepFive();
      *motorPos  += dir;
      break;

    case 6:
      stepSix();
      *motorPos  += dir;
      break;

    case 7:
      stepSeven();
      *motorPos  += dir;
      break;

    case 8:
      stepEight();
      if (dir == 1) {
        *motorPos = 1;
      }
      else {
        *motorPos  += dir;
      }
      break;

    default:
      *motorPos = 0; // Sets default position
      break;
  }
}


void runSteps(int *Position, float angleWanted) {
  float stepsTest = (float) totalSteps;
  float angleNow = ((360 * stepsTest) / (200 * 12.20 * 2));
  int dir = dirBasedOnAngle(angleWanted, angleNow);
  long stepsWanted = stepsCalculation(angleWanted);

  if (angleWanted > 90 or angleWanted < 0) {
    Serial.print("STOP, angle is too small or large to HANDLE");
    delay(10000000);
  }
  //for (int i = 0; i < abs(stepsWanted); i++) {
  if (abs(stepsWanted)>=1){
    singleStep(Position, dir);
  }
}


//Deciding direction of stepper, either upwards or downwards.
int dirBasedOnAngle(float angleWanted, float currentAngle) {
  int dir;
  Serial.print("Current angle: ");
  Serial.println(currentAngle);
  if (currentAngle < angleWanted) {
    return dir = 1;
  }
  else {
    return dir = 0;
  }
}

// Calculating the necessary steps needed to reach the new position.
long stepsCalculation(float angleWanted) {

  float stepsCalculated =  ((angleWanted * 200 * 12.20 * 2) / 360) - totalSteps ;
  long  stepsInt = (long) stepsCalculated;
  Serial.print("Steps needed to reach angle: ");
  Serial.println(stepsCalculated);
  return stepsInt;
}





//Vinkelberegning
// This functions finds the determinant of a 3x3 Matrix
double determinantOfMatrix(double mat[3][3])
{
  double ans;
  ans = mat[0][0] * (mat[1][1] * mat[2][2] - mat[2][1] * mat[1][2])
        - mat[0][1] * (mat[1][0] * mat[2][2] - mat[1][2] * mat[2][0])
        + mat[0][2] * (mat[1][0] * mat[2][1] - mat[1][1] * mat[2][0]);
  return ans;
}


double * findPlanes(double alphaA, double alphaB, double alphaC) {
  double pointA[3] = { -1000, 0, 0};
  double pointB[3] = {0, 0, 0};
  double pointC[3] = {1000, 0, 0};

  double planeA[4] = {0};
  double planeB[4] = {0};
  double planeC[4] = {0};

  double dirA[3] = {cos(alphaA), sin(alphaA), 0};
  double dirB[3] = {0, sin(alphaB), cos(alphaB)};
  double dirC[3] = {cos(alphaC), sin(alphaC), 0};

  double orthA[3] = {0, 0, 1};
  double orthB[3] = {1, 0, 0};
  double orthC[3] = {0, 0, 1};

  double normA[3] = {dirA[1]*orthA[2] - dirA[2]*orthA[1], -(dirA[0]*orthA[2] - dirA[2]*orthA[0]), dirA[0]*orthA[1] - dirA[1]*orthA[0]};
  double normB[3] = {dirB[1]*orthB[2] - dirB[2]*orthB[1], -(dirB[0]*orthB[2] - dirB[2]*orthB[0]), dirB[0]*orthB[1] - dirB[1]*orthB[0]};
  double normC[3] = {dirC[1]*orthC[2] - dirC[2]*orthC[1], -(dirC[0]*orthC[2] - dirC[2]*orthC[0]), dirC[0]*orthC[1] - dirC[1]*orthC[0]};

  planeA[0] = normA[0];
  planeA[1] = normA[1];
  planeA[2] = normA[2];
  planeA[3] = normA[0] * pointA[0] + normA[1] * pointA[1] + normA[2] * pointA[2];

  planeB[0] = normB[0];
  planeB[1] = normB[1];
  planeB[2] = normB[2];
  planeB[3] = normB[0] * pointB[0] + normB[1] * pointB[1] + normB[2] * pointB[2];

  planeC[0] = normC[0];
  planeC[1] = normC[1];
  planeC[2] = normC[2];
  planeC[3] = normC[0] * pointC[0] + normC[1] * pointC[1] + normC[2] * pointC[2];

  double planes[3][4] = {
    { planeA[0], planeA[1], planeA[2], planeA[3]},
    { planeB[0], planeB[1], planeB[2], planeB[3]},
    { planeC[0], planeC[1], planeC[2], planeC[3]}
  };

  // Matrix d using planes as given in cramer's rule
  double d[3][3] = {
    { planes[0][0], planes[0][1], planes[0][2] },
    { planes[1][0], planes[1][1], planes[1][2] },
    { planes[2][0], planes[2][1], planes[2][2] },
  };
  // Matrix d1 using planes as given in cramer's rule
  double d1[3][3] = {
    { planes[0][3], planes[0][1], planes[0][2] },
    { planes[1][3], planes[1][1], planes[1][2] },
    { planes[2][3], planes[2][1], planes[2][2] },
  };
  // Matrix d2 using planes as given in cramer's rule
  double d2[3][3] = {
    { planes[0][0], planes[0][3], planes[0][2] },
    { planes[1][0], planes[1][3], planes[1][2] },
    { planes[2][0], planes[2][3], planes[2][2] },
  };
  // Matrix d3 using planes as given in cramer's rule
  double d3[3][3] = {
    { planes[0][0], planes[0][1], planes[0][3] },
    { planes[1][0], planes[1][1], planes[1][3] },
    { planes[2][0], planes[2][1], planes[2][3] },
  };

  // Calculating Determinant of Matrices d, d1, d2, d3
  double D = determinantOfMatrix(d);
  double D1 = determinantOfMatrix(d1);
  double D2 = determinantOfMatrix(d2);
  double D3 = determinantOfMatrix(d3);

  double x, y, z;


  if (D != 0) {
    x = D1 / D;  // calculating x using cramer's rule
    y = D2 / D;  // calculating y using cramer's rule
    z = D3 / D;  // calculating z using cramer's rule
    //Serial.println("%lf %lf %lf \n", x, y, z);
  }

  double phi = atan(sqrt(pow(x, 2) + pow(y, 2)) / z);
  double theta = 0;
  if (x > 0) {
    theta = atan(y / x);
  }
  else if (x < 0) {
    theta = atan(y / x) + 3.1415926;
  }
  else {
    theta = 3.1415926 / 2;
  }
  double r = sqrt(pow(x, 2) + pow(y, 2) + pow(z, 2));

  double *angles = (double *) malloc(sizeof(double) * 2);
  angles[0] = theta;
  angles[1] = phi;
  angles[2] = r;
  return angles;
}
