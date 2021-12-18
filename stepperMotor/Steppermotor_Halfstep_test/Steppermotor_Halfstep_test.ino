void setup() {
  Serial.begin(115200);
  delay(1500);   //venter paa serial
}

int motorPinA1 = 1;
int motorPinB1 = 2;
int motorPinA2 = 3;
int motorPinB2 = 4;


int motorPos = 1;
long totalSteps = 0;


//
void stepOne() {
  digitalWrite(motorPinA1, HIGH);
  digitalWrite(motorPinB1, LOW);
  digitalWrite(motorPinA2, LOW);
  digitalWrite(motorPinB2, LOW);
}
void stepTwo() {
  digitalWrite(motorPinA1, HIGH);
  digitalWrite(motorPinB1, HIGH);
  digitalWrite(motorPinA2, LOW);
  digitalWrite(motorPinB2, LOW);
}
void stepThree() {
  digitalWrite(motorPinA1, LOW);
  digitalWrite(motorPinB1, HIGH);
  digitalWrite(motorPinA2, LOW);
  digitalWrite(motorPinB2, LOW);
}
void stepFour() {
  digitalWrite(motorPinA1, LOW);
  digitalWrite(motorPinB1, HIGH);
  digitalWrite(motorPinA2, HIGH);
  digitalWrite(motorPinB2, LOW);
}
void stepFive() {
  digitalWrite(motorPinA1, LOW);
  digitalWrite(motorPinB1, LOW);
  digitalWrite(motorPinA2, HIGH);
  digitalWrite(motorPinB2, LOW);
}
void stepSix() {
  digitalWrite(motorPinA1, LOW);
  digitalWrite(motorPinB1, LOW);
  digitalWrite(motorPinA2, HIGH);
  digitalWrite(motorPinB2, HIGH);
}
void stepSeven() {
  digitalWrite(motorPinA1, LOW);
  digitalWrite(motorPinB1, LOW);
  digitalWrite(motorPinA2, LOW);
  digitalWrite(motorPinB2, HIGH);
}
void stepEight() {
  digitalWrite(motorPinA1, HIGH);
  digitalWrite(motorPinB1, LOW);
  digitalWrite(motorPinA2, LOW);
  digitalWrite(motorPinB2, HIGH);
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
  float angleNow = ((360*stepsTest) / (200 * 50 * 2));
  int dir = dirBasedOnAngle(angleWanted, angleNow);
  long stepsWanted = stepsCalculation(angleWanted);

  if (angleWanted > 90 or angleWanted < 0) {
    Serial.print("STOP, angle is to small or large to HANDLE");
    delay(10000000);
  }
  Serial.print("7:Steppning: ");
  Serial.println(stepsWanted);
  for (int i = 0; i < abs(stepsWanted); i++) {
    
    singleStep(Position, dir);
  }
    Serial.print("9:pos: ");
  Serial.println(totalSteps);
}


//Deciding direction of stepper, either upwards or downwards.
int dirBasedOnAngle(float angleWanted, float currentAngle) {
  int dir;
  Serial.print("1: Total steps already taken to know angle: ");
  Serial.println(totalSteps);
  Serial.print("2: Current angle: ");
  Serial.println(currentAngle);
  if (currentAngle < angleWanted) {
    Serial.println("3: Stepping uoward");
    return dir = 1;
  }
  else {
    Serial.println("3: Stepping downwards");
    return dir = 0;
  }
}

// Calculating the necessary steps needed to reach the new position.
long stepsCalculation(float angleWanted) {
  Serial.print("4: Steps calculation from the given angle wanted: ");
  Serial.println(angleWanted);
  
  float stepsCalculated =  ((angleWanted * 200 * 50 * 2) / 360) - totalSteps ;
  long  stepsInt = (long) stepsCalculated; 
  Serial.print("5: Steps needed to reach angle: ");
  Serial.println(stepsCalculated);
  return stepsInt;
}



// Giving different inputs to test the code.
void loop() {
  runSteps(&motorPos,15);
  delay(1000);
  runSteps(&motorPos,70);
  delay(1000);
  runSteps(&motorPos,30);
  delay(100000);

}
