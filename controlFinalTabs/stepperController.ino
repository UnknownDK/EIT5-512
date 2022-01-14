void stepOne() {
  digitalWrite(motorPinA1, HIGH);
  digitalWrite(motorPinB1, LOW);
  digitalWrite(motorPinA2, LOW);
  digitalWrite(motorPinB2, LOW);
  //delay(5);
}
void stepTwo() {
  digitalWrite(motorPinA1, HIGH);
  digitalWrite(motorPinB1, HIGH);
  digitalWrite(motorPinA2, LOW);
  digitalWrite(motorPinB2, LOW);
  //delay(5);
}
void stepThree() {
  digitalWrite(motorPinA1, LOW);
  digitalWrite(motorPinB1, HIGH);
  digitalWrite(motorPinA2, LOW);
  digitalWrite(motorPinB2, LOW);
  //delay(5);
}
void stepFour() {
  digitalWrite(motorPinA1, LOW);
  digitalWrite(motorPinB1, HIGH);
  digitalWrite(motorPinA2, HIGH);
  digitalWrite(motorPinB2, LOW);
  //delay(5);
}
void stepFive() {
  digitalWrite(motorPinA1, LOW);
  digitalWrite(motorPinB1, LOW);
  digitalWrite(motorPinA2, HIGH);
  digitalWrite(motorPinB2, LOW);
  //delay(5);
}
void stepSix() {
  digitalWrite(motorPinA1, LOW);
  digitalWrite(motorPinB1, LOW);
  digitalWrite(motorPinA2, HIGH);
  digitalWrite(motorPinB2, HIGH);
  //delay(5);
}
void stepSeven() {
  digitalWrite(motorPinA1, LOW);
  digitalWrite(motorPinB1, LOW);
  digitalWrite(motorPinA2, LOW);
  digitalWrite(motorPinB2, HIGH);
  //delay(5);
}
void stepEight() {
  digitalWrite(motorPinA1, HIGH);
  digitalWrite(motorPinB1, LOW);
  digitalWrite(motorPinA2, LOW);
  digitalWrite(motorPinB2, HIGH);
  //delay(5);
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
    delay(10000000);
  }
  //for (int i = 0; i < abs(stepsWanted); i++) {
  if (abs(stepsWanted) >= 1) {
    singleStep(Position, dir);
  }
}


//Deciding direction of stepper, either upwards or downwards.
int dirBasedOnAngle(float angleWanted, float currentAngle) {
  int dir;
  Serial.print("Stepper current angle: ");
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
  Serial.print("Stepper steps needed to reach angle: ");
  Serial.println(stepsCalculated);
  return stepsInt;
}
