#define Encoder_output_A 5 // pin 2 of the Arduino
#define Encoder_output_B 19 // pin 4 of the Arduino
//#define Encoder_output_I 18 // pin 3 of the Arduino

// Pin til PWM
const int pinPWM = 16;  // 16 corresponds to GPIO16 men hvad vil vi vælge ???

// setting PWM properties
const int freq = 10000; // PWM frekvens
const int pwmChannel = 0; // Ved jeg ikke helt hvad gør
const int resolution = 12; // bit resolution tror jeg (så 0 - 4095)
 
double gearRatio = 6.1838;
double angPrStep = 360.0/(2000.0*gearRatio);


double Kd = 0;  // Differential gain
double Kp = 0;  // Proportional gain
double Ki = 0;  // Integrator gain
double samplingPeriod = 1;   // 1/frekvens (enhed er millis)

double angle = 0;         // y(t) (DER VI ER - LÆSES MED ENCODER) 
double controlSignal[2] = {0, 0}; // u(t)
double errorSignal[3] = {0, 0, 0};   // e(t) til e(t-2)
double setPoint = 0;      // r(t) (DER VI VIL HEN)

unsigned long currentTime, previousTime, someDelay;
double elapsedTime;



void isrA(){
  if((digitalRead(Encoder_output_A) == LOW) and (digitalRead(Encoder_output_B) == HIGH)){
    angle += angPrStep;
  } else if((digitalRead(Encoder_output_A) == LOW) and (digitalRead(Encoder_output_B) == LOW)){
    angle -= angPrStep;
  } else if((digitalRead(Encoder_output_A) == HIGH) and (digitalRead(Encoder_output_B) == LOW)){
    angle += angPrStep;
  } else if((digitalRead(Encoder_output_A) == HIGH) and (digitalRead(Encoder_output_B)) == HIGH){
    angle -= angPrStep;
  }
}

void isrB(){
  if((digitalRead(Encoder_output_B) == LOW) and (digitalRead(Encoder_output_A) == LOW)){
    angle += angPrStep;
  } else if((digitalRead(Encoder_output_B) == LOW) and (digitalRead(Encoder_output_A) == HIGH)){
    angle -= angPrStep;
  } else if((digitalRead(Encoder_output_B) == HIGH) and (digitalRead(Encoder_output_A) == HIGH)){
    angle += angPrStep;
  } else if((digitalRead(Encoder_output_B) == HIGH) and (digitalRead(Encoder_output_A) == LOW)){
    angle -= angPrStep;
  }
}

void setup(){
  // configure LED PWM functionalitites
  ledcSetup(pwmChannel, freq, resolution);
  
  // attach the channel to the GPIO to be controlled
  ledcAttachPin(pinPWM, pwmChannel);


  Serial.begin(115200); // activates the serial communication
  pinMode(Encoder_output_A, INPUT); // sets the Encoder_output_A pin as the input
  pinMode(Encoder_output_B, INPUT); // sets the Encoder_output_B pin as the input
  pinMode(2, OUTPUT);

  attachInterrupt(Encoder_output_A, isrA, CHANGE);
  attachInterrupt(Encoder_output_B, isrB, CHANGE);
}




void loop() {
  currentTime = millis();
  elapsedTime = (double)(currentTime - previousTime);
  errorSignal[0] = angle;
  setPoint = 45;            //bliver sat af vinkelberegning
  
  errorSignal[0] = setPoint - angle;

  // Controllerdelen
  P = Kp*(errorSignal[0]-errorSignal[1]);
  I = samplingPeriod*Ki*errorSignal[0];
  D = (Kd/samplingPeriod)*(errorSignal[0]-2*errorSignal[1]+errorSignal[2]);
  controlSignal[0] = P + I + D + controlSignal[1];
  controlSignal[1] = controlSignal[0];
  ledcWrite(pwmChannel, 200);

  previousTime = currentTime;

  // Venter på tiden h som er tiden mellem samples
  someDelay = millis();
  errorSignal[2] = errorSignal[1];
  errorSignal[1] = errorSignal[0];
  while (millis() < someDelay + h){}
}


//u(t) =Kd/h·[e(t)−2e(t−1) + e(t−2)] + Kp·[e(t)−e(t−1)] + h·Ki·e(t) + u(t−1)
/*
Pseudo code
x=0 initialization of past values for first time trough the loop
Ko = gain
C1 = 1-bT
C2=Ko(aT-1)
Read r and y from the A/D converter
e = r-y
u = x+Koe
Send u to the D/A converter
x= C1u+C2e ( update x for next run through the loop)
Wait until T seconds from last read


*/
