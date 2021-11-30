#define Encoder_output_A 5 // pin 2 of the Arduino
#define Encoder_output_B 19 // pin 4 of the Arduino
//#define Encoder_output_I 18 // pin 3 of the Arduino

int retning = 0;
double angle = 0;
double gearRatio = 6.1838;
double angPrStep = 360.0/(2000.0*gearRatio);

void isrA(){
  if((digitalRead(Encoder_output_A) == LOW) and (digitalRead(Encoder_output_B) == HIGH)){
    retning = 1;
    angle += angPrStep;
  } else if((digitalRead(Encoder_output_A) == LOW) and (digitalRead(Encoder_output_B) == LOW)){
    retning = 0;
    angle -= angPrStep;
  } else if((digitalRead(Encoder_output_A) == HIGH) and (digitalRead(Encoder_output_B) == LOW)){
    retning = 1; //Måske Højre
    angle += angPrStep;
  } else if((digitalRead(Encoder_output_A) == HIGH) and (digitalRead(Encoder_output_B)) == HIGH){
    retning = 0; //Måske Venstre
    angle -= angPrStep;
  }
}

void isrB(){
  if((digitalRead(Encoder_output_B) == LOW) and (digitalRead(Encoder_output_A) == LOW)){
    retning = 1;
    angle += angPrStep;
  } else if((digitalRead(Encoder_output_B) == LOW) and (digitalRead(Encoder_output_A) == HIGH)){
    retning = 0;
    angle -= angPrStep;
  } else if((digitalRead(Encoder_output_B) == HIGH) and (digitalRead(Encoder_output_A) == HIGH)){
    retning = 1; //Måske Højre
    angle += angPrStep;
  } else if((digitalRead(Encoder_output_B) == HIGH) and (digitalRead(Encoder_output_A) == LOW)){
    retning = 0; //Måske Venstre
    angle -= angPrStep;
  }
}

void setup(){
  Serial.begin(115200); // activates the serial communication
  pinMode(Encoder_output_A, INPUT); // sets the Encoder_output_A pin as the input
  pinMode(Encoder_output_B, INPUT); // sets the Encoder_output_B pin as the input
  pinMode(2, OUTPUT);

  //attachInterrupt(Encoder_output_A, isrRisingA, RISING);
  attachInterrupt(Encoder_output_A, isrA, CHANGE);
  //attachInterrupt(Encoder_output_B, isrRisingB, );
  attachInterrupt(Encoder_output_B, isrB, CHANGE);
}

void loop() {

  for (int i=0;i<10000;i++){
    if (i==1){
      Serial.println(angle,7);
    }
  }
  
}
