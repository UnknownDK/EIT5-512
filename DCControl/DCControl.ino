#define Encoder_output_A 5 // pin 2 of the Arduino
#define Encoder_output_B 19 // pin 4 of the Arduino
//#define Encoder_output_I 18 // pin 3 of the Arduino

int retning = 0;
double angle = 0;
double gearRatio = 6.6;
double angPrStep = 360.0/(500.0*gearRatio);

void isrRising(){
  if(digitalRead(Encoder_output_B) == LOW){
    retning = 1; //Måske Højre
    //Serial.println("Rising B LOW Retning 1");
    angle += angPrStep;
  } else if(digitalRead(Encoder_output_B) == HIGH){
    retning = 0; //Måske Venstre
    //Serial.println("Rising B HIGH Retning 0");
    angle -= angPrStep;
  }
  //Serial.print("Hej 1 ");
  Serial.println(angle);
}

void setup(){
  Serial.begin(115200); // activates the serial communication
  pinMode(Encoder_output_A, INPUT); // sets the Encoder_output_A pin as the input
  pinMode(Encoder_output_B, INPUT); // sets the Encoder_output_B pin as the input

  attachInterrupt(Encoder_output_A, isrRising, RISING);
}

void loop() {
  
}
