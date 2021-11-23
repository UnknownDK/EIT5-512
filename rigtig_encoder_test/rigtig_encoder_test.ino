#define Encoder_output_A 2 // pin 2 of the Arduino
#define Encoder_output_B 4 // pin 4 of the Arduino
#define Encoder_output_I 3 // pin 3 of the Arduino

int Count_pulses = 0;
void setup() {
Serial.begin(9600); // activates the serial communication
pinMode(Encoder_output_A,INPUT); // sets the Encoder_output_A pin as the input
pinMode(Encoder_output_B,INPUT); // sets the Encoder_output_B pin as the input
pinMode(Encoder_output_I,INPUT); // sets the Encoder_output_I pin as the input
attachInterrupt(digitalPinToInterrupt(Encoder_output_A),DC_Motor_Encoder,RISING);
attachInterrupt(digitalPinToInterrupt(Encoder_output_I),DC_Motor_Reset,RISING);
}

void loop() {
}

void DC_Motor_Encoder(){
  int b = digitalRead(Encoder_output_B);
  if(b > 0){
    Count_pulses++;
  }
  else{
    Count_pulses--;
  }
}

void DC_Motor_Reset(){
  Serial.println(Count_pulses);
  Count_pulses = 0;
}
