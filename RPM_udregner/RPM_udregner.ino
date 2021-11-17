#define Encoder_output_A 2 // pin 2 of the Arduino
#define Encoder_output_B 4 // pin 4 of the Arduino
#define Encoder_output_I 3 // pin 3 of the Arduino

int Count_pulses = 0;
int RPMCount = 0;
float RPM = 0.0;
uint64_t previousMillis = 0;
uint64_t currentMillis = 0;
int interval = 0;
float rpm_to_radians = 0.10471975512;
float rad_to_deg = 57.29578;
float ang_velocity = 0;   
float ang_velocity_deg = 0;


void setup() {
Serial.begin(19200); // activates the serial communication
pinMode(Encoder_output_A,INPUT); // sets the Encoder_output_A pin as the input
pinMode(Encoder_output_B,INPUT); // sets the Encoder_output_B pin as the input
pinMode(Encoder_output_I,INPUT); // sets the Encoder_output_I pin as the input
attachInterrupt(digitalPinToInterrupt(Encoder_output_A),DC_Motor_Encoder,RISING);
attachInterrupt(digitalPinToInterrupt(Encoder_output_I),DC_Motor_Reset,RISING);
}

void loop() {
  //Serial.println(Count_pulses);
  currentMillis = millis();
  Serial.println(RPM);
  if (currentMillis - previousMillis > interval) {
   //RPM_Motor();
  }
}

void DC_Motor_Encoder(){
  int b = digitalRead(Encoder_output_B);
  if(b > 0){
    Count_pulses++;
  }
  else{
    Count_pulses--;
  }
  RPM_Motor();
}

void DC_Motor_Reset(){ //pulser pr omgang
  //Serial.println(Count_pulses);
  //Count_pulses = 0;
}
void RPM_Motor(){
  interval = currentMillis - previousMillis;
  RPM= (1.0/(interval/1000))/500;  
  previousMillis = currentMillis;
}
