#define Encoder_output_A 5 // pin 2 of the Arduino
#define Encoder_output_B 19 // pin 4 of the Arduino
#define Encoder_output_I 18 // pin 3 of the Arduino


unsigned long previousMillis = 0;
unsigned long currentMillis = 0;
int rounds = 0;
int count = 0;
double latest = 0;

//

void IRAM_ATTR I_Counter() {
  currentMillis = millis();
  if ((currentMillis - previousMillis) > 1){
    latest = (rounds/500.0)*6000.0;
    rounds = 0;
    previousMillis = millis();
    Serial.println(latest);
  }
  rounds += 1;
  
  
}

void setup() {
  Serial.begin(115200); // activates the serial communication
  pinMode(Encoder_output_A, INPUT); // sets the Encoder_output_A pin as the input
  pinMode(Encoder_output_B, INPUT); // sets the Encoder_output_B pin as the input
  pinMode(Encoder_output_I, INPUT);
  attachInterrupt(Encoder_output_A, I_Counter, FALLING);
}

void loop() {
  
}


/*
  void DC_Motor_Encoder() {
  if (someCount < 400) {
    currentMillis = micros();
    delta = float(currentMillis - previousMillis);
    someArray[someCount] = 0.0125664 / (delta / 1000000.0);
    previousMillis = micros();
    someCount += 1;
  } else{
    for (int i=0;i<400;i++){
      avgAngSpeed += someArray[i];
    }
    Serial.println(avgAngSpeed/400, 7);
    avgAngSpeed = 0;
    //Serial.println(angularSpeed / 100.0, 7);
    someCount = 0;
  }
  int b = digitalRead(Encoder_output_B);
  if (b > 0) {
    vinkel += vinkel_pr_puls;
  }
  else {
    vinkel -= vinkel_pr_puls;
  }


  }*/
