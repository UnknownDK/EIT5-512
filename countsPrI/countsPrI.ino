#define Encoder_output_A 5 // pin 2 of the Arduino
#define Encoder_output_B 19 // pin 4 of the Arduino
#define Encoder_output_I 18 // pin 3 of the Arduino


unsigned long previousMillis = 0;
unsigned long currentMillis = 0;
int rounds = 0;
int count = 0;
double latest = 0;

//
/*
void IRAM_ATTR A_Counter() {
  if (digitalRead(Encoder_output_B) == LOW){
    currentMillis = micros();
    if ((currentMillis - previousMillis) > 4000){
      latest = ((rounds/500.0))*1500.0;
      rounds = 0;
      previousMillis = micros();
      //Serial.println(String(latest) + "," + String(previousMillis)); //Til Scriptet
      Serial.println(String(latest)); //Til plotteren
    }
    rounds += 1;
  }
}

void IRAM_ATTR B_Counter() {
  if (digitalRead(Encoder_output_A) == LOW){
    currentMillis = micros();
    if ((currentMillis - previousMillis) > 4000){
      latest = ((rounds/500.0))*1500.0;
      rounds = 0;
      previousMillis = micros();
      //Serial.println(String(latest) + "," + String(previousMillis)); //Til Scriptet
      Serial.println(String(latest)); //Til plotteren
    }
    rounds += 1;
  }
}
*/

void IRAM_ATTR A_Counter() {
    rounds += 1;
}

void IRAM_ATTR I_Counter() {
  Serial.println(rounds);
  rounds = 0;
}

void setup() {
  Serial.begin(115200); // activates the serial communication
  pinMode(Encoder_output_A, INPUT); // sets the Encoder_output_A pin as the input
  pinMode(Encoder_output_B, INPUT); // sets the Encoder_output_B pin as the input
  pinMode(Encoder_output_I, INPUT);
  attachInterrupt(Encoder_output_A, A_Counter, RISING);
  attachInterrupt(Encoder_output_I, I_Counter, RISING);
}

void loop() {
  
}
