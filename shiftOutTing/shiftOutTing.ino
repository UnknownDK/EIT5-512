#define CLOCK_PIN 5
#define DATA_PIN 7
#define LE_PIN 9
#define FOELER_PIN 10

void shiftOutADF(uint8_t dataPin, uint8_t clockPin, unsigned long val);

int start = 1;
unsigned long R[5];

void setup() {
  Serial.begin(115200);
  pinMode(CLOCK_PIN, OUTPUT);
  pinMode(DATA_PIN, OUTPUT);
  pinMode(LE_PIN, OUTPUT);
  pinMode(FOELER_PIN, INPUT_PULLUP);

  digitalWrite(LE_PIN, LOW);
  digitalWrite(CLOCK_PIN, LOW);  
 
  R[0] = 0x181f0000;
  R[1] = 0x00000001;
  R[2] = 0x0d050002; //10 divider //Old config: 0x0f2a0002;// for 2.5 mA: 0x072a0002;
  R[3] = 0x00000003;
  R[4] = 0x00000004; //for activation of negative bleed current 0x01800004;
}


void loop() {
  start = digitalRead(FOELER_PIN);
  while(start == 0){
    for (int i = 4 ; i >= 0 ; i--){
      shiftOutADF(DATA_PIN, CLOCK_PIN, R[i]);
      digitalWrite(LE_PIN, HIGH); 
      delayMicroseconds(10);     //micros inserted for test
      digitalWrite(LE_PIN, LOW); 
    }
    exit(0);
  }

}

void shiftOutADF(uint8_t dataPin, uint8_t clockPin, unsigned long val){
  digitalWrite(clockPin, LOW); 
  int i;
  for (i = 0; i < 32; i++)  {
    digitalWrite(dataPin, !!(val & (1 << (31 - i))));   // !! in order to make everything above 1 equal 1
    digitalWrite(clockPin, HIGH);
    delayMicroseconds(10);     //micros inserted for test
    digitalWrite(clockPin, LOW);
    delayMicroseconds(10);            
  }
}
