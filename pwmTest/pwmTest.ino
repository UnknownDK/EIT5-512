// Pins til PWM
const int pinPWMR = 16;  // 16 corresponds to GPIO16 men hvad vil vi vælge ???
const int pinPWML = 17;  // blå ledning

// setting PWM properties
const int freq = 10000; // PWM frekvens
const int pwmChannelR = 0; // Ved jeg ikke helt hvad gør
const int pwmChannelL = 1; // Ved jeg ikke helt hvad gør
const int resolution = 12; // bit resolution tror jeg (så 0 - 4095)



void setup() {
  // put your setup code here, to run once:
  // configure LED PWM functionalitites
  ledcSetup(pwmChannelR, freq, resolution);
  ledcSetup(pwmChannelL, freq, resolution);

  // attach the channel to the GPIO to be controlled
  ledcAttachPin(pinPWMR, pwmChannelR);
  ledcAttachPin(pinPWML, pwmChannelL);
  
}

void loop() {
  // put your main code here, to run repeatedly:
  
  ledcWrite(pwmChannelR, 2000);
  ledcWrite(pwmChannelL, 0);
  delay(500);
  ledcWrite(pwmChannelR, 4095);
  ledcWrite(pwmChannelL, 0);
  delay(500);
  ledcWrite(pwmChannelR, 0);
  ledcWrite(pwmChannelL, 0);
  delay(500);
  ledcWrite(pwmChannelR, 0);
  ledcWrite(pwmChannelL, 2000);
  delay(500);


}
