float un = 1.0; // x[n]
float imVar[8][2] = {0.0}; // w[n]
float yn = 0.0; // y[n]
float gain = 0.0335;
float b[8][2] = {{1.9711, 0.9712}, { -1.9924, 0.9914}, {2.0288, 1.0296}, { -2.0075, 1.0087}, {0.0024, 0.3571}, { -0.0463, 0.3966}, {0.0018, 0.3898}, { -0.0601, 0.3607}};
float a[8][2] = {{0.0572, 0.2862}, { -0.1155, 0.2938}, {0.3595, 0.2939}, { -0.4050, 0.3001}, {0.1536, 0.4865}, { -0.1972, 0.4911}, {0.8505, 0.6936}, { -0.8996, 0.6991}};
long currentMillis = 0;

// freq sweep
const int numFreq =140;
int n = 0;
float x = 0.0;
float Fc = 1 * pow(10, 6);
float res = 0.0;
float maxRes = 0.0;
float freq[numFreq] = {0.0};
float amp[numFreq] = {0.0};


void setup() {
  Serial.begin(1000000);
}

void loop() {
  for (float a = 1.0; a < 13.0; a += float((13.0 / numFreq))) {
    for ( int i = 0; i < 300; i++) {
      un = sin(2 * 3.14159265 * x * Fc * a);
      res = gain * SOS(un);
      x += 3.6 * pow(10, -8);
      if (res > maxRes && x > 3.6 * pow(10, -8) * 100) {
        maxRes = abs(res);
      }
    }
    freq[n] = Fc * a;
    amp[n] = maxRes;
    maxRes = 0;
    x=0.0;
    n++;
  }

  for (int i = 0; i < numFreq; i++) {
    Serial.print(freq[i]);
    Serial.print(",");
    Serial.print(amp[i], 7);
    Serial.print("\n");
  }
  delay(1000);
}

float SOS(float ADCin) {
  for (int i = 0; i < 8; i++) {
    yn = ADCin + imVar[i][0];
    imVar[i][0] = b[i][0] * ADCin - a[i][0] * yn + imVar[i][1];
    imVar[i][1] = b[i][1] * ADCin - a[i][1] * yn;
    ADCin = yn;
  }
  return yn;
}
