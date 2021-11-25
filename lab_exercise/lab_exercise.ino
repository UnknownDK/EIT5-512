#include <TimerOne.h>

// ESCON DRIVE
#define PWM_PIN 9
#define ENABLE_PIN 7

#define ESCON_PWM_MAX 916
#define ESCON_PWM_MIN 105

#define PWM_50 512
#define PWM_100 1024

// SENSORS
#define POTMETER A0
#define TACHOMETER A3

// Sampling
unsigned long t0 = 0;
const unsigned long ts = 5;

// Square wave
int square_count = 0;

// Controller related
const float theta_base_ref = 368;
float theta_ref = theta_base_ref;

// Gains
float Kp = 5;
float Kv = 0;

// Control inputs (to be applied to PWM signal)
float u_p = 0;
float u_v = 0;

// PWM output
float PWM_out = 0;

void setup() {
    pinMode(PWM_PIN, OUTPUT);
    pinMode(ENABLE_PIN, OUTPUT);

    // Timer setup
    Timer1.initialize(2000);
    Timer1.stop();
    Timer1.restart();

    // Setup to 500Hz and 50% duty-cycle
    // (To give ESCON a valid PWM at startup)
    Timer1.pwm(PWM_PIN, 512, 2000);
    
    // Disable ESCON to clear errors
    digitalWrite(ENABLE_PIN, LOW);
    delay(2000);
    // Enable ESCON
    digitalWrite(ENABLE_PIN, HIGH);

    Serial.begin(57600);
    t0 = millis();
}

void loop() {
    // Raw sensor readouts
    float theta_meas = analogRead(POTMETER);
    float omega_meas = analogRead(TACHOMETER) - 500.0;

    // Generate square wave
    if (square_count > 1600) {
        square_count = 0;
    }
    else {
        square_count++;
    }
    
    // Hookup reference to square wave
    if (square_count < 800) {
        theta_ref = theta_base_ref - 150;
    }
    else {
        theta_ref = theta_base_ref + 150;
    }

    // P regulator w. gain Kp
    u_p = Kp * (theta_ref - theta_meas);
    
    // Inner-loop w. gain Kv
    u_v = Kv * omega_meas;
    
    // Duty-cycle = 50% + (u_p - u_v)
    PWM_out = PWM_50 + (u_p + u_v);

    // Limit PWM_out (for the sake of ESCON!)
    if (PWM_out > ESCON_PWM_MAX) {
        PWM_out = ESCON_PWM_MAX;
    }
    if (PWM_out < ESCON_PWM_MIN) {
        PWM_out = ESCON_PWM_MIN;
    }

    // Invert PWM_out
    PWM_out = PWM_100 - PWM_out;

    // Logging
    Serial.print(theta_meas);
    Serial.print(",");
    Serial.print(theta_ref);
    Serial.print(",");
    Serial.print(PWM_out);
    Serial.print("\n");

    Timer1.setPwmDuty(PWM_PIN, PWM_out);

    // Wait
    while ((t0 + ts) > millis()){}
    t0 = millis();
}