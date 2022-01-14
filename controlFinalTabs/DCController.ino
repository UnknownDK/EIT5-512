


void isrA(){
  if((digitalRead(Encoder_output_A) == LOW) and (digitalRead(Encoder_output_B) == HIGH)){
    angle += angPrStep;
  } else if((digitalRead(Encoder_output_A) == LOW) and (digitalRead(Encoder_output_B) == LOW)){
    angle -= angPrStep;
  } else if((digitalRead(Encoder_output_A) == HIGH) and (digitalRead(Encoder_output_B) == LOW)){
    angle += angPrStep;
  } else if((digitalRead(Encoder_output_A) == HIGH) and (digitalRead(Encoder_output_B)) == HIGH){
    angle -= angPrStep;
  }
  digitalWrite(2, HIGH);
}

void isrB(){
  if((digitalRead(Encoder_output_B) == LOW) and (digitalRead(Encoder_output_A) == LOW)){
    angle += angPrStep;
  } else if((digitalRead(Encoder_output_B) == LOW) and (digitalRead(Encoder_output_A) == HIGH)){
    angle -= angPrStep;
  } else if((digitalRead(Encoder_output_B) == HIGH) and (digitalRead(Encoder_output_A) == HIGH)){
    angle += angPrStep;
  } else if((digitalRead(Encoder_output_B) == HIGH) and (digitalRead(Encoder_output_A) == LOW)){
    angle -= angPrStep;
  }
  digitalWrite(2, LOW);
}


void DCCONTROL(){
  
  //elapsedTime = (double)(DCcurrentTime - previousTime);

  //setPoint += 0.011;            //bliver sat af vinkelberegning
  
  errorSignal[0] = setPoint[0] - angle;
  //Serial.print("ERROR: "); Serial.println(errorSignal[0]);

  // Controllerdelen
  P = Kp*(errorSignal[0]-errorSignal[1]);
  I = (samplingPeriod/1000)*Ki*errorSignal[0];
  D = (Kd/(samplingPeriod/1000))*(errorSignal[0]-2*errorSignal[1]+errorSignal[2]);
  controlSignal[0] = 8.191*(P + I + D) + controlSignal[1]; // burde ganges med 81.91, men det bliver for meget
  if(controlSignal[0]>8191){
    controlSignal[0] = 8191;
  } else if (controlSignal[0]<-8191){
    controlSignal[0] = -8191;
  }
  controlSignal[1] = controlSignal[0];
  
  //Serial.print("Controlsignal: "); Serial.println(controlSignal[0]);
  //Serial.print("CONTROLLER: "); Serial.println(controlSignal[0]);
  //Serial.print("Setpoint: "); Serial.println(setPoint);
  
  if (controlSignal[0] > 0){
    ledcWrite(pwmChannelR, 0); //4095/controlsignal[0]
    ledcWrite(pwmChannelL, abs(controlSignal[0]));
  } else {
    ledcWrite(pwmChannelR, abs(controlSignal[0]));
    ledcWrite(pwmChannelL, 0);
  }
 
  //previousTime = DCcurrentTime;
  
  //*setPoint += 0.011;
  //someDelay = millis();
  
  errorSignal[2] = errorSignal[1];
  errorSignal[1] = errorSignal[0];

  
  // Venter på tiden h som er tiden mellem samples
  //while (millis() < someDelay + samplingPeriod){}
  //while (millis() - currentTime < samplingPeriod){}
}
