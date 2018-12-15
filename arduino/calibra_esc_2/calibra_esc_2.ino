#include <Servo.h>

Servo mot;
int pin_mot = 9;
int data;
int pwm_value = 1001;

void setup() {
  Serial.begin(115200);
  mot.attach(pin_mot,1000,2000);
  mot.writeMicroseconds(1000);
  
  Serial.println("SetUp Ready");
}

void loop() {
  if(Serial.available()){
    data = Serial.parseInt();
    if(data != pwm_value){
      if((data >= 1000) and (data <= 2000)){
        pwm_value = data;
        mot.writeMicroseconds(pwm_value);
        Serial.print("PWM Write: "); Serial.println(pwm_value);
      }
      else{
        Serial.println("Input is not in range 1000-2000");
        Serial.print("Input: ");
        Serial.println(data);
      }
    }
    else{
      Serial.println("Input not different from previous one");
    }
  }
  delay(200);
}
