#include <Servo.h>

Servo mot;
int data;
int pwm_value = 1001;
int pwm_value_old = pwm_value;

int flag = 0;

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  mot.attach(9,1000,2000);
  mot.writeMicroseconds(1000);
  
  Serial.println("SetUp Ready");
}

void loop() {
  // put your main code here, to run repeatedly:
  if(Serial.available()){
    pwm_value=Serial.parseInt();
    //pwm_value = atoi(data);
    if(pwm_value_old != pwm_value){
      pwm_value_old = pwm_value;
      Serial.print("Wrinting value: ");
      Serial.println(pwm_value);
      flag = 1;
      
    }
    else{
      Serial.print("Value: ");
      Serial.println(data);
      delay(100);
    }
  }
  if((pwm_value >= 1000) and (pwm_value <= 2000) and (flag == 1)){
    mot.writeMicroseconds(pwm_value);
    Serial.print("PWM Write: "); Serial.println(pwm_value);
    flag = 0;
  }
}
