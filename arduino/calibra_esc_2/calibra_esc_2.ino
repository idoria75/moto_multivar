// Program to calibrate ESCs following the manual:
// https://www.firediy.fr/files/drone/HW-01-V4.pdf

#include <Servo.h>

Servo mot;
int pin_mot = 9;
int pwm_min = 1000;
int pwm_max = 2000;
char data, old_data;
unsigned long count = 0;

void setup() {
  Serial.begin(115200);
  mot.attach(pin_mot,pwm_min,pwm_max);
  mot.writeMicroseconds(pwm_min);
  Serial.println("ESC CALIBRATOR");
  Serial.println("Instructions: ");
  Serial.println("Please, enter i to set pwm to max and o to set pwm to min.");
  Serial.println("Follow the .txt file with the instructions");
  Serial.println("");
}

void loop() {
  if(Serial.available()){
    data = Serial.read();
    if(data != old_data){
      count++;
      old_data = data;
      switch(data){
        case 'o':
          mot.writeMicroseconds(pwm_min);
          Serial.print(count); Serial.println(": PWM set to MIN");
          break;
        case 'i':
          mot.writeMicroseconds(pwm_max);
          Serial.print(count); Serial.println(": PWM set to MAX");
          break;
        default:
          Serial.print(count); Serial.println(": Please enter i(Max) or o(Min)");
          break;
      }
    }
  }
}
