#include <Servo.h>
#include <Mpu6050.h>
// Define Gyroscope, Servo objects
Servo gyro, myServo;
// Define imu object
Mpu6050 imu;
// Define limits
#define MIN_PULSE_LENGTH 1000 // (usec)
#define MAX_PULSE_LENGTH 2000 // (usec)
#define MIN_SERVO_ANGLE     0 // (deg)
#define MAX_SERVO_ANGLE   180 // (deg)
#define MIN_ANALOG_READ     0 // (analog resolution)
#define MAX_ANALOG_READ  1023 // (analog resolution)
// Define Pins
const int potPin = A0;
const int buttonPin = 2;
const int servoPin = 5;
const int gyroPin = 9;
const int ledPin = 13;

bool flag_esc = false; // True for ESC ON
int cur_speed  = 1200;
volatile byte speed_state = 1; // state <-> cur_speed: 1 = 1200; 2 = 1500; 3 = 1800;
volatile bool buttonPressed = false;
int button_counter = 0;
int potValue = 0;

float offset_posicao_x = 0.65;
float offset_posicao_y = 0.0;
float offset_velocidade_x = -10.58;
float offset_velocidade_y = 0.0;

Rotation rot;
Velocity vel;

void setup() {
    // Initialize Serial Port
    Serial.begin(115200);
    
    // Start IMU service
    imu.Begin();

    // Setup Button
    pinMode(ledPin, OUTPUT);
    pinMode(buttonPin, INPUT);

    // Initialization routine
    initialize_imu();
    initialize_servo();
    initialize_gyro();
    attachInterrupt(digitalPinToInterrupt(buttonPin), changeSpeed, FALLING);
    //delay(1000);
    Serial.println("Setup Complete");
}

void loop() {
    // if(digitalRead(buttonPin)){
    //  changeSpeed();
    // }
    if(buttonPressed == true){
        button_counter = button_counter + 1;
        switch (speed_state){
            case 1:
                cur_speed = 1200;
                break;
            case 2:
                cur_speed = 1500;
                break;
            case 3:
                cur_speed = 1800;
                break;
            case 4:
                cur_speed = 1200;
                speed_state = 1;
                break;
        }
        Serial.print("Set speed to: "); Serial.println(cur_speed);
        gyro.writeMicroseconds(cur_speed);
        delay(1000);
        buttonPressed = false;
    }
    potServoControl();
    handle_imu();
    delay(10);
}

void initialize_imu(){
    imu.SetXPosOffset(offset_posicao_x);
    imu.SetYPosOffset(offset_posicao_y); 
    imu.SetXSpeedOffset(offset_velocidade_x);
    imu.SetYSpeedOffset(offset_velocidade_y);
}
void initialize_servo(){
    myServo.attach(servoPin);
}

void initialize_gyro(){
    gyro.attach(gyroPin, MIN_PULSE_LENGTH, MAX_PULSE_LENGTH);
    gyro.writeMicroseconds(1000);
    Serial.println("Please, turn on ESC");
    Serial.println("When ESC is on, press button!");
    while(not(flag_esc)){
        flag_esc = digitalRead(buttonPin);
    }
    gyro.writeMicroseconds(1800);
    Serial.println("Gyro setup complete!");
}

void changeSpeed(){
    if(buttonPressed == false){
        speed_state = speed_state+1;
        buttonPressed = true;
    }
}

void potServoControl(){
    potValue = analogRead(potPin);
    potValue = map(potValue,MIN_ANALOG_READ,MAX_ANALOG_READ,MIN_SERVO_ANGLE,MAX_SERVO_ANGLE);
    myServo.write(potValue);
}

void handle_imu(){
    imu.Update(); // Pega os valores atuais

  rot = imu.GetRotation(); // Pega rotação e velocidade
  vel = imu.GetVelocity();

  // Imprime os valores, comente a linha para desabilitar
  Serial.print("X: ");
  Serial.print(rot.x); Serial.print('\t');
  Serial.print(vel.x); Serial.print('\t');

  Serial.print("Y: ");
  Serial.print(rot.y); Serial.print('\t');
  Serial.print(vel.y); Serial.print('\t');

  Serial.println();

  //delay(10);
}