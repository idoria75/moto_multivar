#include <Servo.h>

// Define Gyroscope, Servo
Servo gyro, myServo;
// Define limits
#define MIN_PULSE_LENGTH 1000 // (usec)
#define MAX_PULSE_LENGTH 2000 // (usec)
#define MIN_SERVO_ANGLE     0 // (deg)
#define MAX_SERVO_ANGLE   180 // (deg)
#define MIN_ANALOG_READ     0 // (analog resolution)
#define MAX_ANALOG_READ  1023 // (analog resolution)
// Define Pins
const int buttonPin = 2;
const int servoPin = 5;
const int gyroPin = 9;
const int ledPin = 13;

bool flag_esc = false; // True for ESC ON
int cur_speed  = 1200;
volatile byte speed_state = 1; // state <-> cur_speed: 1 = 1200; 2 = 1500; 3 = 1800;
volatile bool buttonPressed = false;
int button_counter = 0;

void setup() {
    // put your setup code here, to run once:
	Serial.begin(115200);

	// Setup Button
	pinMode(ledPin, OUTPUT);
	pinMode(buttonPin, INPUT);

	// Initialization routine
	initialize_servo();
	initialize_gyro();
	attachInterrupt(digitalPinToInterrupt(buttonPin), changeSpeed, FALLING);

	//delay(1000);
	
	Serial.println("Setup Complete");
}

void loop() {
    if(digitalRead(buttonPin)){
    	changeSpeed();
    }
    if(buttonPressed == true){
    	Serial.print("Set speed to: ");
    	Serial.println(cur_speed);
    	gyro.writeMicroseconds(cur_speed);
    	delay(1000);
    	buttonPressed = false;
    	Serial.print("Button counter: ");
    	Serial.println(button_counter);
    }
    
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
		button_counter = button_counter+1;
		if(cur_speed == 1200){
			cur_speed = 1500;
			buttonPressed = true;
			return;
		}
		if(cur_speed == 1500){
			cur_speed = 1800;
			buttonPressed = true;
			return;
		}
		if(cur_speed == 1800){
			cur_speed = 1200;
			buttonPressed = true;
			return;
		}
	}
}