#include "BluetoothSerial.h"
BluetoothSerial SerialBT; //creating object for BluetoothSerial
// Motor pins
#define ENB 13
#define IN1 26
#define IN2 27
#define IN3 14
#define IN4 12
#define ENA 25

//Built in led
#define led 2
bool led_state = false;

//Ultrasonic Sensors
#define trig_1 17 //TX2 pin (front sensor)
#define echo_1 16 //RX2 pin (front sensor)
#define trig_2 2 //back sensor
#define echo_2 15 //back sensor

float time_1; //time taken by the signal of front sensor 
float time_2; //time taken by the signal of back sensor 
float distance_1; //distance read by front sensor
float distance_2; //distance read by back sensor

float front_dist; //distance of front of the robot
float back_dist; //distance of back of the robot
float right_front_dist; //distance of right front of the robot
float left_front_dist; //distance of left front of the robot
float right_back_dist; //distance of right back of the robot
float left_back_dist; //distance of left back of the robot

//IR Sensor(line follower)
#define left_ir   23 
#define center_ir 22 
#define right_ir  21 

bool left_reading = false;
bool center_ir_reading = false;
bool right_ir_reading = false;
  
//Proximity IR
#define left_prox  32 
#define right_prox 5

bool left_prox_reading = false;
bool right_prox_reading = false;

//current sensor
#define current_sensor 18

float current_sensor_read;

//servo motors
#include <ESP32Servo.h>
#define front_servo_pin 33
#define back_servo_pin  4

Servo front_servo; //creating object for the front servo
Servo back_servo; //creating object for the back servo

int front_servo_pos;
int back_servo_pos;

void setup() {
  //MD
  pinMode(ENB, OUTPUT);
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);
  pinMode(ENA, OUTPUT);

  //led
  pinMode(led, OUTPUT);

  //Ultrasonic
  pinMode(trig_1, OUTPUT);
  pinMode(echo_1, INPUT);
  pinMode(trig_2, OUTPUT);
  pinMode(echo_2, INPUT);

  //IR(line)
  pinMode(left_ir, INPUT);
  pinMode(center_ir, INPUT);
  pinMode(right_ir, INPUT);

  //IR(proximity)
  pinMode(left_prox, INPUT);
  pinMode(right_prox, INPUT);

  //current sensor
  pinMode(current_sensor, INPUT);

  //servo
  // Allow allocation of all timers
  ESP32PWM::allocateTimer(0);
	ESP32PWM::allocateTimer(1);
	ESP32PWM::allocateTimer(2);
	ESP32PWM::allocateTimer(3);

	front_servo.setPeriodHertz(50); // standard 50 hz servo
  back_servo.setPeriodHertz(50); // standard 50 hz servo

  front_servo.attach(front_servo_pin, 1000, 2000);
  back_servo.attach(back_servo_pin, 1000, 2000);
  // using default min/max of 1000us and 2000us
	// different servos may require different min/max settings

  //initialize serial
  Serial.begin(115200);
  SerialBT.begin("O&L_Smart_Car");
}

void loop() {
  // put your main code here, to run repeatedly:

}
