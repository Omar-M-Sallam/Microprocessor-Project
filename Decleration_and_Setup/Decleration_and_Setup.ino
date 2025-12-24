#include "BluetoothSerial.h"
BluetoothSerial SerialBT; //creating object for BluetoothSerial
// Motor pins
#define ENB 13
#define IN1 26
#define IN2 27
#define IN3 14
#define IN4 12
#define ENA 25

float speed_pwm = 170; //initialize speed 
float robot_speed = 43; //measured speed of the robot in cm/s 

//Ultrasonic Sensors
#define trig_2 17 //TX2 pin (front sensor)
#define echo_2 16 //RX2 pin (front sensor)
#define trig_1 2 //back sensor
#define echo_1 15 //back sensor

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

bool left_ir_reading = false;
bool center_ir_reading = false;
bool right_ir_reading = false;
  
//Proximity IR
#define left_prox  5 
#define right_prox 32

bool left_prox_reading = false;
bool right_prox_reading = false;

//servo motors
#include <ESP32Servo.h>
#define back_servo_pin 33
#define front_servo_pin  4

Servo front_servo; //creating object for the front servo
Servo back_servo; //creating object for the back servo

int front_servo_pos;
int back_servo_pos;

//initialize variables to store the reading from bluetooth
char current_mode = 'N'; //initialize variable with none mode
char cmd;

//initialize time for FSM's 
unsigned long start_time;
unsigned long current_time;
unsigned long elapsed_time;

//states of the parking mode
enum scan_parking {
  idle,
  right_parking_scan,
  left_parking_scan,
  right_scan_end,
  left_scan_end, 
  start_parking_right, 
  start_parking_left, 
  end_parking_left, 
  end_parking_right
  };

scan_parking current_state = idle; //initialize the FSM to idle

//Teach and Repeat 
#include <EEPROM.h>
int addr = 0; //initialize address variable
int lastAddr = -1; //initialize last address variable to store last address
bool replayDone = false;

//kalman filter
#include "kalman_soc.h"
unsigned long lastUpdate = 0;
//current sensor
#define CURRENT_PIN 18
float CURRENT_PIN_read;
// ====================
// Kalman Object
// ====================
KalmanSOC socEstimator(
  0.8,    // Initial SOC = 80%
  0.1,    // Initial uncertainty P
  0.01,   // Q - process noise
  0.05,   // R - measurement noise
  2.6    // Battery capacity in Ah
);

void setup() {
  //MD
  pinMode(ENB, OUTPUT);
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);
  pinMode(ENA, OUTPUT);

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
  pinMode(CURRENT_PIN, INPUT);

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

  // initialize emulated EEPROM
  EEPROM.begin(64);  //64 Byte
}