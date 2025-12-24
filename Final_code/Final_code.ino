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

//==================Movements===================
void forward(int speed) {
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, HIGH);
  digitalWrite(IN4, LOW);
  analogWrite(ENA, speed);
  analogWrite(ENB, speed);
}

void backward(int speed) {
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, HIGH);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, HIGH);
  analogWrite(ENA, speed);
  analogWrite(ENB, speed);
}

void right(int speed) {
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, HIGH);
  digitalWrite(IN3, HIGH);
  digitalWrite(IN4, LOW);
  analogWrite(ENA, speed);
  analogWrite(ENB, speed);
}

void left(int speed) {
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, HIGH);
  analogWrite(ENA, speed);
  analogWrite(ENB, speed);
}

void forw_right(int leftPwm, int rightPwm) {
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, HIGH);
  digitalWrite(IN4, LOW);
  analogWrite(ENA, leftPwm);
  analogWrite(ENB, rightPwm);
}

void forw_left(int leftPwm, int rightPwm) {
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, HIGH);
  digitalWrite(IN4, LOW);
  analogWrite(ENA, leftPwm);
  analogWrite(ENB, rightPwm);
}

void back_right(int leftPwm, int rightPwm) {
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, HIGH);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, HIGH);
  analogWrite(ENA, leftPwm);
  analogWrite(ENB, rightPwm);
}

void back_left(int leftPwm, int rightPwm) {
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, HIGH);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, HIGH);
  analogWrite(ENA, leftPwm);
  analogWrite(ENB, rightPwm);
}

void stop() {
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, HIGH);
  digitalWrite(IN3, HIGH);
  digitalWrite(IN4, HIGH);
}

//==================Ultrasonic Reading===================
float ultrasonic_reading(int mode) { //mode(1 or 2) 1->front ultrasonic 2->back ultrasonic

  if(mode == 1) { //check the mode if 1 then assigne the values to the front sensor variables
    do{
      digitalWrite(trig_1, LOW); //clearing the trig pin
      delayMicroseconds(2);
      digitalWrite(trig_1, HIGH); //sending the signal of lenght 10us 
      delayMicroseconds(10);
      digitalWrite(trig_1, LOW); //ending the signal

      time_1 = pulseIn(echo_1, HIGH); //calculating the time the signal took from the beginning until it hits the echo pin
    }
    while(time_1 == 0); //remeasure time if it is 0 

    distance_1 = 0.0343 * time_1 / 2.0; //calculating the distance
    return distance_1; 
    
  }

  else if (mode == 2) { //check the mode if 2 then assigne the values to the back sensor variables
    do{
      digitalWrite(trig_2, LOW); //clearing the trig pin
      delayMicroseconds(2);
      digitalWrite(trig_2, HIGH); //sending the signal of lenght 10us 
      delayMicroseconds(10);
      digitalWrite(trig_2, LOW); //ending the signal
      
      time_2 = pulseIn(echo_2, HIGH); //calculating the time the signal took from the beginning until it hits the echo pin
    }
    while(time_2 == 0); //remeasure time if it is 0

    distance_2 = 0.0343 * time_2 / 2.0; //calculating the distance
    return distance_2;
    }
    
  else {return -1;} //handling wrong mode 
}

//==================Manual Mode===================
void manual_mode(char cmd) {

//speed control
if (cmd >= '0' && cmd <= '9') { // accept only numeric ASCII digits
    int value = cmd - '0';     // convert the char to int
    speed_pwm = map(value, 0, 9, 60, 255);
    SerialBT.println(speed_pwm); 
  }
  switch(cmd){
    case 'F':
    forward(speed_pwm); 
    break;
    
    case 'B': 
    backward(speed_pwm); 
    break;

    case 'L': 
    left(speed_pwm);  
    break;

    case 'R':
    right(speed_pwm); 
    break;

    case 'Q':
    forw_right(speed_pwm, speed_pwm * 0.5); 
    break;
    
    case 'E':
    forw_left(speed_pwm * 0.5, speed_pwm); 
    break;

    case 'Z':
    back_right(speed_pwm, speed_pwm * 0.5); 
    break;

    case 'C':
    back_left(speed_pwm * 0.5, speed_pwm); 
    break;

    case 'S':  
    stop(); 
    break;
  }
}

//==================Auto Mode===================
void auto_mode() {
  forward(speed_pwm);
  front_servo.write(90); // center

  front_dist = ultrasonic_reading(1);

  if (front_dist < 30) {
    backward(speed_pwm);
    delay(600);

    stop();
    // Look left
    front_servo.write(0);
    delay(600);
    left_front_dist = ultrasonic_reading(1);
    if (left_front_dist <= 0) left_front_dist = ultrasonic_reading(1);

    // Look right
    front_servo.write(180);
    delay(600);
    right_front_dist = ultrasonic_reading(1);
    if (right_front_dist <= 0) right_front_dist = ultrasonic_reading(1);

    // Decide direction
    if (right_front_dist >= left_front_dist) {
      right(speed_pwm);
      delay(1000);
    } else {
      left(speed_pwm );
      delay(1000);
    }
  }
}

//==================Parking Mode===================
void parking_mode(){

  left_prox_reading = digitalRead(left_prox);
  right_prox_reading = digitalRead(right_prox);

  switch(current_state){
    
    case idle :

    //start the motion for navigation
    forward(speed_pwm); //start the motion for navigation

    //check the first wall direction
    if (right_prox_reading == 1 && left_prox_reading == 0){current_state = right_parking_scan;}
    else if (right_prox_reading == 0 && left_prox_reading == 1){current_state = left_parking_scan;} 
    break;


    case right_parking_scan:
    if (right_prox_reading == 0){ //void place detected on the right
      start_time = millis();//initialize the start time
      current_state = right_scan_end;
    }
    break;

    case left_parking_scan:
    if (left_prox_reading == 0){//void place detected on the left
      start_time = millis();//initialize the start time
      current_state = left_scan_end;
    }
    break;

    case right_scan_end:
    if (right_prox_reading == 1){ //second wall detected means the robot reached the end of the slot
      stop(); //stop the motion
      current_time = millis(); //storing the time when the robot reached the end of the slot (second wall)
      elapsed_time = current_time - start_time; //calculating the time the robot have taken starting from the first wall detection till the second wall
      start_time = 0; //restarting the timer
      unsigned long slot_distance = (elapsed_time * robot_speed) / 1000; //calculating the slot distance to check if the slot is sutable for the robot (dist = time * velocity)
      if (slot_distance >= 20){ // 20cm from the robot dimentions
        current_state = start_parking_right;
      }
      else{SerialBT.println("NOT SIUTABLE PARKING SLOT");}
    }
    break;

    case left_scan_end:
    if (left_prox_reading == 1){ //second wall detected means the robot reached the end of the slot
      stop(); //stop the motion
      current_time = millis(); //storing the time when the robot reached the end of the slot (second wall)
      elapsed_time = current_time - start_time; //calculating the time the robot have taken starting from the first wall detection till the second wall
      start_time = 0; //restarting the timer
      unsigned long slot_distance = (elapsed_time * robot_speed) / 1000; //calculating the slot distance to check if the slot is sutable for the robot (dist = time * velocity)
      if (slot_distance >= 20){ // 20cm from the robot dimentions
        current_state = start_parking_left;
      }
      else{SerialBT.println("NOT SIUTABLE PARKING SLOT");}
    }
    break;

    case start_parking_right:
    back_right(speed_pwm, speed_pwm * 0.6); //moving back right till both sensors read 1
    if (right_prox_reading == 1 && left_prox_reading == 1){
      current_state = end_parking_right;
      }
    break;

    case start_parking_left:
    back_left(speed_pwm * 0.6, speed_pwm);///moving back left till both sensors read 1
    if (right_prox_reading == 1 && left_prox_reading == 1){
      current_state = end_parking_left;
      }
    break;

    case end_parking_right:
    stop();
    SerialBT.println("DONE PARKING ON THE RIGHT");
    break;

    case end_parking_left:
    stop();
    SerialBT.println("DONE PARKING ON THE LEFT");
    break;
    
  }
}

//==================Teach Mode===================
void teach_mode(char cmd) { 
  
  switch(cmd){ 
    case 'F': 
    forward(speed_pwm); 
    break;
    
    case 'B': 
    backward(speed_pwm); 
    break; 
    
    case 'L': 
    left(speed_pwm); 
    break; 
    
    case 'R': 
    right(speed_pwm); 
    break; 
    
    case 'Q': 
    forw_right(speed_pwm, speed_pwm * 0.5); 
    break; 
    
    case 'E': 
    forw_left(speed_pwm * 0.5, speed_pwm); 
    break; 
    
    case 'Z': 
    back_right(speed_pwm, speed_pwm * 0.5); 
    break; 
    
    case 'C': 
    back_left(speed_pwm * 0.5, speed_pwm); 
    break; 
    
    case 'S': 
    stop(); 
    break; 
    } 

    EEPROM.write(addr, cmd); // Staging command sequentially in RAM 
    EEPROM.commit(); // now saved in flash
    lastAddr = addr; //storing last address
    addr++; //increment the address value
    if (addr >= 64) {addr = 0;} // wrap around 
  }

//==================Repeat Mode===================
void repeat_mode(){
  for (int i = 0; i <= lastAddr; i++){
    char storedCmd = (char)EEPROM.read(i); //retrieve the stored commands form the ram in sequence by looping on each address
    switch(storedCmd){ 
      case 'F': 
      forward(speed_pwm); 
      break;
        
      case 'B': 
      backward(speed_pwm); 
      break; 
        
      case 'L': 
      left(speed_pwm); 
      break; 
        
      case 'R': 
      right(speed_pwm); 
      break; 
        
      case 'Q': 
      forw_right(speed_pwm, speed_pwm * 0.5); 
      break; 
        
      case 'E': 
      forw_left(speed_pwm * 0.5, speed_pwm); 
      break; 
        
      case 'Z': 
      back_right(speed_pwm, speed_pwm * 0.5); 
      break; 
        
      case 'C': 
      back_left(speed_pwm * 0.5, speed_pwm); 
      break; 
        
      case 'S': 
      stop(); 
      break; 
    }
    delay(500); //a delay to observe the motion
  }
}

//==================Line Follower Mode===================
void line_tracking_mode(){

  left_ir_reading = digitalRead(left_ir);
  center_ir_reading = digitalRead(center_ir);
  right_ir_reading = digitalRead(right_ir);

  int ir_cases = right_ir_reading * 2 + center_ir_reading;

  switch(ir_cases)
 {
    case 0: //00
    forward(speed_pwm);
    break;

    case 1: //01
    forw_left(speed_pwm * 0.8, speed_pwm );
    break;

    case 2: //10
    forw_right(speed_pwm, speed_pwm * 0.8);
    break;

    case 3: //11
    stop();
    break;
  }
}

//==================Porcess Command===================
void processCommand(char cmd) {
  switch (cmd) {
    //switching mode
    case 'M': 
    current_mode = 'M';
    break;

    case 'A': 
    current_mode = 'A';
    break;

    case 'P':
    if (current_mode != 'P') {
      current_state = idle;   // reset FSM once when entering parking mode
      start_time = 0;
      elapsed_time = 0;
    }
    current_mode = 'P';
    parking_mode();
    break;
    
    case 'U':
    // Reset EEPROM once when entering teach mode
    for (int i = 0; i < 64; i++) {
      EEPROM.write(i, 0); // clear all cells 
      }
      EEPROM.commit(); // make erase permanent 
      addr = 0; // reset address pointer 
      lastAddr = -1; // reset last address tracker
      current_mode = 'U';
      break;
      
      
      case 'O':
      current_mode = 'O';
      replayDone = false;
      break;

    case'T':
    current_mode = 'T';
    break;
    
    default:
    break;
  }
  
  //ensure that manual mode never works but when the cmd = M 
  if(current_mode == 'M'){ manual_mode(cmd);}

  else if(current_mode == 'A'){auto_mode();}

  else if(current_mode == 'P'){parking_mode();}

  else if(current_mode == 'U'){teach_mode(cmd);}

  else if(current_mode == 'T'){line_tracking_mode();}

  else if(current_mode == 'O' && replayDone == false){
    repeat_mode();
    replayDone = true; //running the repeat once
  }  
}
//==================Kalman Read Current===================
float readCurrent() {
    int adcValue = analogRead(CURRENT_PIN);        // 0 → 4095
    float voltage = adcValue * (3.3 / 4095.0);     // ADC → Voltage

    float voltageOffset = 1.65;   // 0A reference (ACS712)
    float sensitivity   = 0.185;  // V/A (example: ACS712-5A)

    float current = (voltage - voltageOffset) / sensitivity;
    return current;
}

// ====================
// Kalman Update Function
// ====================
void updateSOC() {
  unsigned long now = millis();
  unsigned long elapsed = now - lastUpdate;

  // Run update every 1000 ms (1 second)
  if (elapsed >= 1000) {
    float dt = elapsed / 1000.0;   // convert ms → seconds
    lastUpdate = now;

    // Read current
    float currentA = readCurrent();

    // Kalman Prediction with real dt
    socEstimator.predict(currentA, dt);

    // (Optional) Correction
    // float measuredSOC = 0.75;
    // socEstimator.correct(measuredSOC);

    // Send data via Bluetooth
    SerialBT.print(socEstimator.soc * 100.0);
    SerialBT.print(" % | P = ");
    SerialBT.println(socEstimator.P);
  }
}

void loop() {
  
   cmd = SerialBT.read();
    processCommand(cmd);

    updateSOC();
  /*int readVal = SerialBT.read();
    if (readVal == -1) return;  // ignore empty reads completely
    cmd = (char)readVal;
    processCommand(cmd);*/

 /*Serial.println(ultrasonic_reading(1));
  delay(500);*/
  
}






/*
void parking_mode(){

  right_back_dist = ultrasonic_reading(2);
  
  switch(current_state){
    
    case idle :
    back_servo.write(0);
    right_back_dist = ultrasonic_reading(2);
    //start the motion for navigation
    forward(speed_pwm); //start the motion for navigation

    //check the first wall direction
    if (right_back_dist <= 30){current_state = right_parking_scan;}
    break;


    case right_parking_scan:
    right_back_dist = ultrasonic_reading(2);
    if (right_back_dist >= 30){ //void place detected on the right
      start_time = millis();//initialize the start time
      current_state = right_scan_end;
    }
    break;

 

    case right_scan_end:
    right_back_dist = ultrasonic_reading(2);
    if (right_back_dist <= 30){ //second wall detected means the robot reached the end of the slot
      stop(); //stop the motion
      current_time = millis(); //storing the time when the robot reached the end of the slot (second wall)
      elapsed_time = current_time - start_time; //calculating the time the robot have taken starting from the first wall detection till the second wall
      start_time = 0; //restarting the timer
      unsigned long slot_distance = (elapsed_time * robot_speed) / 1000; //calculating the slot distance to check if the slot is sutable for the robot (dist = time * velocity)
      if (slot_distance >= 20){ // 20cm from the robot dimentions
        current_state = start_parking_right;
      }
      else{SerialBT.println("NOT SIUTABLE PARKING SLOT");}
    }
    break;

    case start_parking_right:
    back_servo.write(180);
    back_right(speed_pwm, speed_pwm * 0.6); //moving back right till the back ultrasonic reads less than 7cm
    if (right_back_dist <= 10){
      current_state = end_parking_right;
      }
    break;

    case end_parking_right:
    stop();
    SerialBT.println("DONE PARKING ON THE RIGHT");
    break;
    
  }
}
*/
