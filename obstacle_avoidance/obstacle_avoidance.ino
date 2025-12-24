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