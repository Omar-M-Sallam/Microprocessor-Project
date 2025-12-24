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
    back_right(speed_pwm, speed_pwm * 0.6); //moving back right till the back ultrasonic reads less than 7cm
    if (right_prox_reading == 1 && left_prox_reading == 1){
      current_state = end_parking_right;
      }
    break;

    case start_parking_left:
    back_left(speed_pwm * 0.6, speed_pwm);//moving back right till the back ultrasonic reads less than 7cmback_left(speed_pwm * 0.7, speed_pwm);
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