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