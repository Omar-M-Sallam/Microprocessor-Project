void line_tracking_mode(){

  left_ir_reading = digitalRead(left_ir);
  center_ir_reading = digitalRead(center_ir);
  right_ir_reading = digitalRead(right_ir);

  int ir_cases = left_ir_reading * 4 + center_ir_reading * 2 + right_ir_reading;

  switch(ir_cases)
 {
    case 0: //000
    stop();
    break;

    case 1: //001
    forw_right(speed_pwm, speed_pwm * 0.8);
    break;

    case 2: //010
    forward(speed_pwm);
    break;

    case 3: //011
    forw_right(speed_pwm, speed_pwm * 0.4);
    break;

    case 4: //100
    forw_right(speed_pwm * 0.8, speed_pwm);
    break;

    case 5: //101
    stop();
    break;

    case 6: //110
    forw_left(speed_pwm * 0.4, speed_pwm);
    break;

    case 7: //111
    forward(speed_pwm);
    break;
    }
  }



