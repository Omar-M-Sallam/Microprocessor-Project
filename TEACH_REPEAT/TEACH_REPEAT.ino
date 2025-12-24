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