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