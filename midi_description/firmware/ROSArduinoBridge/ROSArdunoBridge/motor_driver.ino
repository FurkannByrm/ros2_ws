/***************************************************************
   Motor driver definitions
   
   Add a "#elif defined" block to this file to include support
   for a particular motor driver.  Then add the appropriate
   #define near the top of the main ROSArduinoBridge.ino file.
   
   *************************************************************/

#ifdef USE_BASE
   
#ifdef defined POLOLU_MC33926
  /* Include the Pololu library */
  #include "DualMC33926MotorShield.h"

  /* Create the motor driver object */
  DualMC33926MotorShield drive;
  
  /* Wrap the motor driver initialization */
  void initMotorController() {
    drive.init();
  }

  /* Wrap the drive motor set speed function */
  void setMotorSpeed(int i, int spd) {
    if (i == LEFT) drive.setM1Speed(spd);
    else drive.setM2Speed(spd);
  }

  // A convenience function for setting both motor speeds
  void setMotorSpeeds(int leftSpeed, int rightSpeed) {
    setMotorSpeed(LEFT, leftSpeed);
    setMotorSpeed(RIGHT, rightSpeed);
  }
#elif defined L298_MOTOR_DRIVER
  void initMotorController() {
    pinMode(RIGHT_NSLEEP,OUTPUT);
    pinMode(LEFT_NSLEEP,OUTPUT);
    digitalWrite(RIGHT_NSLEEP, HIGH);
    digitalWrite(LEFT_NSLEEP, HIGH);  
    
    pinMode(RIGHT_MOTOR_DIR,OUTPUT);
    pinMode(LEFT_MOTOR_DIR,OUTPUT);
    pinMode(RIGHT_MOTOR_ENABLE,OUTPUT);
    pinMode(LEFT_MOTOR_ENABLE,OUTPUT);
    
  }
  
  void setMotorSpeed(int i, int spd) {
    unsigned char reverse = 0;
  
    if (spd < 0)
    {
      spd = -spd;
      reverse = 1;
    }
    if (spd > 255)
      spd = 255;
    
    if (i == LEFT) { 
      if      (reverse == 0) { digitalWrite(LEFT_MOTOR_DIR, 1); analogWrite(LEFT_MOTOR_ENABLE, spd/2);}
      else if (reverse == 1) { digitalWrite(LEFT_MOTOR_DIR, 0); analogWrite(LEFT_MOTOR_ENABLE, spd/2);}
    }
    else /*if (i == RIGHT) //no need for condition*/ {
      if      (reverse == 0) { digitalWrite(RIGHT_MOTOR_DIR, 0); analogWrite(RIGHT_MOTOR_ENABLE, spd/3.5);}
      else if (reverse == 1) { digitalWrite(RIGHT_MOTOR_DIR, 1); analogWrite(RIGHT_MOTOR_ENABLE, spd/3.5);}
    }
  }
  
  void setMotorSpeeds(int leftSpeed, int rightSpeed) {
    setMotorSpeed(LEFT, leftSpeed);
    setMotorSpeed(RIGHT, rightSpeed);
  }
#else
  #error A motor driver must be selected!
#endif

#endif
