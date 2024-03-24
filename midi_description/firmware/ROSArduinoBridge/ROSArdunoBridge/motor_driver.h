/***************************************************************
   Motor driver function definitions - by James Nugen
   *************************************************************/

#ifdef L298_MOTOR_DRIVER

  #define RIGHT_MOTOR_DIR  4
  #define RIGHT_MOTOR_ENABLE 5
  #define RIGHT_VREF 7
  #define RIGHT_NSLEEP 6
  
  #define LEFT_MOTOR_DIR 9
  #define LEFT_MOTOR_ENABLE 10
  #define LEFT_VREF 12
  #define LEFT_NSLEEP 11

#endif

void initMotorController();
void setMotorSpeed(int i, int spd);
void setMotorSpeeds(int leftSpeed, int rightSpeed);
