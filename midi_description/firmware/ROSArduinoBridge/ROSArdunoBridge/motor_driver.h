/***************************************************************
   Motor driver function definitions - by James Nugen
   *************************************************************/

#ifdef L298_MOTOR_DRIVER
  #define RIGHT_MOTOR_DIR  9
  #define RIGHT_MOTOR_ENABLE 10
  #define RIGHT_VREF 11
  #define RIGHT_NSLEEP 12
  #define LEFT_MOTOR_DIR 18
  #define LEFT_MOTOR_ENABLE 13
  #define LEFT_VREF 6
  #define LEFT_NSLEEP 7

#endif

void initMotorController();
void setMotorSpeed(int i, int spd);
void setMotorSpeeds(int leftSpeed, int rightSpeed);
