#include <Arduino.h>
#include <micro_ros_platformio.h>
#include <midi_custom_interfaces/msg/encoder_data.h>
#include <midi_custom_interfaces/msg/motor_command.h>
#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <std_msgs/msg/float64_multi_array.h>


#include <rmw_microros/rmw_microros.h>
#include <rcl/error_handling.h>
#include <std_msgs/msg/int32.h>


#define LED_PIN 5
#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){return false;}}
#define EXECUTE_EVERY_N_MS(MS, X)  do { \
  static volatile int64_t init = -1; \
  if (init == -1) { init = uxr_millis();} \
  if (uxr_millis() - init > MS) { X; init = uxr_millis();} \
} while (0)\

rclc_support_t support;
rcl_node_t node;
rcl_timer_t timer;
rclc_executor_t executor;
rcl_allocator_t allocator;
rcl_publisher_t publisher;
std_msgs__msg__Int32 msg;
bool micro_ros_init_successful;

enum states {
  WAITING_AGENT,
  AGENT_AVAILABLE,
  AGENT_CONNECTED,
  AGENT_DISCONNECTED
} state;

void timer_callback(rcl_timer_t * timer, int64_t last_call_time)
{
  (void) last_call_time;
  if (timer != NULL) {
    rcl_publish(&publisher, &msg, NULL);
    msg.data++;
  }
}
bool create_entities()
{
  allocator = rcl_get_default_allocator();

  // create init_options
  RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));

  // create node
  RCCHECK(rclc_node_init_default(&node, "int32_publisher_rclc", "", &support));

  // create publisher
  RCCHECK(rclc_publisher_init_best_effort(
    &publisher,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32),
    "std_msgs_msg_Int32"));

  // create timer,
  const unsigned int timer_timeout = 1000;
  RCCHECK(rclc_timer_init_default(
    &timer,
    &support,
    RCL_MS_TO_NS(timer_timeout),
    timer_callback));

  // create executor
  executor = rclc_executor_get_zero_initialized_executor();
  RCCHECK(rclc_executor_init(&executor, &support.context, 1, &allocator));
  RCCHECK(rclc_executor_add_timer(&executor, &timer));

  return true;
}

void destroy_entities()
{
  rmw_context_t * rmw_context = rcl_context_get_rmw_context(&support.context);
  (void) rmw_uros_set_context_entity_destroy_session_timeout(rmw_context, 0);

  rcl_publisher_fini(&publisher, &node);
  rcl_timer_fini(&timer);
  rclc_executor_fini(&executor);
  rcl_node_fini(&node);
  rclc_support_fini(&support);
}


struct MotorDriver
{
    uint8_t in1;
    uint8_t in2;
    uint8_t vref;
    uint8_t nsleep;

    MotorDriver(uint8_t in1, uint8_t in2, uint8_t vref, uint8_t nsleep)
    {
        this->in1 = in1;
        this->in2 = in2;
        this->vref = vref;
        this->nsleep = nsleep;
    }

    void setPins()
    {
            
        pinMode(in1, OUTPUT);
        pinMode(in2, OUTPUT);
        pinMode(vref, OUTPUT);
        pinMode(nsleep, OUTPUT);

        digitalWrite(nsleep, HIGH);
        analogWrite(vref, 0);
        analogWrite(in2,0);
    }

    void setVel(int16_t pwm)
    {   
        int direction = HIGH;
        if(pwm < 0)
        {
            direction = LOW;
        }

        digitalWrite(in1, direction);
/*         digitalWrite(in2, HIGH);
 */        analogWrite(in2, abs(pwm));
    }
};

struct Encoder
{
    uint8_t phaseA;
    uint8_t phaseB;

    Encoder(uint8_t phaseA, uint8_t phaseB)
    {
        this->phaseA = phaseA;
        this->phaseB = phaseB;
        
    }

    void setPins(void(*callback_func)())
    {
        attachInterrupt(phaseA, callback_func, RISING);
        pinMode(phaseB, INPUT);
        
    }
};

#if !defined(MICRO_ROS_TRANSPORT_ARDUINO_SERIAL)
#error This example is only available for Arduino framework with serial transport.
#endif

std_msgs__msg__Float64MultiArray encoder_data_msg;
std_msgs__msg__Float64MultiArray motor_cmd_msg;

MotorDriver leftMotorDriver(1, 2, 42, 41);
MotorDriver rightMotorDriver(14, 13, 22, 11);

Encoder leftEncoder(10, 9);
Encoder rightEncoder(8, 18);

volatile int64_t leftEncoderCount = 0;
volatile int64_t rightEncoderCount = 0;

unsigned long lastUpdateTime = 0;
const unsigned long loopPeriod = 10;

rcl_init_options_t init_options;
rcl_publisher_t encoder_info_pub;
rcl_subscription_t motor_command_sub;

void countLeftEncoder()
{
    if(digitalRead(leftEncoder.phaseB) == LOW)
    {
        leftEncoderCount = leftEncoderCount -1;
    }
    else
    {
        leftEncoderCount = leftEncoderCount + 1;
    }
}

void countRightEncoder()
{
    if(digitalRead(rightEncoder.phaseB) == LOW)
    {
        rightEncoderCount = rightEncoderCount -1;
    }
    else
    {
        rightEncoderCount = rightEncoderCount + 1;
    }
}

void motorCmdCallback(const void *msgin)
{
    const std_msgs__msg__Float64MultiArray *cmd_msg = (const std_msgs__msg__Float64MultiArray *)msgin;
    motor_cmd_msg = *cmd_msg;
}

void setup()
{
    Serial.begin(115200);
    delay(1000);
    set_microros_serial_transports(Serial);
    pinMode(LED_PIN, OUTPUT);

    state = WAITING_AGENT;

    msg.data = 0;

    leftMotorDriver.setPins();
    rightMotorDriver.setPins();

    leftEncoder.setPins(countLeftEncoder);
    rightEncoder.setPins(countRightEncoder);

    init_options = rcl_get_zero_initialized_init_options();
    allocator = rcl_get_default_allocator();
   /*  motor_cmd_msg.data.data = new double[2];
    encoder_data_msg.data.data = new double[2]; */

    motor_cmd_msg.data.capacity = 2;
    motor_cmd_msg.data.data = (double*)malloc(sizeof(double)*2);
    motor_cmd_msg.data.size = 0;

    encoder_data_msg.data.capacity = 2;
    encoder_data_msg.data.data = (double*)malloc(sizeof(double) * 2);
    encoder_data_msg.data.size = 0;

    /* static double mem[2];
    encoder_data_msg.data.data = mem; */

    encoder_data_msg.layout.dim.capacity = 2;
    encoder_data_msg.layout.dim.size = 0;
    encoder_data_msg.layout.dim.data = (std_msgs__msg__MultiArrayDimension*) malloc(encoder_data_msg.layout.dim.capacity * sizeof(std_msgs__msg__MultiArrayDimension));

    rclc_support_init(&support, 0, NULL, &allocator);
    rclc_node_init_default(&node, "esp32_hardware_node", "", &support);
    rclc_publisher_init_default(
        &encoder_info_pub,
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float64MultiArray),
        "encoder_data");

    rclc_subscription_init_default(
        &motor_command_sub,
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float64MultiArray),
        "motor_command");

    rclc_executor_init(&executor, &support.context, 1, &allocator);

    rclc_executor_add_subscription(
        &executor,
        &motor_command_sub,
        &motor_cmd_msg,
        &motorCmdCallback,
        ON_NEW_DATA);

    lastUpdateTime = millis();
}

void loop()
{   

    switch (state) {
    case WAITING_AGENT:
      EXECUTE_EVERY_N_MS(500, state = (RMW_RET_OK == rmw_uros_ping_agent(100, 1)) ? AGENT_AVAILABLE : WAITING_AGENT;);
      break;
    case AGENT_AVAILABLE:
      state = (true == create_entities()) ? AGENT_CONNECTED : WAITING_AGENT;
      if (state == WAITING_AGENT) {
        destroy_entities();
      };
      break;
    case AGENT_CONNECTED:
      EXECUTE_EVERY_N_MS(200, state = (RMW_RET_OK == rmw_uros_ping_agent(100, 1)) ? AGENT_CONNECTED : AGENT_DISCONNECTED;);
      if (state == AGENT_CONNECTED) {
        rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100));
      }
      break;
    case AGENT_DISCONNECTED:
      destroy_entities();
      state = WAITING_AGENT;
      break;
    default:
      break;
  }

  if (state == AGENT_CONNECTED) {
    digitalWrite(LED_PIN, 1);
  } else {
    digitalWrite(LED_PIN, 0);
  }
    unsigned long currentTime = millis();

    if((currentTime - lastUpdateTime) >= loopPeriod)
    {

        rclc_executor_spin_some(&executor, RCL_MS_TO_NS(10));

        double leftEncoderCountF = static_cast<double>(leftEncoderCount);
        double rightEncoderCountF = static_cast<double>(rightEncoderCount);
        
        encoder_data_msg.data.data[0] = leftEncoderCountF;
        encoder_data_msg.data.size = 1;
        encoder_data_msg.data.data[1] = rightEncoderCountF;
        encoder_data_msg.data.size = 2;
        /* memcpy(&encoder_data_msg.data.data[0], &leftEncoderCountF, sizeof(leftEncoderCountF));
        memcpy(&encoder_data_msg.data.data[1], &rightEncoderCountF, sizeof(rightEncoderCountF)); */

        /* memcpy(&encoder_data_msg.data.data[0], &leftEncoderCountF, sizeof(leftEncoderCountF));
        memcpy(&encoder_data_msg.data.data[1], &rightEncoderCountF, sizeof(rightEncoderCountF)); */
        rcl_publish(&encoder_info_pub, &encoder_data_msg, NULL);

        int leftDir, rightDir = LOW;
        
        leftMotorDriver.setVel(motor_cmd_msg.data.data[0]);
        rightMotorDriver.setVel((-1.0)*motor_cmd_msg.data.data[1]);
        encoder_data_msg.data.size = 0;
    }

}