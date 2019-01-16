// Atmega 128 dev board custom Libraries
#include <io128.h>
#include <sra128.h>
#include <pinDefsManualNew.h>

#include"avr/io.h"
#include"avr/interrupt.h"

// ROS Includes
#include <ros.h>
#include <std_msgs/String.h>
#include <std_msgs/Int16MultiArray.h>
#include <std_msgs/Float32MultiArray.h>
#include <geometry_msgs/Twist.h>

// Encoder Pin Definition
#define encoder_0_PinA 18         // D0
#define encoder_0_PinB 20         // D2
#define encoder_1_PinA 19         // D1
#define encoder_1_PinB 21         // D3

// Autonomous Navigation control variables
char key_auto_nav;
bool autonav = 0;
int pwm_l = 0, pwm_r = 0;


// Tele-operation control variables
char key_teleop;
int pwm = 80;                     // Linear motion PWM value
int pwm_spot = 40;                // Spot Turn PWM value
unsigned int turn_angle = 45;     // Spot Turn Step size

volatile unsigned long prev_time_l = 0.0;
volatile unsigned long prev_time_r = 0.0;
volatile int ticks_left, ticks_right, cumulative_ticks = 0;

ros::NodeHandle nh;               // ROS Node Handler

std_msgs::Int16MultiArray ticks;  // ROS msg array for encoder ticks
std_msgs::Float32MultiArray vel;  // ROS msg array for instantaneous velocity

// Declare ROS Publisher for encoder ticks and instantaneous velocity
ros::Publisher encoder_ticks("ticks", &ticks);
ros::Publisher wheel_velocities("velocity", &vel);

// ROS callback for tele-operation commands
void teleop_cb(const std_msgs::String& command)
{
  key_teleop = command.data[0];
}

// ROS callback for autonomous navigation
void auto_nav_cb(const std_msgs::Int16MultiArray& pwm_val)
{
  if ((pwm_val.data[0] * pwm_val.data[1]) < 0)  // Check for spot turn velocitites
  {
    if (pwm_val.data[1]>0)
    {
      key_auto_nav = 'k';       // Left turn on the spot
    }
    else
    {
      key_auto_nav = 'l';       // Right turn on the spot
    }
  }
  else if ((pwm_val.data[0] * pwm_val.data[1]) > 0) // Check for Differential Drive mode velocities
  {
    pwm_l = pwm_val.data[0];    // Left motor  PWM value
    pwm_r = pwm_val.data[1];    // Right motor PWM value
    key_auto_nav = 'd';         // Differential Drive mode
  }
  else
  {
    
    key_auto_nav = ' ';         // No velocity commands. Bot Stop!!
  }
}

// Declare ROS Subscriber
ros::Subscriber<std_msgs::String> teleop_sub("teleop_key", &teleop_cb );    // Subscribes to Teleop commands
ros::Subscriber<std_msgs::Int16MultiArray> auto_nav("motor_pwm", &auto_nav_cb);  // Subscribes to PWM values from autonomous navigation planner

bool Encoder_Right_Wheel_Phase_B()
{
  return PIND & B00000100;
}

bool Encoder_Left_Wheel_Phase_B()
{
  return PIND & B00001000;
}

// Interrupt Service Routine for Right Wheel Encoder
void Encoder_Right_Wheel_Phase_A_ISR()
{
  unsigned long time_r = millis();
  double vel_r = 46.5 / (time_r - prev_time_r);   // Right wheel velocity in rad/s
  prev_time_r = time_r;
  
  cumulative_ticks++;           // Cumulative tick count for fixed step spot rotations

  if (Encoder_Right_Wheel_Phase_B()) // Forward direction
  {
    ticks_right++;
    vel.data[1] = vel_r;
  }
  else                          // Backward direction
  {
    ticks_right--;
    vel.data[1] = -vel_r;
  }
  ticks.data[1] = ticks_right;
}

// Interrupt Service Routine for Left Wheel Encoder
void Encoder_Left_Wheel_Phase_A_ISR()
{
  unsigned long time_l = millis();
  double vel_l = 46.5 / (time_l - prev_time_l);   // Left wheel velocity in rad/s
  prev_time_l = time_l;
  
  cumulative_ticks++;           // Cumulative tick count for fixed step spot rotations
  
  if (Encoder_Left_Wheel_Phase_B()) // Backward direction
  {
    ticks_left--;
    vel.data[0] = -vel_l;
  }
  else                          // Forward direction
  {
    ticks_left++;
    vel.data[0] = vel_l;
  }  
  ticks.data[0] = ticks_left;
}

// Initialize IO in pullep-up input mode
void pullup_input_init(int pin)
{
  pinMode(pin, INPUT);
  digitalWrite(pin, HIGH);
}

void setup()
{
  nh.initNode();            //Initialize ROS Node

  // Subscribe to ROS Topics
  nh.subscribe(teleop_sub);
  nh.subscribe(auto_nav);

  // Initialize Array type messages in ROS
  ticks.layout.dim = (std_msgs::MultiArrayDimension *)malloc(sizeof(std_msgs::MultiArrayDimension)*2);
  ticks.layout.dim[0].label = "encoder_ticks";
  ticks.layout.dim[0].size = 2;
  ticks.layout.dim[0].stride = 1;
  ticks.layout.data_offset = 0;
  ticks.data = (int *)malloc(sizeof(int)*8);
  ticks.data_length = 2;

  vel.layout.dim = (std_msgs::MultiArrayDimension *)malloc(sizeof(std_msgs::MultiArrayDimension)*2);
  vel.layout.dim[0].label = "wheel_velocities";
  vel.layout.dim[0].size = 2;
  vel.layout.dim[0].stride = 1;
  vel.layout.data_offset = 0;
  vel.data = (float *)malloc(sizeof(float)*8);
  vel.data_length = 2;

  // Initialize all IOs related to Robot Motion Controllers
  bot_init();

  // Initialize Encoder IO pins
  pullup_input_init(encoder_0_PinA);
  pullup_input_init(encoder_0_PinB);
  pullup_input_init(encoder_1_PinA);
  pullup_input_init(encoder_1_PinB);

  // Attach interrupt for Falling Edge
  attachInterrupt(digitalPinToInterrupt(18), Encoder_Right_Wheel_Phase_A_ISR, FALLING);
  attachInterrupt(digitalPinToInterrupt(19), Encoder_Left_Wheel_Phase_A_ISR, FALLING);

  // Advertise ROS Topics to be Published by Atmega
  nh.advertise(encoder_ticks);
  nh.advertise(wheel_velocities);
}

void loop()
{
  wheel_velocities.publish(&vel);   // Publish wheel angular velocities

  // Callback function execution control
  nh.spinOnce();
  delay(30);

  // Teleoperation Mode
  switch (key_teleop)             // Key Assigned by Tele-operator
  {
    case 'w':                     // Move Forward
      bot_forward(pwm, pwm);
      if (abs(ticks_right) > 10)  // Publish every 10 ticks
      {
        publish_encoder();
        initialize_ticks_to_zero();
      }
      break;

    case 's':                     // Move Backward
      bot_backward(pwm);
      if (abs(ticks_right) > 10)  // Publish every 10 ticks
      {
        publish_encoder();
        initialize_ticks_to_zero();
      }
      break;

    case 'q':                       // Increment Turn step by 15 degrees
      turn_angle = turn_angle + 15;
      key_teleop = ' ';
      break;
    
    case 'e':                       // Set Turn step to 15 degrees
      turn_angle = 15;
      key_teleop = ' ';
      break;

    case 'l':                       // Spot Turn Right
      initialize_ticks_to_zero();
      rotate_anti_clk(turn_angle);
      key_teleop = ' ';
      break;

    case 'k':                       // Spot Turn Left
      initialize_ticks_to_zero();
      rotate_clk(turn_angle);
      key_teleop = ' ';
      break;
 
    case 'c':                       // Reset all variables
      initialize_ticks_to_zero();
      cumulative_ticks = 0;
      turn_angle = 45;
      pwm = 80;
      pwm_spot = 40;
      break;

    case 'a':                       // Enable Autonomous Mode
      autonav = 1;
      break;
      
    case 't':             
      autonav = 0;                  // Disbale Autonomous Mode
      break;

    default:
      if (autonav == 0)
      {
        vel.data[0] = 0.0;            // Set Published Velocity to zero when no teleop commands received
        vel.data[1] = 0.0;
        botKill();
      }
  }
  
  // Autonomous Navigation Mode
  if (autonav == 1)
  {
    switch (key_auto_nav)             // Key Assigned by Auto Nav Callback
    {
      case 'w':                       // Diff Drive Mode
        bot_forward(pwm_r, pwm_l);
        if (abs(ticks_right) > 10)
        {
          publish_encoder();
          initialize_ticks_to_zero();
          }
        break;

      case 'l':                       // Spot Turn Left
        bot_spot_left(pwm_spot);
        if (abs(ticks_right) > 10)
        {
          publish_encoder();
          initialize_ticks_to_zero();
          }
        break;
  
      case 'k':                       // Spot Turn Right
        bot_spot_right(pwm_spot);
        if (abs(ticks_right) > 10)
        {
          publish_encoder();
          initialize_ticks_to_zero();
          }
        break;
  
      default:
        vel.data[0] = 0.0;
        vel.data[1] = 0.0;
        botKill();
    }
  }
}

// Function Declarations

// Reset Ticks to zero
void initialize_ticks_to_zero()
{
  ticks_left = ticks_right = 0;
}

// Publish encoder ticks
void publish_encoder()
{
  encoder_ticks.publish(&ticks);
}

void bot_forward(int pwml, int pwmr)
{
  MOTORLA = 1;
  MOTORLB = 0;
  MOTORRA = 0;
  MOTORRB = 1;
  set_pwm3b(pwml);
  set_pwm3a(pwmr);
}

void bot_backward(int pwm)
{
  MOTORLA = 0;
  MOTORLB = 1;
  MOTORRA = 1;
  MOTORRB = 0;
  set_pwm3b(pwm);
  set_pwm3a(pwm);
}

void bot_spot_left(int pwm)
{
  MOTORLA = 0;
  MOTORLB = 1;
  MOTORBA = 1;
  MOTORBB = 0;
  MOTORRA = 0;
  MOTORRB = 1;
  MOTORFA = 0;
  MOTORFB = 1;
  set_pwm1a(pwm);
  set_pwm1b(pwm);
  set_pwm3a(pwm);
  set_pwm3b(pwm);
}

void bot_spot_right(int pwm)
{
  MOTORLA = 1;
  MOTORLB = 0;
  MOTORBA = 0;
  MOTORBB = 1;
  MOTORRA = 1;
  MOTORRB = 0;
  MOTORFA = 1;
  MOTORFB = 0;
  set_pwm1a(pwm);
  set_pwm1b(pwm);
  set_pwm3a(pwm);
  set_pwm3b(pwm);
}

// Step Turns 
void rotate_clk(int turn_angle)
{
  cumulative_ticks = 0;
  
  while(cumulative_ticks < (53 * turn_angle / 15))
  { 
    bot_spot_right(pwm_spot);
    if (abs(ticks_right)>=20)
      {
        publish_encoder();
        initialize_ticks_to_zero();
      }
    wheel_velocities.publish(&vel);
  }
  publish_encoder();
  initialize_ticks_to_zero();
}

void rotate_anti_clk(int turn_angle)
{
  cumulative_ticks = 0;
  
  while (cumulative_ticks < (53 * turn_angle / 15))
  {
    bot_spot_left(pwm_spot);
    if (abs(ticks_right) >= 20)
      {
        publish_encoder();
        initialize_ticks_to_zero();
      }
    wheel_velocities.publish(&vel);
  }
  publish_encoder();
  initialize_ticks_to_zero();
}

void bot_init()
{
  DDRC = 0xFF;
  PORTC = 0x00;
  DDRB |= (1 << PB4) | (1 << PB5) | (1 << PB6);
  DDRD |= (1 << PD0) | (1 << PD1);
  DDRE |= (1 << PE3) | (1 << PE4);
  pwm1_init();
  pwm3_init();
}

void botKill()
{
  set_pwm1a(0);
  set_pwm1b(0);
  set_pwm3a(0);
  set_pwm3b(0);
  PORTC = 0xFF;
  PORTD &= 0xFC;
}
