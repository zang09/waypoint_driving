#include <SoftwareSerial.h> 
#include <Servo.h> 
#include <math.h> 

#include <ros.h>
#include <geometry_msgs/Twist.h>


#define DC2_A_PIN      4 
#define DC2_B_PIN      5 
#define DC1_A_PIN      6 
#define DC1_B_PIN      7 

#define VESC1_PIN      9 
#define VESC2_PIN      10 
#define VESC3_PIN      11 
#define VESC4_PIN      12 
#define DC_INHIBIT_PIN 22 

#define STOP           0 
#define FRONT_LEFT     1 
#define FRONT_RIGHT    2 
#define BACK_RIGHT     3 
#define BACK_LEFT      4 
#define LEFT           5 
#define RIGHT          6 
#define UP             7 
#define DOWN           8 

#define ROS_BAUD       115200
#define SERIAL_MONITOR 115200
#define BLUTETOOTH      38400


Servo vesc1_, vesc2_, vesc3_, vesc4_;
double bldc1_val_, bldc2_val_, bldc3_val_, bldc4_val_; 

const double gain_          = 0.4; 
const int    ctr_val_       = 72; 
const int    bldc_max_val_  = 50; 
const int    dc_max_val_    = 255; 
const int    max_filter_sz_ = 50; 

double pwr_array_[max_filter_sz_] = {0.0,}; 
double pwr_val_       = 0.0; 
double pre_speed_     = 0.0;
double cur_speed_     = 0.0;
double pre_angle_val_ = 0.0; 
double angle_val_     = 0.0; 
double pre_vel_l_     = 0.0;
double pre_vel_r_     = 0.0;
double auto_left_vel_ = 0.0;
double auto_right_vel_= 0.0;
int    pre_max_val_   = 0; 
int    cur_max_val_   = 0; 

String rx_string_        = "";
bool   rx_data_start_    = false; 
bool   rx_func_start_    = false; 
 
bool   auto_mode_        = false;
bool   emergency_mode_   = false;

bool   bldc_turn_mode_   = false;
int    bldc_motor_state_ = STOP; 
int    dc_motor_state_   = STOP; 

int    pre_filter_sz_    = 0;
int    cur_filter_sz_    = 0; 
int    progress_bar_     = 0;

void veloCallback(const geometry_msgs::Twist &vel_msg);

ros::NodeHandle nh_;
ros::Subscriber<geometry_msgs::Twist> vel_sub_("argos_mr/motor_vel", veloCallback);

geometry_msgs::Twist motor_vel_;
ros::Publisher vel_check_("check/vel", &motor_vel_);

void setup() 
{
  //ROS INIT
  nh_.getHardware()->setBaud(ROS_BAUD);
  nh_.initNode();
  nh_.subscribe(vel_sub_);
  nh_.advertise(vel_check_);

  //Serial 
  Serial.begin(SERIAL_MONITOR); 
  Serial2.begin(BLUTETOOTH); 

  //Servo 
  pinMode(DC1_A_PIN,OUTPUT);     
  pinMode(DC1_B_PIN,OUTPUT); 
  pinMode(DC2_A_PIN,OUTPUT); 
  pinMode(DC2_B_PIN,OUTPUT); 
  pinMode(DC_INHIBIT_PIN,OUTPUT); // (INHIBIT, 긴급정지)를 1로 맞춰야 드라이버가 활성화된다. 
  digitalWrite(DC_INHIBIT_PIN,HIGH); 

  //BLDC 
  vesc1_.attach(VESC1_PIN); 
  vesc1_.attach(VESC2_PIN); 
  vesc1_.attach(VESC3_PIN); 
  vesc1_.attach(VESC4_PIN); 

  //init state
  progress_bar_  = 1;
  cur_max_val_   = (progress_bar_+1)*10; 
  cur_filter_sz_ = 10; 
  
  delay(2500); 
}

void loop() 
{
  if (Serial2.available() > 0)  
  { 
    char c = Serial2.read(); 
   
    if (!rx_data_start_)  
    { 
      if (c == '#') rx_data_start_ = true; 
    } 
    else  
    { 
      if (c == '\n')  
      { 
        dataParsing(rx_string_); 
        rx_data_start_ = false; 
        rx_string_ = ""; 

        setBLDCMotorManual(); 

        writeBLDCMotor(emergency_mode_); 

        writeDCMotor(); 
      } 
      else  
      { 
        rx_string_ += c; 
      } 
    } 

    if (!rx_func_start_)  
    { 
      if (c == '*')  
        rx_func_start_ = true; 
    } 
    else  
    { 
      funcParsing(c); 
    } 
  } 

//  if(auto_mode_) 
//  {
//    setBLDCMotorAuto();
//
//    writeBLDCMotor(emergency_mode_); 
//
//    writeDCMotor(); 
//
//    delay(10);
//  } 
  
  nh_.spinOnce(); //ROS callback
}

void veloCallback(const geometry_msgs::Twist &vel_msg)
{
  auto_left_vel_ = vel_msg.linear.x;
  auto_right_vel_ = vel_msg.linear.y;
}

void dataParsing(String str)  
{ 
  String ang = str.substring(0, str.indexOf(',')); 
  String pwr = str.substring(str.indexOf(',') + 1); 

  angle_val_ = ang.toDouble() - 90.0; 
  pwr_val_   = pwr.toDouble(); 
  
  if (pwr_val_ == 0.0)  
    angle_val_ = pre_angle_val_; 
  else
    pre_angle_val_ = angle_val_; 
  
  if(bldc_turn_mode_ == false)  
  { 
    if      (angle_val_ < 0.0 )  bldc_motor_state_ = FRONT_LEFT; 
    else if (angle_val_ < 90.0)  bldc_motor_state_ = FRONT_RIGHT; 
    else if (angle_val_ < 180.0) bldc_motor_state_ = BACK_RIGHT; 
    else                         bldc_motor_state_ = BACK_LEFT;       
  }   
} 

void funcParsing(char c)  
{   
  Serial.print("C: "); Serial.println(c); 
  if (c == '1')  
  {  
    progress_bar_ = 1;
    cur_max_val_  = (progress_bar_+1)*10;   //20
  } 
  else if (c == '2')  
  { 
    progress_bar_ = 2;
    cur_max_val_  = (progress_bar_+1.5)*10; //35
  } 
  else if (c == '3')  
  { 
    progress_bar_ = 3;
    cur_max_val_  = (progress_bar_+2)*10;   //50
  } 
  else if (c == 'm') {
    auto_mode_ = false;
  }
  else if (c == 'a') {
    auto_mode_ = true;
  }
  else if (c == 'u') {
    pre_max_val_    = cur_max_val_; 
    cur_max_val_    = 40;  //static
    dc_motor_state_ = UP; 
  }
  else if (c == 'd') {
    pre_max_val_    = cur_max_val_; 
    cur_max_val_    = 40;  //static
    dc_motor_state_ = DOWN;
  } 
  else if (c == 'l')  
  {     
    pre_max_val_      = cur_max_val_; 
    cur_max_val_      = 15;  //static
    bldc_motor_state_ = LEFT; 
    bldc_turn_mode_   = true; 
  } 
  else if (c == 'r')  
  { 
    pre_max_val_      = cur_max_val_; 
    cur_max_val_      = 15;  //static
    bldc_motor_state_ = RIGHT; 
    bldc_turn_mode_   = true; 
  } 
  else if (c == 's')  
  { 
    cur_max_val_    = pre_max_val_; 
    dc_motor_state_ = STOP; 
    bldc_turn_mode_ = false; 
    emergency_mode_ = false; 
  } 
  else if (c == 'f') // brake 
  { 
    pre_max_val_    = cur_max_val_; 
    emergency_mode_ = true;
  } 
  else if (c == 'g')  
  { 
    String s_value(progress_bar_);
    char value = s_value.charAt(0);
    Serial2.write(value);
  } 
  
  rx_func_start_ = false; 
} 

void setBLDCMotorManual() 
{   
  double pwr_value, sin_angle, alpha; 
  double vel_l, vel_r; 

  pwr_value = veloFilter();  
  sin_angle = sin(angle_val_ * PI / 180.0); 

  switch (bldc_motor_state_)  
  { 
    case FRONT_LEFT: 
      alpha = (1.0 + gain_ * sin_angle) / (1.0 - gain_ * sin_angle); 
      vel_l = pwr_value * alpha; 
      vel_r = pwr_value; 
      break; 

    case FRONT_RIGHT: 
      alpha = (1.0 - gain_ * sin_angle) / (1.0 + gain_ * sin_angle); 
      vel_l = pwr_value; 
      vel_r = pwr_value * alpha; 
      break; 

    case BACK_RIGHT: 
      alpha = (1.0 - gain_ * sin_angle) / (1.0 + gain_ * sin_angle); 
      vel_l = -pwr_value; 
      vel_r = -pwr_value * alpha; 
      break; 

    case BACK_LEFT:   
      alpha = (1.0 + gain_ * sin_angle) / (1.0 - gain_ * sin_angle); 
      vel_l = -pwr_value * alpha; 
      vel_r = -pwr_value; 
      break;   

    case LEFT: 
      vel_l = -100.0; 
      vel_r = 100.0;         
      break;  

    case RIGHT: 
      vel_l = 100.0; 
      vel_r = -100.0; 
      break;   

    case STOP: 
      vel_l = 0; 
      vel_r = 0; 
      break; 
  }   

  // prevent discrete input(test code)
  if((pre_vel_l_*vel_l)<0 && pre_vel_l_!=0 && pwr_val_>=80.0) 
  {
    vel_l = pre_vel_l_;
    vel_r = pre_vel_r_; 
  }

  // change filter size adaptively
  changeFilterSz(pwr_value);  
 
  // set bldc value
  bldc1_val_ = ctr_val_ + (cur_max_val_ * (vel_l/100.0)); 
  bldc2_val_ = ctr_val_ + (cur_max_val_ * (vel_r/100.0)); 
  bldc3_val_ = ctr_val_ + (cur_max_val_ * (vel_r/100.0)); 
  bldc4_val_ = ctr_val_ + (cur_max_val_ * (vel_l/100.0));  

  pre_vel_l_ = vel_l;
  pre_vel_r_ = vel_r;
  pre_speed_ = cur_speed_;  
  pre_filter_sz_ = cur_filter_sz_;
  
  //Serial.print("L: "); Serial.println(bldc1_val_); 
  //Serial.print("R: "); Serial.println(bldc2_val_); 
} 

void setBLDCMotorAuto()
{
  bldc1_val_ = ctr_val_ + (cur_max_val_ * (auto_left_vel_/1.0)); //max: 1.0[m/s]
  bldc2_val_ = ctr_val_ + (cur_max_val_ * (auto_right_vel_/1.0)); 
  bldc3_val_ = ctr_val_ + (cur_max_val_ * (auto_right_vel_/1.0)); 
  bldc4_val_ = ctr_val_ + (cur_max_val_ * (auto_left_vel_/1.0)); 

  //Serial.print("L2: "); Serial.println(bldc1_val_); 
  //Serial.print("R2: "); Serial.println(bldc2_val_); 
}

void writeBLDCMotor(bool mode) 
{   
  if(mode)
  {
    double bldc_val = 0;
    memset(pwr_array_, 0, sizeof(pwr_array_));
    vesc1_.write(bldc_val); 
    vesc1_.write(bldc_val); 
    vesc1_.write(bldc_val); 
    vesc1_.write(bldc_val); 
  }
  else
  {
    vesc1_.write(bldc1_val_); 
    vesc1_.write(bldc2_val_); 
    vesc1_.write(bldc3_val_); 
    vesc1_.write(bldc4_val_); 
  }
} 

void writeDCMotor() 
{ 
  double vel = (double)dc_max_val_*((double)cur_max_val_/(double)bldc_max_val_); 

  switch(dc_motor_state_)  
  { 
    case UP: 
      analogWrite(DC1_A_PIN, vel); 
      analogWrite(DC1_B_PIN, 0); 
      analogWrite(DC2_A_PIN, vel); 
      analogWrite(DC2_B_PIN, 0); 
    break;     

    case DOWN: 
      analogWrite(DC1_A_PIN, 0); 
      analogWrite(DC1_B_PIN, vel); 
      analogWrite(DC2_A_PIN, 0); 
      analogWrite(DC2_B_PIN, vel); 
    break; 

    case STOP: 
      analogWrite(DC1_A_PIN, 0); 
      analogWrite(DC1_B_PIN, 0); 
      analogWrite(DC2_A_PIN, 0); 
      analogWrite(DC2_B_PIN, 0); 
    break;    
  }   
  
  Serial.print("Vel: "); Serial.println(vel); 
} 

double veloFilter()
{
  int vel_filter[cur_filter_sz_] = {0, }; 
  int sum_vel_filter = 0; 
  double sum_pwr = 0.0; 

  // Velocity Filter
  memcpy(pwr_array_, pwr_array_ + 1, sizeof(pwr_array_) - sizeof(double)); 
  pwr_array_[cur_filter_sz_ - 1] = pwr_val_; 

  for (int i = cur_filter_sz_ - 1; i >= 0 ; i--) 
  { 
    vel_filter[i] = 1; 
    sum_pwr += vel_filter[i] * pwr_array_[i]; 
    sum_vel_filter += vel_filter[i]; 
  } 

  double pwr_value = sum_pwr / sum_vel_filter; 

  return pwr_value;
}

void changeFilterSz(double pwr_value)
{
  cur_speed_ = cur_max_val_ * (pwr_value/100.0);
  if(cur_speed_ > pre_speed_)
  {
    if(cur_speed_ <= 20)
    {
      cur_filter_sz_ = 10;      
    }
    else if(cur_speed_ > 20 && cur_speed_ <= 25)
    { 
      cur_filter_sz_ = 20;   
      if(pre_filter_sz_ < cur_filter_sz_)
      {
        for(int i=cur_filter_sz_; i>pre_filter_sz_; i--)
          pwr_array_[i - 1] = pwr_value;      
      }      
    } 
    else if(cur_speed_ > 25 && cur_speed_ <= 40)
    { 
      cur_filter_sz_ = 30;   
      if(pre_filter_sz_ < cur_filter_sz_)
      {
        for(int i=cur_filter_sz_; i>pre_filter_sz_; i--)
          pwr_array_[i - 1] = pwr_value;      
      }      
    } 
    else if(cur_speed_ > 40 && cur_speed_ <= 50)
    { 
      cur_filter_sz_ = 40;   
      if(pre_filter_sz_ < cur_filter_sz_)
      {
        for(int i=cur_filter_sz_; i>pre_filter_sz_; i--)
          pwr_array_[i - 1] = pwr_value;      
      }      
    } 
  }
}
