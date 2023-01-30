#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <pigpiod_if2.h>

#define constrain(amt, low, high) ((amt) < (low) ? (low) : ((amt) > (high) ? (high) : (amt)))
const int k_control_cycle = 10;
const int k_pins = 8;
const int k_pwm_range = 1000;
const int k_pwn_frequency = 20000;
const int k_pin_nums[k_pins] = {17,4,22,27,6,5,19,13};
int pi_num;
int target_pwm[k_pins] = {};

void TeleopCallback(const geometry_msgs::Twist &msg)
{
  double X_pos = (msg.linear.x >= 0) ? msg.linear.x : 0;
  double Y_pos = (msg.linear.y >= 0) ? msg.linear.y : 0;
  double Z_pos = (msg.linear.z >= 0) ? msg.linear.z : 0;
  double W_pos = (msg.angular.x > 0) ? constrain(msg.angular.x,15,250) : 0;
  double X_neg = (msg.linear.x < 0) ? msg.linear.x : 0;
  double Y_neg = (msg.linear.y < 0) ? msg.linear.y : 0;
  double W_neg = (msg.angular.x < 0) ? constrain(msg.angular.x,-250,-15) : 0;

  double vh = sqrt(2) * 0.5;

  target_pwm[0] = 0 + 0 + Z_pos + 0;
  target_pwm[1] = -X_neg * vh + Y_pos + 0 + W_pos;
  target_pwm[2] = -X_neg - Y_neg + 0 - W_neg;
  target_pwm[3] = X_pos - Y_neg * vh + 0 + W_pos;
  target_pwm[4] = 0 + 0 + Z_pos + 0;
  target_pwm[5] = -X_neg * vh + Y_pos + 0 - W_neg;
  target_pwm[6] = -X_neg - Y_neg + 0 + W_pos;
  target_pwm[7] = X_pos - Y_neg * vh + 0 - W_neg;
}

bool CheckPigpio()
{
  pi_num = pigpio_start(NULL, NULL);

  if (pi_num < 0)
  {
    ROS_ERROR("PI number is %d",pi_num);
    ROS_ERROR("PIGPIO connection failed.");
    return false;
  }

  ROS_INFO("Setup Finished.");
  return true;
}

void InitPwm()
{
  for (int i = 0; i < k_pins; i++)
  {
    set_mode(pi_num, k_pin_nums[i], PI_OUTPUT);
    set_PWM_range(pi_num, k_pin_nums[i], k_pwm_range);
    set_PWM_frequency(pi_num, k_pin_nums[i], k_pwn_frequency);
  }
}

void SetPwmDutycycle(int rate)
{
  if(rate < 0 || rate > k_pwm_range)
  {
    ROS_WARN("Invalid Dutycycle.");
    for (int i = 0; i < k_pins; i++) set_PWM_dutycycle(pi_num, k_pin_nums[i], 0);
  }

  for (int i = 0; i < k_pins; i++) set_PWM_dutycycle(pi_num, k_pin_nums[i], rate);
}

void SetPwmDutycycle()
{
  for (int i = 0; i < k_pins; i++)
  {
    if(target_pwm[i] < 0 || target_pwm[i] > k_pwm_range)
    {
      ROS_WARN("Invalid Dutycycle.");
      set_PWM_dutycycle(pi_num, k_pin_nums[i], 0);
    }
    set_PWM_dutycycle(pi_num, k_pin_nums[i], target_pwm[i]);
  }
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "drone_controller");
  ros::NodeHandle nh;
  ros::Rate loop_rate(k_control_cycle);
  ros::Subscriber sub_teleop = nh.subscribe("/drone_teleop_pwm", 5, TeleopCallback);

  if(!CheckPigpio()) return -1;
  InitPwm();

  while (ros::ok())
  {
    SetPwmDutycycle();
    loop_rate.sleep();
    ros::spinOnce();
  }

  SetPwmDutycycle(0);
  return 0;
}
