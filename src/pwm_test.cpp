#include <ros/ros.h>
#include <pigpiod_if2.h>

const int k_control_cycle = 10;
const int k_pins = 8;
const int k_pwm_range = 1000;
const int k_pwn_frequency = 20000;
int pin_nums[k_pins] = {17,4,22,27,6,5,19,13};
int pi_num;

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
    set_mode(pi_num, pin_nums[i], PI_OUTPUT);
    set_PWM_range(pi_num, pin_nums[i], k_pwm_range);
    set_PWM_frequency(pi_num, pin_nums[i], k_pwn_frequency);
  }
}

void SetPwmDutycycle(int rate)
{
  if(rate < 0 || rate > k_pwm_range)
  {
    ROS_WARN("Invalid Dutycycle.");
    for (int i = 0; i < k_pins; i++) set_PWM_dutycycle(pi_num, pin_nums[i], 0);
  }

  for (int i = 0; i < k_pins; i++) set_PWM_dutycycle(pi_num, pin_nums[i], rate);
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "pwm_test");
  ros::NodeHandle nh;
  ros::Rate loop_rate(k_control_cycle);

  if(!CheckPigpio()) return -1;
  InitPwm();

  while (ros::ok())
  {
    SetPwmDutycycle(100);
    loop_rate.sleep();
    ros::spinOnce();
  }

  SetPwmDutycycle(0);
  return 0;
}
