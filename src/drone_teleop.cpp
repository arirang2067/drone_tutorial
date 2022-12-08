#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <signal.h>
#include <stdio.h>
#include <termios.h>
#include <unistd.h>

#define KEYCODE_RIGHT 0x43
#define KEYCODE_LEFT 0x44
#define KEYCODE_UP 0x41
#define KEYCODE_DOWN 0x42
#define KEYCODE_B 0x62
#define KEYCODE_Q 0x71
#define KEYCODE_S 0x73
#define KEYCODE_Z 0x7a
#define KEYCODE_X 0x78
#define KEYCODE_SPACE 0x20
const int k_control_cycle = 100;

class KeyboardReader
{
public:
  KeyboardReader(): kfd(0)
  {
    // get the console in raw mode
    tcgetattr(kfd, &cooked);
    struct termios raw;
    memcpy(&raw, &cooked, sizeof(struct termios));
    raw.c_lflag &=~ (ICANON | ECHO);
    // Setting a new line, then end of file
    raw.c_cc[VEOL] = 1;
    raw.c_cc[VEOF] = 2;
    tcsetattr(kfd, TCSANOW, &raw);
  }

  void readOne(char * c)
  {
    int rc = read(kfd, c, 1);
    if (rc < 0)
    {
      throw std::runtime_error("read failed");
    }
  }

  void shutdown()
  {
    tcsetattr(kfd, TCSANOW, &cooked);
  }

private:
  int kfd;
  struct termios cooked;
};

KeyboardReader input;

struct DronePWM
{
  double l_x;
  double l_y;
  double l_z;
  double a_w;
  double l_scale;
  double a_scale;
};


class DroneTeleop
{
public:
  DroneTeleop():
  velocity_x(0),
  velocity_y(0),
  velocity_z(0),
  velocity_w(0),
  l_scale_(1000.0),
  a_scale_(1000.0)
  {
    nh_.param("scale_angular", a_scale_, a_scale_);
    nh_.param("scale_linear", l_scale_, l_scale_);

    twist_pub_ = nh_.advertise<geometry_msgs::Twist>("drone_teleop_pwm", 1);
  }

  void keyLoop()
  {
    char c;
    bool dirty=false;
    ros::Rate loop_rate(k_control_cycle);

    puts("Reading from keyboard");
    puts("---------------------------");
    puts("Use keys to move the drone.");
    puts("↑ : foward      ↓ : backward");
    puts("← : left        → : right");
    puts("z : turn left   x : turn right");
    puts("s : stop        b : all");
    puts("q : quit");


    while (ros::ok())
    {
      // get the next event from the keyboard
      try
      {
        input.readOne(&c);
      }
      catch (const std::runtime_error &)
      {
        perror("read():");
        return;
      }
      ROS_DEBUG("value: 0x%02X\n", c);

      switch(c)
      {
        case KEYCODE_LEFT:
          ROS_DEBUG("LEFT");
          velocity_x = 0.0;
          velocity_y += 0.01;
          velocity_z = 0.0;
          velocity_w = 0.0;
          dirty = true;
          break;
        case KEYCODE_RIGHT:
          ROS_DEBUG("RIGHT");
          velocity_x = 0.0;
          velocity_y -= 0.01;
          velocity_z = 0.0;
          velocity_w = 0.0;
          dirty = true;
          break;
        case KEYCODE_UP:
          ROS_DEBUG("FOWARD");
          velocity_x += 0.01;
          velocity_y = 0.0;
          velocity_z = 0.0;
          velocity_w = 0.0;
          dirty = true;
          break;
        case KEYCODE_DOWN:
          ROS_DEBUG("BACKWARD");
          velocity_x -= 0.01;
          velocity_y = 0.0;
          velocity_z = 0.0;
          velocity_w = 0.0;
          dirty = true;
          break;
        case KEYCODE_Z:
          ROS_DEBUG("TURN LEFT");
          velocity_x = 0.0;
          velocity_y = 0.0;
          velocity_z = 0.0;
          velocity_w += 0.01;
          dirty = true;
          break;
        case KEYCODE_X:
          ROS_DEBUG("TURN RIGHT");
          velocity_x = 0.0;
          velocity_y = 0.0;
          velocity_z = 0.0;
          velocity_w -= 0.01;
          dirty = true;
          break;
        case KEYCODE_SPACE:
          ROS_DEBUG("UP");
          velocity_x = 0.0;
          velocity_y = 0.0;
          velocity_z += 0.01;
          velocity_w = 0.0;
          dirty = true;
          break;
        case KEYCODE_B:
          ROS_DEBUG("ALL");
          velocity_x += 0.01;
          velocity_y += 0.01;
          velocity_z += 0.01;
          velocity_w += 0.01;
          dirty = true;
          break;
        case KEYCODE_S:
          ROS_DEBUG("STOP");
          velocity_x = 0.0;
          velocity_y = 0.0;
          velocity_z = 0.0;
          velocity_w = 0.0;
          dirty = true;
          break;
        case KEYCODE_Q:
          ROS_DEBUG("quit");
          return;
      }

      if(velocity_x > 1.0)velocity_x = 1.0;
      else if(velocity_x < -1.0)velocity_x = -1.0;
      if(velocity_y > 1.0)velocity_y = 1.0;
      else if(velocity_y < -1.0)velocity_y = -1.0;
      if(velocity_z > 1.0)velocity_z = 1.0;
      else if(velocity_z < -1.0)velocity_z = -1.0;
      if(velocity_w > 1.0)velocity_w = 1.0;
      else if(velocity_w < -1.0)velocity_w = -1.0;

      geometry_msgs::Twist twist;
      twist.linear.x = l_scale_*velocity_x;
      twist.linear.y = l_scale_*velocity_y;
      twist.linear.z = l_scale_*velocity_z;
      twist.angular.x = a_scale_*velocity_w;

      if(dirty ==true)
      {
        dirty=false;
        twist_pub_.publish(twist);
      }

      loop_rate.sleep();
      ros::spinOnce();
    }
    return;
  }

private:

  ros::NodeHandle nh_;
  double velocity_x, velocity_y, velocity_z, velocity_w, l_scale_, a_scale_;
  ros::Publisher twist_pub_;
};

void quit(int sig)
{
  (void)sig;
  input.shutdown();
  ros::shutdown();
  exit(0);
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "drone_teleop");
  DroneTeleop drone_teleop;

  signal(SIGINT,quit);

  drone_teleop.keyLoop();
  quit(0);

  return 0;
}
