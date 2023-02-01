# drone_tutorial
남부창의마루 수업을 위한 비행로봇 예제


간단한 bashrc 설정
```
export ROS_WS=$HOME/catkin_ws #define workspace
export ROS_MASTER_URI=http://localhost:11311 #define ROS_MASTER
export ROS_HOSTNAME=localhost #define ROS_HOSTNAME
export ROSCONSOLE_FORMAT='[${node}, ${walltime:%H:%M:%S.%f}]  ${message}'
#change console format

source /opt/ros/noetic/setup.bash #set ROS setting
source $ROS_WS/devel/setup.bash #set workspace setting

alias eb='vi ~/.bashrc' #edit bashrc
alias sb='source ~/.bashrc' #source bashrc
alias cw='cd $ROS_WS' #change directory as workspace
alias cs='cd $ROS_WS/src' #change directory as workspace/src
alias cm='catkin_make -C $ROS_WS' #build workspace
alias rs='rm -rf $ROS_WS/devel/* && rm -rf $ROS_WS/build/*' #remove devel & build
alias rl='rosclean purge -y' #clean ROS log
```

Publisher Node 코드
```
#include <ros/ros.h>
#include <std_msgs/Int16.h>
int main(int argc, char** argv)
{
  ros::init(argc, argv, "pub_node");
  ros::NodeHandle nh;
  ros::Publisher pub_number = nh.advertise<std_msgs::Int16>("/test/topic", 10, true);
  ros::Rate loop_rate(1);
  std_msgs::Int16 count;
  while(ros::ok())
  {
    pub_number.publish(count);
    ROS_INFO("pub %d", count.data);
    count.data++;
    ros::spinOnce();
    loop_rate.sleep();
  }
}
```

Subscriber Node 코드
```
#include <ros/ros.h>
#include <std_msgs/Int16.h>
void NumberCallback(const std_msgs::Int16 &msg)
{
  ROS_INFO("sub %d", msg.data);
}
int main(int argc, char** argv)
{
  ros::init(argc, argv, "sub_node");
  ros::NodeHandle nh;
  ros::Subscriber sub_number = nh.subscribe("/test/topic", 10, NumberCallback);
  ros::spin();
}
```

CMakeLists.txt에 새 노드 등록하기
```
add_executable(pub_node src/pub_node.cpp)
add_dependencies(pub_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(pub_node ${catkin_LIBRARIES})
add_executable(sub_node src/sub_node.cpp)
add_dependencies(sub_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(sub_node ${catkin_LIBRARIES})
```
