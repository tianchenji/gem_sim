#include "ros/ros.h"
#include "std_msgs/String.h"
#include "std_msgs/Float64.h"

#include <sstream>

int main(int argc, char **argv)
{
  ros::init(argc, argv, "driving_command_circle");

  ros::NodeHandle n;

  ros::Publisher left_steering_pub = n.advertise<std_msgs::Float64>("/polaris/front_left_steering_position_controller/command", 1000);
  ros::Publisher right_steering_pub = n.advertise<std_msgs::Float64>("/polaris/front_right_steering_position_controller/command", 1000);
  ros::Publisher front_left_wheel_pub = n.advertise<std_msgs::Float64>("/polaris/front_left_wheel_effort_controller/command", 1000);
  ros::Publisher front_right_wheel_pub = n.advertise<std_msgs::Float64>("/polaris/front_right_wheel_effort_controller/command", 1000);
  ros::Publisher rear_left_wheel_pub = n.advertise<std_msgs::Float64>("/polaris/rear_left_wheel_effort_controller/command", 1000);
  ros::Publisher rear_right_wheel_pub = n.advertise<std_msgs::Float64>("/polaris/rear_right_wheel_effort_controller/command", 1000);

  ros::Rate loop_rate(10);

  while (ros::ok())
  {
    std_msgs::Float64 steering;
    std_msgs::Float64 front_acceleration;
    std_msgs::Float64 rear_acceleration;

    steering.data = 0.3;
    front_acceleration.data = 0.0;
    rear_acceleration.data = 7.0;

    ROS_INFO("The steering command is : %f", steering.data);
    ROS_INFO("The acceleration command is: %f", rear_acceleration.data);

    left_steering_pub.publish(steering);
    right_steering_pub.publish(steering);
    front_left_wheel_pub.publish(front_acceleration);
    front_right_wheel_pub.publish(front_acceleration);
    rear_left_wheel_pub.publish(rear_acceleration);
    rear_right_wheel_pub.publish(rear_acceleration);

    ros::spinOnce();

    loop_rate.sleep();
  }


  return 0;
}
