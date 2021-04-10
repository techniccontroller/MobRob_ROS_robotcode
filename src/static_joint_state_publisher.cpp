/**
 * static_joint_state_publisher.cpp
 * 
 * Send static joint state of robot
 * 
 * Created by techniccontroller on 10.04.2021
 * 
*/

#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <string>

int main(int argc, char** argv)
{
  ros::init(argc, argv, "static_joint_state_publisher");
  ros::NodeHandle n;
  ros::Publisher joint_pub = n.advertise<sensor_msgs::JointState>("joint_states", 1);
  ros::Rate loop_rate(30);

  // message declarations
  sensor_msgs::JointState joint_state;

  while (ros::ok())
  {
    // update joint_state
    joint_state.header.stamp = ros::Time::now();
    joint_state.name.resize(7);
    joint_state.position.resize(7);
    joint_state.name[0] = "gripper_stand_to_gripper_joint";
    joint_state.position[0] = 0;
    joint_state.name[1] = "right_front_wheel_joint";
    joint_state.position[1] = 0;
    joint_state.name[2] = "left_front_wheel_joint";
    joint_state.position[2] = 0;
    joint_state.name[3] = "right_back_wheel_joint";
    joint_state.position[3] = 0;
    joint_state.name[4] = "left_back_wheel_joint";
    joint_state.position[4] = 0;
    joint_state.name[5] = "gripper_to_right_finger_joint";
    joint_state.position[5] = -0.05;
    joint_state.name[6] = "gripper_to_left_finger_joint";
    joint_state.position[6] = 0.05;

    // send the joint state
    joint_pub.publish(joint_state);

    // This will adjust as needed per iteration
    loop_rate.sleep();
  }

  return 0;
}