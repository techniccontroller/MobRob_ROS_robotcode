/**
 * static_joint_state_publisher.cpp
 * 
 * Send static joint state of robot
 * 
 * Created by techniccontroller on 10.04.2021
 * Updated by techniccontroller on 02.02.2023
 * 
*/

#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <myrobot_model/AttinyCommand.h>
#include <string>
#include <math.h>

int main(int argc, char** argv)
{
  ros::init(argc, argv, "static_joint_state_publisher");
  ros::NodeHandle n;
  ros::Publisher joint_pub = n.advertise<sensor_msgs::JointState>("joint_states", 1);
  ros::ServiceClient attiny_client = n.serviceClient<myrobot_model::AttinyCommand>("attiny_command");
  ros::Rate loop_rate(1);

  // message declarations
  sensor_msgs::JointState joint_state;
  myrobot_model::AttinyCommand srv_get_gripper_pos;
  myrobot_model::AttinyCommand srv_get_vertical_pos;
  myrobot_model::AttinyCommand srv_get_servo_pos;
  srv_get_gripper_pos.request.input = "gr_gp(1)\n";
  srv_get_vertical_pos.request.input = "vt_gp(1)\n";
  srv_get_servo_pos.request.input = "sv_gp(1)\n";

  while (ros::ok())
  {
    int gripper_pos = 0;
    int vertical_pos = 142;
    float servo_pos = M_PI/2.0;
    // get gripper position
    if (attiny_client.call(srv_get_gripper_pos))
    {
      gripper_pos = std::stoi(srv_get_gripper_pos.response.output);
      ROS_INFO("Gripper position: %d mm", gripper_pos);
    }
    else
    {
      ROS_ERROR("Failed to call service get_gripper_pos");
    }

    // get vertical position
    if (attiny_client.call(srv_get_vertical_pos))
    {
      vertical_pos = std::stoi(srv_get_vertical_pos.response.output);
      ROS_INFO("Vertical position: %d mm", vertical_pos);
    }
    else
    {
      ROS_ERROR("Failed to call service get_vertical_pos");
    }

    // get servo position
    if (attiny_client.call(srv_get_servo_pos))
    {
      servo_pos = std::stoi(srv_get_servo_pos.response.output) / 180.0 * M_PI;
      ROS_INFO("Servo position: %d deg (%f rad)", std::stoi(srv_get_servo_pos.response.output), servo_pos);
    }
    else
    {
      ROS_ERROR("Failed to call service get_servo_pos");
    }

    // update joint_state
    joint_state.header.stamp = ros::Time::now();
    joint_state.name.resize(8);
    joint_state.position.resize(8);
    joint_state.name[0] = "gripper_stand_to_gripper_joint";
    joint_state.position[0] = -(142-vertical_pos)/1000.0;
    joint_state.name[1] = "right_front_wheel_joint";
    joint_state.position[1] = 0;
    joint_state.name[2] = "left_front_wheel_joint";
    joint_state.position[2] = 0;
    joint_state.name[3] = "right_back_wheel_joint";
    joint_state.position[3] = 0;
    joint_state.name[4] = "left_back_wheel_joint";
    joint_state.position[4] = 0;
    joint_state.name[5] = "gripper_to_right_finger_joint";
    joint_state.position[5] = -gripper_pos/2000.0;
    joint_state.name[6] = "gripper_to_left_finger_joint";
    joint_state.position[6] = gripper_pos/2000.0;
    joint_state.name[7] = "camera_servo_to_camera_base_joint";
    joint_state.position[7] = servo_pos;

    // send the joint state
    joint_pub.publish(joint_state);

    // This will adjust as needed per iteration
    loop_rate.sleep();
  }

  return 0;
}