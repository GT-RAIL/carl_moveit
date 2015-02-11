#ifndef CARL_MOVEIT_H_
#define CARL_MOVEIT_H_

#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>
#include <actionlib/client/simple_action_client.h>
#include <carl_moveit/MoveToJointPoseAction.h>
#include <std_srvs/Empty.h>
#include <wpi_jaco_msgs/GetAngularPosition.h>
#include <wpi_jaco_msgs/HomeArmAction.h>

#define NUM_JACO_JOINTS 6

class CommonActions
{

public:

  CommonActions();

private:
  ros::NodeHandle n;
  ros::Publisher angularCmdPublisher;

  ros::ServiceClient eraseTrajectoriesClient;
  ros::ServiceClient jacoPosClient;

  actionlib::SimpleActionClient<carl_moveit::MoveToJointPoseAction> moveToJointPoseClient;
  actionlib::SimpleActionServer<wpi_jaco_msgs::HomeArmAction> readyArmServer;

  std::vector<float> homePosition;

  void readyArm(const wpi_jaco_msgs::HomeArmGoalConstPtr &goal);
};

#endif
