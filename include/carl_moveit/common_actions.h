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

  /**
  * \brief Constructor
  */
  CommonActions();

private:
  ros::NodeHandle n;
  ros::Publisher angularCmdPublisher;

  ros::ServiceClient eraseTrajectoriesClient;
  ros::ServiceClient jacoPosClient;

  actionlib::SimpleActionClient<carl_moveit::MoveToJointPoseAction> moveToJointPoseClient;
  actionlib::SimpleActionServer<wpi_jaco_msgs::HomeArmAction> readyArmServer;

  std::vector<float> homePosition;

  /**
  * \brief Move arm to the home position with obstacle avoidance
  *
  * Action server to move the arm to the ready (JACO home) position, optionally followed by a move to a retracted
  * position.
  *
  * @param goal Ready/retract action goal.
  */
  void readyArm(const wpi_jaco_msgs::HomeArmGoalConstPtr &goal);
};

#endif
