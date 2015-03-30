#ifndef CARL_MOVEIT_H_
#define CARL_MOVEIT_H_

#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>
#include <actionlib/client/simple_action_client.h>
#include <carl_moveit/ArmAction.h>
#include <carl_moveit/CartesianPath.h>
#include <carl_moveit/MoveToJointPoseAction.h>
#include <carl_moveit/MoveToPoseAction.h>
#include <carl_moveit/PickupAction.h>
#include <rail_manipulation_msgs/LiftAction.h>
#include <rail_manipulation_msgs/GripperAction.h>
#include <std_srvs/Empty.h>
#include <tf/transform_listener.h>
#include <wpi_jaco_msgs/AngularCommand.h>
#include <wpi_jaco_msgs/GetAngularPosition.h>
#include <wpi_jaco_msgs/GetCartesianPosition.h>

#define NUM_JACO_JOINTS 6
#define MAX_HOME_ATTEMPTS 3

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
  ros::ServiceClient cartesianPathClient;
  ros::ServiceClient jacoPosClient;

  actionlib::SimpleActionClient<carl_moveit::MoveToJointPoseAction> moveToJointPoseClient;
  actionlib::SimpleActionClient<carl_moveit::MoveToPoseAction> moveToPoseClient;
  actionlib::SimpleActionClient<rail_manipulation_msgs::GripperAction> gripperClient;
  actionlib::SimpleActionClient<rail_manipulation_msgs::LiftAction> liftClient;
  actionlib::SimpleActionServer<rail_manipulation_msgs::LiftAction> liftServer;
  actionlib::SimpleActionServer<carl_moveit::ArmAction> armServer;
  actionlib::SimpleActionServer<carl_moveit::PickupAction> pickupServer;

  tf::TransformListener tfListener;

  std::vector<float> homePosition;
  std::vector<float> defaultRetractPosition;

  /**
  * \brief Move arm to a defined position with obstacle avoidance
  *
  * Action server to execute pre-defined arm actions, such as moving to the ready (JACO home) position, or moving
  * to a retracted position.
  *
  * @param goal Specification of the arm action to be executed.
  */
  void executeArmAction(const carl_moveit::ArmGoalConstPtr &goal);

  void executePickup(const carl_moveit::PickupGoalConstPtr &goal);

  void liftArm(const rail_manipulation_msgs::LiftGoalConstPtr &goal);

  bool isArmRetracted(const std::vector<float> &retractPos);
};

#endif
