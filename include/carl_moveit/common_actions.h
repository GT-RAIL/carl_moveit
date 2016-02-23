#ifndef COMMON_ACTIONS_H_
#define COMMON_ACTIONS_H_

#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>
#include <actionlib/client/simple_action_client.h>
#include <carl_moveit/WipeSurfaceAction.h>
#include <rail_manipulation_msgs/ArmAction.h>
#include <rail_manipulation_msgs/CartesianPath.h>
#include <rail_manipulation_msgs/GripperAction.h>
#include <rail_manipulation_msgs/LiftAction.h>
#include <rail_manipulation_msgs/MoveToJointPoseAction.h>
#include <rail_manipulation_msgs/MoveToPoseAction.h>
#include <rail_manipulation_msgs/PickupAction.h>
#include <rail_manipulation_msgs/StoreAction.h>
#include <std_srvs/Empty.h>
#include <tf/transform_broadcaster.h>
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
  ros::ServiceClient attachClosestObjectClient;
  ros::ServiceClient detachObjectsClient;

  actionlib::SimpleActionClient<rail_manipulation_msgs::MoveToJointPoseAction> moveToJointPoseClient;
  actionlib::SimpleActionClient<rail_manipulation_msgs::MoveToPoseAction> moveToPoseClient;
  actionlib::SimpleActionClient<rail_manipulation_msgs::GripperAction> gripperClient;
  actionlib::SimpleActionClient<rail_manipulation_msgs::LiftAction> liftClient;
  actionlib::SimpleActionServer<rail_manipulation_msgs::LiftAction> liftServer;
  actionlib::SimpleActionServer<rail_manipulation_msgs::ArmAction> armServer;
  actionlib::SimpleActionServer<rail_manipulation_msgs::PickupAction> pickupServer;
  actionlib::SimpleActionServer<rail_manipulation_msgs::StoreAction> storeServer;
  actionlib::SimpleActionServer<carl_moveit::WipeSurfaceAction> wipeSurfaceServer;

  tf::TransformBroadcaster tfBroadcaster;
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
  void executeArmAction(const rail_manipulation_msgs::ArmGoalConstPtr &goal);

  /**
  * \brief Perform a pickup action
  *
  * A pickup action with CARL consists of sequentially executing the following actions: move to approach angle,
  * open gripper, move along approach angle to the grasp pose, close gripper, optionally lift the object, and
  * optionally verify that it is in-hand.
  *
  * @param goal Grasp pose with which to execute the pickup
  */
  void executePickup(const rail_manipulation_msgs::PickupGoalConstPtr &goal);

  /**
  * \brief Perform a store action
  *
  * A store action with CARL consists of moving the end effector above the storage location on CARL's platform,
  * lowering the end effector, opening the gripper to release the object, and raising the end effector.
  *
  * @param goal Empty store goal
  */
  void executeStore(const rail_manipulation_msgs::StoreGoalConstPtr &goal);

  void executeWipeSurface(const carl_moveit::WipeSurfaceGoalConstPtr &goal);

  /**
  * \brief Raise the hand vertically by 10 cm
  * @param goal empty goal
  */
  void liftArm(const rail_manipulation_msgs::LiftGoalConstPtr &goal);

  /**
  * \brief determine if the arm is currently retracted in a given retracted position
  * @param retractPos vector of joint states the make up a retracted position
  * @return true if the arm is retracted, false otherwise
  */
  bool isArmRetracted(const std::vector<float> &retractPos);
};

#endif
