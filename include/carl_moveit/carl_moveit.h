#ifndef CARL_MOVEIT_H_
#define CARL_MOVEIT_H_

#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>
#include <actionlib/client/simple_action_client.h>
#include <carl_moveit/MoveToPoseAction.h>
#include <carl_moveit/CallIK.h>
#include <moveit/move_group_interface/move_group.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/robot_model/robot_model.h>
#include <moveit/robot_state/robot_state.h>
#include <moveit_msgs/GetPositionIK.h>
#include <geometry_msgs/PoseStamped.h>
#include <sensor_msgs/JointState.h>
#include <control_msgs/FollowJointTrajectoryAction.h>

#define NUM_JACO_JOINTS 6

class CarlMoveIt
{

public:

  CarlMoveIt();

  ~CarlMoveIt();

  void moveToPose(const carl_moveit::MoveToPoseGoalConstPtr &goal);

  bool ikCallback(carl_moveit::CallIK::Request &req, carl_moveit::CallIK::Response &res);

  moveit_msgs::GetPositionIK::Response callIK(geometry_msgs::Pose pose);

  void armJointStatesCallback(const sensor_msgs::JointState &msg);

private:
  ros::NodeHandle n;
  ros::Subscriber armJointStateSubscriber;
  ros::ServiceServer ikServer;
  ros::ServiceClient ikClient;

  actionlib::SimpleActionClient<control_msgs::FollowJointTrajectoryAction> armTrajectoryClient;
  actionlib::SimpleActionServer<carl_moveit::MoveToPoseAction> moveToPoseServer;

  move_group_interface::MoveGroup *armGroup;
  robot_model::RobotModelPtr kinematicModel;

  sensor_msgs::JointState jointState;
};

#endif
