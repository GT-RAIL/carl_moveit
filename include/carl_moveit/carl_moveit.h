#ifndef CARL_MOVEIT_H_
#define CARL_MOVEIT_H_

//CPP
#include <boost/thread/recursive_mutex.hpp>

//ROS
#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>
#include <actionlib/client/simple_action_client.h>
#include <carl_moveit/CallIK.h>
#include <carl_moveit/CartesianPath.h>
#include <carl_moveit/MoveToJointPoseAction.h>
#include <carl_moveit/MoveToPoseAction.h>
#include <control_msgs/FollowJointTrajectoryAction.h>
#include <moveit/robot_state/conversions.h>
#include <moveit/move_group_interface/move_group.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/robot_model/robot_model.h>
#include <moveit/robot_state/robot_state.h>
#include <moveit_msgs/CollisionObject.h>
#include <moveit_msgs/DisplayTrajectory.h>
#include <moveit_msgs/GetPositionIK.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Twist.h>
#include <pcl_ros/point_cloud.h>
#include <rail_manipulation_msgs/SegmentedObjectList.h>
#include <sensor_msgs/JointState.h>
#include <sensor_msgs/point_cloud_conversion.h>
#include <std_msgs/Bool.h>
#include <std_srvs/Empty.h>
#include <tf2_ros/transform_listener.h>
#include <wpi_jaco_msgs/AngularCommand.h>

//PCL
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/common/common.h>
#include <pcl/common/transforms.h>

#define NUM_JACO_JOINTS 6
#define SCENE_OBJECT_DST_THRESHOLD 0.2

class CarlMoveIt
{

public:

  CarlMoveIt();

  ~CarlMoveIt();

private:
  ros::NodeHandle n;
  ros::Subscriber armJointStateSubscriber;
  ros::Subscriber cartesianControlSubscriber;
  ros::Subscriber armHomedSubscriber;
  ros::Subscriber recognizedObjectsSubscriber;
  ros::Publisher angularCmdPublisher;
  ros::Publisher trajectoryVisPublisher;
  ros::ServiceServer cartesianPathServer;
  ros::ServiceServer ikServer;
  ros::ServiceServer attachObjectServer;
  ros::ServiceServer detachObjectServer;
  ros::ServiceClient ikClient;
  ros::ServiceClient clearOctomapClient;

  actionlib::SimpleActionClient<control_msgs::FollowJointTrajectoryAction> armTrajectoryClient;
  actionlib::SimpleActionServer<carl_moveit::MoveToPoseAction> moveToPoseServer;
  actionlib::SimpleActionServer<carl_moveit::MoveToJointPoseAction> moveToJointPoseServer;

  tf2_ros::Buffer tfBuffer;
  tf2_ros::TransformListener tfListener;

  move_group_interface::MoveGroup *armGroup;
  move_group_interface::PlanningSceneInterface *planningSceneInterface;

  boost::recursive_mutex api_mutex;

  sensor_msgs::JointState jointState;
  rail_manipulation_msgs::SegmentedObjectList objectList;  //the last received object list
  std::vector<std::string> attachedObjects;  //the names of the objects (in the planning scene) attached to the robot
  std::vector<std::string> unattachedObjects; //the names of the objects (in the planning scene) not attached to the robot

  void moveToPose(const carl_moveit::MoveToPoseGoalConstPtr &goal);

  void moveToJointPose(const carl_moveit::MoveToJointPoseGoalConstPtr &goal);

  bool cartesianPathCallback(carl_moveit::CartesianPath::Request &req, carl_moveit::CartesianPath::Response &res);

  bool ikCallback(carl_moveit::CallIK::Request &req, carl_moveit::CallIK::Response &res);

  moveit_msgs::GetPositionIK::Response callIK(geometry_msgs::PoseStamped pose);

  void armJointStatesCallback(const sensor_msgs::JointState &msg);

  void cartesianControlCallback(const geometry_msgs::Twist &msg);

  void armHomedCallback(const std_msgs::Bool &msg);

  void recognizedObjectsCallback(const rail_manipulation_msgs::SegmentedObjectList &msg);

  bool attachClosestSceneObject(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res);

  bool detachSceneObjects(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res);
};

#endif
