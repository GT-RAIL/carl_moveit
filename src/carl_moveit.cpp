#include <carl_moveit/carl_moveit.h>
#include <carl_moveit/eigen_pinv.hpp>

using namespace std;

CarlMoveIt::CarlMoveIt() :
    armTrajectoryClient("jaco_arm/joint_velocity_controller/trajectory"),
    moveToPoseServer(n, "carl_moveit_wrapper/move_to_pose", boost::bind(&CarlMoveIt::moveToPose, this, _1), false),
    moveToJointPoseServer(n, "carl_moveit_wrapper/move_to_joint_pose", boost::bind(&CarlMoveIt::moveToJointPose, this, _1), false)
{
  armJointStateSubscriber = n.subscribe("joint_states", 1, &CarlMoveIt::armJointStatesCallback, this);
  cartesianControlSubscriber = n.subscribe("carl_moveit_wrapper/cartesian_control", 1, &CarlMoveIt::cartesianControlCallback, this);

  angularCmdPublisher = n.advertise<wpi_jaco_msgs::AngularCommand>("jaco_arm/angular_cmd", 1);
  trajectoryVisPublisher = n.advertise<moveit_msgs::DisplayTrajectory>("carl_moveit/computed_trajectory", 1);

  ikClient = n.serviceClient<moveit_msgs::GetPositionIK>("compute_ik");

  armGroup = new move_group_interface::MoveGroup("arm");
  armGroup->startStateMonitor();

  //advertise service
  cartesianPathServer = n.advertiseService("carl_moveit_wrapper/cartesian_path", &CarlMoveIt::cartesianPathCallback, this);
  ikServer = n.advertiseService("carl_moveit_wrapper/call_ik", &CarlMoveIt::ikCallback, this);

  //start action server
  moveToPoseServer.start();
  moveToJointPoseServer.start();
}

CarlMoveIt::~CarlMoveIt()
{
  delete armGroup;
}

void CarlMoveIt::armJointStatesCallback(const sensor_msgs::JointState &msg)
{
  jointState = msg;
}

/** Adjust angle to equivalent angle on [-pi, pi]
*  @param angle the angle to be simplified (-inf, inf)
*  @return the simplified angle on [-pi, pi]
*/
static inline double simplify_angle(double angle)
{
  double previous_rev = floor(angle / (2.0 * M_PI)) * 2.0 * M_PI;
  double next_rev = ceil(angle / (2.0 * M_PI)) * 2.0 * M_PI;
  double current_rev;
  if (fabs(angle - previous_rev) < fabs(angle - next_rev))
    return angle - previous_rev;
  return angle - next_rev;
}

/** Calculates nearest desired angle to the current angle
*  @param desired desired joint angle [-pi, pi]
*  @param current current angle (-inf, inf)
*  @return the closest equivalent angle (-inf, inf)
*/
static inline double nearest_equivalent(double desired, double current)
{
  //calculate current number of revolutions
  double previous_rev = floor(current / (2 * M_PI));
  double next_rev = ceil(current / (2 * M_PI));
  double current_rev;
  if (fabs(current - previous_rev * 2 * M_PI) < fabs(current - next_rev * 2 * M_PI))
    current_rev = previous_rev;
  else
    current_rev = next_rev;

  //determine closest angle
  double lowVal = (current_rev - 1) * 2 * M_PI + desired;
  double medVal = current_rev * 2 * M_PI + desired;
  double highVal = (current_rev + 1) * 2 * M_PI + desired;
  if (fabs(current - lowVal) <= fabs(current - medVal) && fabs(current - lowVal) <= fabs(current - highVal))
    return lowVal;
  if (fabs(current - medVal) <= fabs(current - lowVal) && fabs(current - medVal) <= fabs(current - highVal))
    return medVal;
  return highVal;
}

void CarlMoveIt::moveToPose(const carl_moveit::MoveToPoseGoalConstPtr &goal)
{
  moveit_msgs::GetPositionIK::Response ikRes = callIK(goal->pose);

  carl_moveit::MoveToPoseResult result;
  if (ikRes.error_code.val == ikRes.error_code.SUCCESS)
  {
    ROS_INFO("IK service call succeeded");

    //extract joint states
    int jacoStartIndex = 0;
    for (unsigned int i = 0; i < jointState.name.size(); i++)
    {
      if (jointState.name[i].compare("jaco_joint_1") == 0)
      {
        jacoStartIndex = i;
        break;
      }
    }

    std::vector<double> jointGoal;
    jointGoal.resize(6);
    //set joints to be closest to current joint positions
    for (unsigned int i = jacoStartIndex; i < jacoStartIndex + NUM_JACO_JOINTS; i++)
    {
      jointGoal[i - jacoStartIndex] = nearest_equivalent(simplify_angle(ikRes.solution.joint_state.position[i]), jointState.position[i]);
    }

    //plan and execute
    armGroup->setPlannerId("arm[KPIECEkConfigDefault]");
    if (goal->planningTime == 0.0)
      armGroup->setPlanningTime(5.0);
    else
      armGroup->setPlanningTime(goal->planningTime);
    armGroup->setStartStateToCurrentState();
    armGroup->setJointValueTarget(jointGoal);
    ROS_INFO("Planning and moving...");
    //armGroup->asyncMove();
    bool moveSuccess = armGroup->move();
    ROS_INFO("Finished plan and move");
    if (moveSuccess)
      ROS_INFO("Succeeded");
    else
      ROS_INFO("Failed");

    result.success = moveSuccess;
  }
  else
  {
    ROS_INFO("IK service call failed with error code: %d", ikRes.error_code.val);
    result.success = false;
  }

  moveToPoseServer.setSucceeded(result);
}

void CarlMoveIt::moveToJointPose(const carl_moveit::MoveToJointPoseGoalConstPtr &goal)
{
  //extract joint states
  int jacoStartIndex = 0;
  for (unsigned int i = 0; i < jointState.name.size(); i++)
  {
    if (jointState.name[i].compare("jaco_joint_1") == 0)
    {
      jacoStartIndex = i;
      break;
    }
  }

  vector<double> jointGoal;
  jointGoal.resize(NUM_JACO_JOINTS);
  //set joints to be closest to current joint positions
  for (unsigned int i = jacoStartIndex; i < jacoStartIndex + NUM_JACO_JOINTS; i++)
  {
    jointGoal[i - jacoStartIndex] = nearest_equivalent(simplify_angle(goal->joints[i - jacoStartIndex]), jointState.position[i]);
  }

  //plan and execute
  carl_moveit::MoveToJointPoseResult result;
  armGroup->setPlannerId("arm[KPIECEkConfigDefault]");
  if (goal->planningTime == 0.0)
    armGroup->setPlanningTime(5.0);
  else
    armGroup->setPlanningTime(goal->planningTime);
  armGroup->setStartStateToCurrentState();
  armGroup->setJointValueTarget(jointGoal);
  ROS_INFO("Planning and moving...");
  //armGroup->asyncMove();
  bool moveSuccess = armGroup->move();
  if (moveSuccess)
    ROS_INFO("Plan and move succeeded");
  else
    ROS_INFO("Plan and move failed");

  result.success = moveSuccess;

  moveToJointPoseServer.setSucceeded(result);
}

bool CarlMoveIt::cartesianPathCallback(carl_moveit::CartesianPath::Request &req, carl_moveit::CartesianPath::Response &res)
{
  double eefStep = .05;
  double  jumpThreshold = 1.5;
  moveit_msgs::RobotTrajectory finalTraj;

  //calculate trajectory
  moveit_msgs::RobotTrajectory tempTraj;
  double completion = armGroup->computeCartesianPath(req.waypoints, eefStep, jumpThreshold, tempTraj, req.avoidCollisions);
  if (completion == -1)
  {
    ROS_INFO("Could not calculate a path.");
    res.success = false;
    return true;
  }

  if (completion == 1.0)
  {
    finalTraj = tempTraj;
  }
  else
  {
    ROS_INFO("Could not find a complete path, varying parameters and recalculating...");
    //vary jumpThreshold and eefStep
    for (unsigned int i = 0; i < 2; i ++)
    {
      double newCompletion;
      if (i != 0)
        jumpThreshold += 1.5;
      for (unsigned int j = 0; j < 3; j ++)
      {
        if (j == 0)
          eefStep /= 2.0;
        else
          eefStep *= 2.0;
        newCompletion = armGroup->computeCartesianPath(req.waypoints, eefStep, jumpThreshold, tempTraj, req.avoidCollisions);
        if (newCompletion > completion)
        {
          ROS_INFO("Found a better path.");
          finalTraj = tempTraj;
          completion = newCompletion;
          if (newCompletion == 1.0)
          {
            ROS_INFO("Found a complete path!");
            break;
          }
        }
      }
      if (newCompletion == 1.0)
        break;
    }
  }

  if (completion == 0.0)
  {
    ROS_INFO("Could not calculate a path.");
    res.success = false;
    return true;
  }
  else
  {
    res.success = true;
    res.completion = completion;
  }

  //display trajectory
  /*
  moveit_msgs::DisplayTrajectory trajVis;
  trajVis.model_id = "carl";
  trajVis.trajectory.clear();
  trajVis.trajectory.push_back(finalTraj);
  moveit::core::robotStateToRobotStateMsg(*(armGroup->getCurrentState()), trajVis.trajectory_start);
  trajectoryVisPublisher.publish(trajVis);
  */

  //execute the trajectory
  move_group_interface::MoveGroup::Plan plan;
  plan.trajectory_ = finalTraj;
  moveit::core::robotStateToRobotStateMsg(*(armGroup->getCurrentState()), plan.start_state_);
  //plan.planning_time_ = 0.0; //does this matter?
  armGroup->execute(plan);

  res.success = true;
  return true;
}

bool CarlMoveIt::ikCallback(carl_moveit::CallIK::Request &req, carl_moveit::CallIK::Response &res)
{
  moveit_msgs::GetPositionIK::Response ikRes = callIK(req.pose);

  if (ikRes.error_code.val == ikRes.error_code.SUCCESS)
  {
    ROS_INFO("IK service call succeeded");

    //extract joint states
    int jacoStartIndex = 0;
    for (unsigned int i = 0; i < jointState.name.size(); i++)
    {
      if (jointState.name[i].compare("jaco_joint_1") == 0)
      {
        jacoStartIndex = i;
        break;
      }
    }

    std::vector<double> joints;
    joints.resize(6);
    //set joints to be closest to current joint positions
    for (unsigned int i = jacoStartIndex; i <= jacoStartIndex + NUM_JACO_JOINTS; i++)
    {
      joints[i - jacoStartIndex] = nearest_equivalent(simplify_angle(ikRes.solution.joint_state.position[i]), jointState.position[i]);
    }

    res.jointPositions = joints;
    res.success = true;
  }
  else
  {
    ROS_INFO("IK service call failed with error code: %d", ikRes.error_code.val);
    res.success = false;
  }

  return true;
}

moveit_msgs::GetPositionIK::Response CarlMoveIt::callIK(geometry_msgs::Pose pose)
{
  moveit_msgs::GetPositionIK::Request ikReq;
  moveit_msgs::GetPositionIK::Response ikRes;

  //robot_state::RobotStatePtr kinematicState(new robot_state::RobotState(kinematicModel));
  robot_state::RobotStatePtr kinematicState = armGroup->getCurrentState();
  const robot_state::JointModelGroup *jointModelGroup = kinematicState->getRobotModel()->getJointModelGroup("arm");
  //kinematicState->setVariableValues(jointState);

  ikReq.ik_request.group_name = "arm";
  ikReq.ik_request.pose_stamped.header.frame_id = "base_footprint";
  ikReq.ik_request.pose_stamped.pose = pose;
  ikReq.ik_request.ik_link_name = "jaco_link_hand";
  //seed state
  ikReq.ik_request.robot_state.joint_state.name = jointModelGroup->getJointModelNames();
  kinematicState->copyJointGroupPositions(jointModelGroup, ikReq.ik_request.robot_state.joint_state.position);
  //other parameters
  //ikReq.ik_request.avoid_collisions = true;
  ikReq.ik_request.timeout = ros::Duration(.1);
  ikReq.ik_request.attempts = 10;

  ikClient.call(ikReq, ikRes);

  return ikRes;
}

void CarlMoveIt::cartesianControlCallback(const geometry_msgs::Twist &msg)
{
  //get the jacobian
  robot_state::RobotStatePtr kinematicState = armGroup->getCurrentState();
  const moveit::core::JointModelGroup* jointModelGroup = kinematicState->getRobotModel()->getJointModelGroup("arm");
  Eigen::Vector3d referencePointPosition(0.0, 0.0, 0.0);  //what does this do?
  Eigen::MatrixXd jacobian;
  kinematicState->getJacobian(jointModelGroup, kinematicState->getLinkModel(jointModelGroup->getLinkModelNames().back()), referencePointPosition, jacobian);

  //calculate the jacobian pseudoinverse

  //Method 1: SVD
  Eigen::MatrixXd pInv = EIGEN_PINV::pinv(jacobian, 0.001);


  //Method 2: Permuting the jacobian's main diagonal
  /*
  Eigen::MatrixXd pInv;
  float val = .001;
  Eigen::MatrixXd permutedJacobian = jacobian + val*Eigen::MatrixXd::Identity(6, 6);
  pInv = permutedJacobian.inverse();
  */

  //calculate joint velocities
  Eigen::VectorXd twist(6);
  twist << msg.linear.x, msg.linear.y, msg.linear.z, msg.angular.x, msg.angular.y, msg.angular.z;
  Eigen::VectorXd jointVel(6);
  jointVel = pInv * twist;

  //publish joint velocity command to the arm
  wpi_jaco_msgs::AngularCommand cmd;
  cmd.position = false;
  cmd.armCommand = true;
  cmd.fingerCommand = false;
  cmd.repeat = true;
  cmd.joints.resize(6);
  for(unsigned int i = 0; i < jointVel.size(); i ++)
  {
    cmd.joints[i] = jointVel[i];
  }
  angularCmdPublisher.publish(cmd);
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "carl_moveit_wrapper");

  CarlMoveIt c;

  ros::spin();
}
