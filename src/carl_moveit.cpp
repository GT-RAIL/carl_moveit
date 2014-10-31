#include <carl_moveit/carl_moveit.h>

using namespace std;

CarlMoveIt::CarlMoveIt() :
    armTrajectoryClient("jaco_arm/joint_velocity_controller/trajectory"),
    moveToPoseServer(n, "carl_moveit_wrapper/move_to_pose", boost::bind(&CarlMoveIt::moveToPose, this, _1), false)
{
  armJointStateSubscriber = n.subscribe("joint_states", 1, &CarlMoveIt::armJointStatesCallback, this);

  ikClient = n.serviceClient<moveit_msgs::GetPositionIK>("compute_ik");

  robot_model_loader::RobotModelLoader robotModelLoader("robot_description");
  kinematicModel = robotModelLoader.getModel();
  armGroup = new move_group_interface::MoveGroup("arm");
  armGroup->startStateMonitor();

  //advertise service
  ikServer = n.advertiseService("carl_moveit_wrapper/call_ik", &CarlMoveIt::ikCallback, this);

  //start action server
  moveToPoseServer.start();
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
    for (unsigned int i = jacoStartIndex; i <= jacoStartIndex + NUM_JACO_JOINTS; i++)
    {
      jointGoal[i - jacoStartIndex] = nearest_equivalent(simplify_angle(ikRes.solution.joint_state.position[i]), jointState.position[i]);
    }

    //plan and execute
    armGroup->setPlannerId("arm[KPIECEkConfigDefault]");
    armGroup->setPlanningTime(5.0);
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
    /*
    //Give the planner some time to start trajectory execution
    ros::Duration(2.0).sleep();
    
    //TODO: Test this
    //if the arm trajectory server is not active, the planner failed
    actionlib::SimpleClientGoalState armTrajectoryState = armTrajectoryClient.getState();
    if (armTrajectoryState.isDone())
    {
      ROS_INFO("Detected failure to plan an appropriate trajectory.");
      result.success = false;
    }
    
    //wait for arm trajectory server to reach a terminal state
    ros::Rate pollRate(30);
    while (!armTrajectoryState.isDone())
    {
      pollRate.sleep();
      armTrajectoryState = armTrajectoryClient.getState();
    }
    //TODO: End testing
    */

    result.success = moveSuccess;
  }
  else
  {
    ROS_INFO("IK service call failed with error code: %d", ikRes.error_code.val);
    result.success = false;
  }

  moveToPoseServer.setSucceeded(result);
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

  robot_state::RobotStatePtr kinematicState(new robot_state::RobotState(kinematicModel));
  const robot_state::JointModelGroup *jointModelGroup = kinematicModel->getJointModelGroup("arm");
  kinematicState->setVariableValues(jointState);

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

int main(int argc, char **argv)
{
  ros::init(argc, argv, "carl_moveit_wrapper");

  CarlMoveIt c;

  ros::spin();
}

