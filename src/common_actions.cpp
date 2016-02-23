#include <carl_moveit/common_actions.h>

using namespace std;

CommonActions::CommonActions() :
    moveToJointPoseClient("carl_moveit_wrapper/move_to_joint_pose"),
    moveToPoseClient("carl_moveit_wrapper/move_to_pose"),
    gripperClient("jaco_arm/manipulation/gripper"),
    liftClient("carl_moveit_wrapper/common_actions/lift"),
    liftServer(n, "carl_moveit_wrapper/common_actions/lift", boost::bind(&CommonActions::liftArm, this, _1), false),
    armServer(n, "carl_moveit_wrapper/common_actions/arm_action", boost::bind(&CommonActions::executeArmAction, this, _1), false),
    pickupServer(n, "carl_moveit_wrapper/common_actions/pickup", boost::bind(&CommonActions::executePickup, this, _1), false),
    storeServer(n, "carl_moveit_wrapper/common_actions/store", boost::bind(&CommonActions::executeStore, this, _1), false),
    wipeSurfaceServer(n, "carl_moveit_wrapper/common_actions/wipe_surface", boost::bind(&CommonActions::executeWipeSurface, this, _1), false)
{
  //setup home position
  homePosition.resize(NUM_JACO_JOINTS);
  homePosition[0] = -1.410;
  homePosition[1] = 2.975;
  homePosition[2] = .868;
  homePosition[3] = -2.323;
  homePosition[4] = 1.626;
  homePosition[5] = 1.393;

  defaultRetractPosition.resize(NUM_JACO_JOINTS);
  defaultRetractPosition[0] = -2.57;
  defaultRetractPosition[1] = 1.39;
  defaultRetractPosition[2] = .527;
  defaultRetractPosition[3] = -.084;
  defaultRetractPosition[4] = .515;
  defaultRetractPosition[5] = -1.745;

  angularCmdPublisher = n.advertise<wpi_jaco_msgs::AngularCommand>("jaco_arm/angular_cmd", 1);

  eraseTrajectoriesClient = n.serviceClient<std_srvs::Empty>("jaco_arm/erase_trajectories");
  cartesianPathClient = n.serviceClient<rail_manipulation_msgs::CartesianPath>("carl_moveit_wrapper/cartesian_path");
  jacoPosClient = n.serviceClient<wpi_jaco_msgs::GetAngularPosition>("jaco_arm/get_angular_position");
  attachClosestObjectClient = n.serviceClient<std_srvs::Empty>("carl_moveit_wrapper/attach_closest_object");
  detachObjectsClient = n.serviceClient<std_srvs::Empty>("carl_moveit_wrapper/detach_objects");

  //start action server
  liftServer.start();
  armServer.start();
  pickupServer.start();
  storeServer.start();
  wipeSurfaceServer.start();
}

void CommonActions::executePickup(const rail_manipulation_msgs::PickupGoalConstPtr &goal)
{
  rail_manipulation_msgs::PickupFeedback feedback;
  rail_manipulation_msgs::PickupResult result;
  stringstream ss;

  //make sure pose is in the base_footprint frame
  geometry_msgs::PoseStamped graspPose, approachAnglePose;
  graspPose.header.frame_id = "base_footprint";
  if (goal->pose.header.frame_id != "base_footprint")
    tfListener.transformPose("base_footprint", goal->pose, graspPose);
  else
    graspPose = goal->pose;

  tf::Transform graspTransform;
  graspTransform.setOrigin(tf::Vector3(graspPose.pose.position.x, graspPose.pose.position.y, graspPose.pose.position.z));
  graspTransform.setRotation(tf::Quaternion(graspPose.pose.orientation.x, graspPose.pose.orientation.y, graspPose.pose.orientation.z, graspPose.pose.orientation.w));
  ros::Time now = ros::Time::now();
  tfBroadcaster.sendTransform(tf::StampedTransform(graspTransform, now, "base_footprint", "grasp_frame"));
  tfListener.waitForTransform("grasp_frame", "base_footprint", now, ros::Duration(5.0));

  approachAnglePose.header.frame_id = "grasp_frame";
  approachAnglePose.pose.position.x = -0.1;
  approachAnglePose.pose.orientation.w = 1.0;

  //move to approach angle
  ss.str("");
  ss << "Attempting to move gripper to approach angle...";
  feedback.message = ss.str();
  pickupServer.publishFeedback(feedback);

  rail_manipulation_msgs::MoveToPoseGoal approachAngleGoal;
  approachAngleGoal.pose = approachAnglePose;
  moveToPoseClient.sendGoal(approachAngleGoal);
  moveToPoseClient.waitForResult(ros::Duration(30.0));
  if (!moveToPoseClient.getResult()->success)
  {
    ss.str("");
    ss << "Moving to approach angle failed for this grasp.";
    feedback.message = ss.str();
    pickupServer.publishFeedback(feedback);
    result.success = false;
    pickupServer.setAborted(result, "Unable to move to appraoch angle.");
    return;
  }

  //open gripper
  ss.str("");
  ss << "Opening gripper...";
  feedback.message = ss.str();
  pickupServer.publishFeedback(feedback);

  rail_manipulation_msgs::GripperGoal gripperGoal;
  gripperGoal.close = false;
  gripperClient.sendGoal(gripperGoal);
  gripperClient.waitForResult(ros::Duration(10.0));
  if (!gripperClient.getResult()->success)
  {
    ROS_INFO("Opening gripper failed.");
    result.success = false;
    pickupServer.setAborted(result, "Unable to open gripper.");
    return;
  }

  //follow approach angle to grasp
  ss.str("");
  ss << "Moving along approach angle...";
  feedback.message = ss.str();
  pickupServer.publishFeedback(feedback);

  rail_manipulation_msgs::CartesianPath srv;
  geometry_msgs::PoseStamped cartesianPose;
  cartesianPose.header.frame_id = "base_footprint";
  tfListener.transformPose("base_footprint", graspPose, cartesianPose);
  srv.request.waypoints.push_back(cartesianPose.pose);
  srv.request.avoidCollisions = false;
  if (!cartesianPathClient.call(srv))
  {
    ROS_INFO("Could not call Jaco Cartesian path service.");
    result.success = false;
    pickupServer.setAborted(result, "Could not call Jaco Cartesian path service.");
    return;
  }

  //close gripper
  ss.str("");
  ss << "Closing gripper...";
  feedback.message = ss.str();
  pickupServer.publishFeedback(feedback);

  gripperGoal.close = true;
  gripperClient.sendGoal(gripperGoal);
  gripperClient.waitForResult(ros::Duration(10.0));
  if (!gripperClient.getResult()->success)
  {
    ROS_INFO("Closing gripper failed.");
    result.success = false;
    pickupServer.setAborted(result, "Unable to close gripper.");
    return;
  }

  //attach scene object to gripper
  std_srvs::Empty emptySrv;
  if (!attachClosestObjectClient.call(emptySrv))
  {
    ROS_INFO("No scene object to attach...");
  }

  if (goal->lift)
  {
    //lift hand
    ss.str("");
    ss << "Lifting hand...";
    feedback.message = ss.str();
    pickupServer.publishFeedback(feedback);

    rail_manipulation_msgs::LiftGoal liftGoal;
    liftClient.sendGoal(liftGoal);
    liftClient.waitForResult(ros::Duration(10.0));
  }

  result.success = true;
  pickupServer.setSucceeded(result);
}

void CommonActions::executeStore(const rail_manipulation_msgs::StoreGoalConstPtr &goal)
{
  rail_manipulation_msgs::StoreFeedback feedback;
  rail_manipulation_msgs::StoreResult result;
  stringstream ss;

  //make sure pose is in the base_footprint frame
  /*
  geometry_msgs::PoseStamped storePose;
  storePose.header.frame_id = "base_link";
  storePose.pose.position.x = .047;
  storePose.pose.position.y = -.191;
  storePose.pose.position.z = .831;
  storePose.pose.orientation.x = .699;
  storePose.pose.orientation.y = -.294;
  storePose.pose.orientation.z = -.620;
  storePose.pose.orientation.w = -.202;
   */

  //move above store position
  ss.str("");
  ss << "Moving gripper above store position...";
  feedback.message = ss.str();
  storeServer.publishFeedback(feedback);

  rail_manipulation_msgs::MoveToPoseGoal approachAngleGoal;
  approachAngleGoal.pose = goal->store_pose;
  approachAngleGoal.pose.pose.position.z += .1;
  moveToPoseClient.sendGoal(approachAngleGoal);
  moveToPoseClient.waitForResult(ros::Duration(30.0));
  if (!moveToPoseClient.getResult()->success)
  {
    ss.str("");
    ss << "Moving gripper above store position failed.";
    feedback.message = ss.str();
    storeServer.publishFeedback(feedback);
    result.success = false;
    storeServer.setAborted(result, "Unable to move gripper above store position.");
    return;
  }

  //lower gripper
  ss.str("");
  ss << "Lowering gripper...";
  feedback.message = ss.str();
  storeServer.publishFeedback(feedback);

  rail_manipulation_msgs::CartesianPath srv;
  geometry_msgs::PoseStamped cartesianPose;
  approachAngleGoal.pose.pose.position.z -= .1;
  cartesianPose.header.frame_id = "base_footprint";
  tfListener.transformPose("base_footprint", approachAngleGoal.pose, cartesianPose);
  srv.request.waypoints.push_back(cartesianPose.pose);
  srv.request.avoidCollisions = false;
  if (!cartesianPathClient.call(srv))
  {
    ROS_INFO("Could not call Jaco Cartesian path service.");
    result.success = false;
    storeServer.setAborted(result, "Could not call Jaco Cartesian path service.");
    return;
  }

  //open gripper
  ss.str("");
  ss << "Opening gripper...";
  feedback.message = ss.str();
  storeServer.publishFeedback(feedback);

  rail_manipulation_msgs::GripperGoal gripperGoal;
  gripperGoal.close = false;
  gripperClient.sendGoal(gripperGoal);
  gripperClient.waitForResult(ros::Duration(10.0));
  if (!gripperClient.getResult()->success)
  {
    ROS_INFO("Opening gripper failed.");
    result.success = false;
    storeServer.setAborted(result, "Unable to open gripper.");
    return;
  }

  //detach any attached collision objects
  std_srvs::Empty emptySrv;
  if (!detachObjectsClient.call(emptySrv))
  {
    ROS_INFO("Couldn't call detach objects service.");
  }

  //raise gripper
  ss.str("");
  ss << "Raising gripper...";
  feedback.message = ss.str();
  storeServer.publishFeedback(feedback);

  cartesianPose.pose.position.z += .05;
  srv.request.waypoints.clear();
  srv.request.waypoints.push_back(cartesianPose.pose);
  srv.request.avoidCollisions = false;
  if (!cartesianPathClient.call(srv))
  {
    ROS_INFO("Could not call Jaco Cartesian path service.");
    result.success = false;
    storeServer.setAborted(result, "Could not call Jaco Cartesian path service.");
    return;
  }

  result.success = true;
  storeServer.setSucceeded(result);
}

void CommonActions::executeArmAction(const rail_manipulation_msgs::ArmGoalConstPtr &goal)
{
  rail_manipulation_msgs::ArmFeedback feedback;
  rail_manipulation_msgs::ArmResult result;

  switch (goal->action) {
    case rail_manipulation_msgs::ArmGoal::READY:
      feedback.message = "Readying arm...";
      armServer.publishFeedback(feedback);
    break;
    case rail_manipulation_msgs::ArmGoal::RETRACT:
      feedback.message = "Retracting arm...";
      armServer.publishFeedback(feedback);
    break;
    default:
      feedback.message = "Executing arm action...";
      armServer.publishFeedback(feedback);
    break;
  }

  if (goal->action == rail_manipulation_msgs::ArmGoal::RETRACT)
  {
    if (isArmRetracted(defaultRetractPosition))
    {
      feedback.message = "Arm is already retracted.";
      armServer.publishFeedback(feedback);

      result.success = true;
      armServer.setSucceeded(result);
      return;
    }
  }

  rail_manipulation_msgs::MoveToJointPoseGoal jointPoseGoal;

  vector<float> baseJointPoseGoal;
  baseJointPoseGoal.resize(homePosition.size());
  for (unsigned int i = 0; i < baseJointPoseGoal.size(); i ++)
  {
    baseJointPoseGoal[i] = homePosition[i];
  }
  jointPoseGoal.joints.resize(baseJointPoseGoal.size());
  for (unsigned int i = 0; i < jointPoseGoal.joints.size(); i ++)
  {
    jointPoseGoal.joints[i] = baseJointPoseGoal[i];
  }
  bool succeeded = false;
  int counter = 0;
  int attempts = MAX_HOME_ATTEMPTS;
  while (!succeeded && counter < attempts)
  {
    stringstream ss;
    ss << "Planning and moving arm to ready position (attempt " << counter + 1 << "/" << attempts << ")";
    feedback.message = ss.str();
    armServer.publishFeedback(feedback);

    ROS_INFO("Ready arm attempt %d", counter);

    moveToJointPoseClient.sendGoal(jointPoseGoal);
    ROS_INFO("Moving arm to ready position...");
    while (!moveToJointPoseClient.getState().isDone())
    {
      if (armServer.isPreemptRequested() || !ros::ok())
      {
        ROS_INFO("Ready arm action preempted.");
        moveToJointPoseClient.cancelAllGoals();
        result.success = false;
        armServer.setPreempted(result);
        return;
      }
    }

    rail_manipulation_msgs::MoveToJointPoseResultConstPtr readyResult = moveToJointPoseClient.getResult();

    succeeded = readyResult->success;
    counter ++;

    //slightly vary joint goal and retry planning
    if (!succeeded && counter < attempts)
    {
      ROS_INFO("Ready arm failed, resampling goal for another attempt...");
      for (unsigned int i = 0; i < jointPoseGoal.joints.size(); i ++)
      {
        jointPoseGoal.joints[i] = baseJointPoseGoal[i] + (rand() % 700 - 350) / 10000;  //vary by up to ~2 degrees
      }
    }
  }

  if (!succeeded)
  {
    feedback.message = "Failed to ready arm.";
    armServer.publishFeedback(feedback);

    ROS_INFO("Plan and move to ready position failed.");
    result.success = false;
    armServer.setSucceeded(result);
    return;
  }

  if (goal->action == rail_manipulation_msgs::ArmGoal::RETRACT)
  {
    feedback.message = "Moving arm to retracted position...";
    armServer.publishFeedback(feedback);

    wpi_jaco_msgs::AngularCommand cmd;
    cmd.armCommand = true;
    cmd.fingerCommand = false;
    cmd.position = true;
    cmd.repeat = false;
    cmd.joints = defaultRetractPosition;
    angularCmdPublisher.publish(cmd);
    ros::Time startTime = ros::Time::now();
    ros::Rate loopRate(30);
    bool retracted = false;
    while (!retracted)
    {
      //check for preempt
      if (armServer.isPreemptRequested() || !ros::ok())
      {
        ROS_INFO("Ready arm action preempted.");
        std_srvs::Empty srv;
        if (!eraseTrajectoriesClient.call(srv))
        {
          ROS_INFO("Could not call erase trajectories service...");
        }
        result.success = false;
        armServer.setPreempted(result);
        return;
      }

      //check if arm is retracted
      retracted = isArmRetracted(defaultRetractPosition);

      //check for timeout
      ros::Time currentTime = ros::Time::now();
      if ((currentTime.toSec() - startTime.toSec()) > 10.0)
      {
        feedback.message = "Retracting arm timed out.";
        armServer.publishFeedback(feedback);

        ROS_INFO("Ready arm timed out.");
        result.success = false;
        armServer.setSucceeded(result);
        return;
      }
      loopRate.sleep();
    }
  }

  switch (goal->action) {
    case rail_manipulation_msgs::ArmGoal::READY:
      feedback.message = "Ready arm completed.";
      armServer.publishFeedback(feedback);
      break;
    case rail_manipulation_msgs::ArmGoal::RETRACT:
      feedback.message = "Retract arm completed.";
      armServer.publishFeedback(feedback);
      break;
    default:
      feedback.message = "Arm action completed.";
      armServer.publishFeedback(feedback);
      break;
  }

  result.success = succeeded;
  armServer.setSucceeded(result);
}

void CommonActions::executeWipeSurface(const carl_moveit::WipeSurfaceGoalConstPtr &goal)
{
  carl_moveit::WipeSurfaceFeedback feedback;
  carl_moveit::WipeSurfaceResult result;
  stringstream ss;

  //move to wiping position
  rail_manipulation_msgs::MoveToPoseGoal poseGoal;
  poseGoal.pose.header.frame_id = "base_footprint";
  poseGoal.pose.pose.position.x = 0.869;
  poseGoal.pose.pose.position.y = 0.0;
  poseGoal.pose.pose.position.z = goal->height;
  poseGoal.pose.pose.orientation.x = -0.605;
  poseGoal.pose.pose.orientation.y = 0.271;
  poseGoal.pose.pose.orientation.z = 0.699;
  poseGoal.pose.pose.orientation.w = 0.267;

  moveToPoseClient.sendGoal(poseGoal);
  moveToPoseClient.waitForResult(ros::Duration(30.0));
  if (!moveToPoseClient.getResult()->success)
  {
    ss.str("");
    ss << "Moving to initial wiping position failed.";
    feedback.message = ss.str();
    wipeSurfaceServer.publishFeedback(feedback);
    result.success = false;
    wipeSurfaceServer.setAborted(result, "Unable to move to appraoch angle.");
    return;
  }

  //move arm back and forth to wipe
  ss.str("");
  ss << "Wiping...";
  feedback.message = ss.str();
  wipeSurfaceServer.publishFeedback(feedback);

  rail_manipulation_msgs::CartesianPath srv;
  for (unsigned int i = 0; i < 3; i ++)
  {
    geometry_msgs::Pose pose = poseGoal.pose.pose;
    /*
    //back and forth motion
    pose.position.y += .2;
    srv.request.waypoints.push_back(pose);
    pose.position.y -= .4;
    srv.request.waypoints.push_back(pose);
    */
    //diamond motion
    pose.position.y += .15;
    srv.request.waypoints.push_back(pose);
    pose.position.y -= .15;
    pose.position.x += .05;
    srv.request.waypoints.push_back(pose);
    pose.position.y -= .15;
    pose.position.x -= .05;
    srv.request.waypoints.push_back(pose);
    pose.position.y += .15;
    pose.position.x -= .05;
    srv.request.waypoints.push_back(pose);
  }
  ROS_INFO("Waypoints y:");
  for (unsigned int i = 0; i < srv.request.waypoints.size(); i ++)
  {
    ROS_INFO("%f", srv.request.waypoints[i].position.y);
  }
  srv.request.avoidCollisions = false;
  if (!cartesianPathClient.call(srv))
  {
    ROS_INFO("Could not call Jaco Cartesian path service.");
    result.success = false;
    wipeSurfaceServer.setAborted(result, "Could not call Jaco Cartesian path service.");
    return;
  }

  ROS_INFO("Wipe surface action completed successfully.");
  result.success = true;
  wipeSurfaceServer.setSucceeded(result);
}

void CommonActions::liftArm(const rail_manipulation_msgs::LiftGoalConstPtr &goal)
{
  rail_manipulation_msgs::LiftResult result;

  rail_manipulation_msgs::CartesianPath srv;
  tf::StampedTransform currentEefTransform;
  tfListener.waitForTransform("jaco_link_eef", "base_footprint", ros::Time::now(), ros::Duration(1.0));
  tfListener.lookupTransform("base_footprint", "jaco_link_eef", ros::Time(0), currentEefTransform);
  geometry_msgs::Pose liftPose;
  liftPose.position.x = currentEefTransform.getOrigin().x();
  liftPose.position.y = currentEefTransform.getOrigin().y();
  liftPose.position.z = currentEefTransform.getOrigin().z() + .1;
  liftPose.orientation.x = currentEefTransform.getRotation().x();
  liftPose.orientation.y = currentEefTransform.getRotation().y();
  liftPose.orientation.z = currentEefTransform.getRotation().z();
  liftPose.orientation.w = currentEefTransform.getRotation().w();
  srv.request.waypoints.push_back(liftPose);
  srv.request.avoidCollisions = false;

  if (!cartesianPathClient.call(srv))
  {
    ROS_INFO("Could not call Jaco Cartesian path service.");
    result.success = false;
    liftServer.setAborted(result, "Could not call Jaco Cartesian path service.");
    return;
  }

  result.success = srv.response.success;
  liftServer.setSucceeded(result);
}

bool CommonActions::isArmRetracted(const vector<float> &retractPos)
{
  float dstFromRetract = 0;

  //get joint positions
  wpi_jaco_msgs::GetAngularPosition::Request req;
  wpi_jaco_msgs::GetAngularPosition::Response res;
  if(!jacoPosClient.call(req, res))
  {
    ROS_INFO("Could not call Jaco joint position service.");
    return false;
  }

  for (unsigned int i = 0; i < retractPos.size(); i ++)
  {
    dstFromRetract += fabs(retractPos[i] - res.pos[i]);
  }

  if (dstFromRetract > 0.175)
    return false;
  return true;
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "carl_moveit_common_actions");

  CommonActions ca;

  ros::spin();
}
