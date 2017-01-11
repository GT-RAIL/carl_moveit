#include <carl_moveit/carl_moveit.h>
#include <carl_moveit/eigen_pinv.hpp>

using namespace std;

CarlMoveIt::CarlMoveIt() :
    tfListener(tfBuffer),
    armTrajectoryClient("jaco_arm/joint_velocity_controller/trajectory"),
    moveToPoseServer(n, "carl_moveit_wrapper/move_to_pose", boost::bind(&CarlMoveIt::moveToPose, this, _1), false),
    moveToJointPoseServer(n, "carl_moveit_wrapper/move_to_joint_pose", boost::bind(&CarlMoveIt::moveToJointPose, this, _1), false)
{
  armJointStateSubscriber = n.subscribe("joint_states", 1, &CarlMoveIt::armJointStatesCallback, this);
  cartesianControlSubscriber = n.subscribe("carl_moveit_wrapper/cartesian_control", 1, &CarlMoveIt::cartesianControlCallback, this);
  armHomedSubscriber = n.subscribe("jaco_arm/arm_homed", 1, &CarlMoveIt::armHomedCallback, this);
  recognizedObjectsSubscriber = n.subscribe("object_recognition_listener/recognized_objects", 1, &CarlMoveIt::recognizedObjectsCallback, this);

  angularCmdPublisher = n.advertise<wpi_jaco_msgs::AngularCommand>("jaco_arm/angular_cmd", 1);
  trajectoryVisPublisher = n.advertise<moveit_msgs::DisplayTrajectory>("carl_moveit/computed_trajectory", 1);

  ikClient = n.serviceClient<moveit_msgs::GetPositionIK>("compute_ik");
  clearOctomapClient = n.serviceClient<std_srvs::Empty>("clear_octomap");

  armGroup = new move_group_interface::MoveGroup("arm");
  armGroup->startStateMonitor();

  planningSceneInterface = new move_group_interface::PlanningSceneInterface();

  //advertise service
  cartesianPathServer = n.advertiseService("carl_moveit_wrapper/cartesian_path", &CarlMoveIt::cartesianPathCallback, this);
  ikServer = n.advertiseService("carl_moveit_wrapper/call_ik", &CarlMoveIt::ikCallback, this);
  attachObjectServer = n.advertiseService("carl_moveit_wrapper/attach_closest_object", &CarlMoveIt::attachClosestSceneObject, this);
  detachObjectServer = n.advertiseService("carl_moveit_wrapper/detach_objects", &CarlMoveIt::detachSceneObjects, this);

  //start action server
  moveToPoseServer.start();
  moveToJointPoseServer.start();
}

CarlMoveIt::~CarlMoveIt()
{
  delete armGroup;
  delete planningSceneInterface;
}

void CarlMoveIt::armHomedCallback(const std_msgs::Bool &msg)
{
  if (msg.data)
  {
    std_srvs::Empty srv;
    if (!clearOctomapClient.call(srv))
      ROS_INFO("Failed to call clear octomap service.");
  }
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

void CarlMoveIt::moveToPose(const rail_manipulation_msgs::MoveToPoseGoalConstPtr &goal)
{
  moveit_msgs::GetPositionIK::Response ikRes = callIK(goal->pose);

  rail_manipulation_msgs::MoveToPoseResult result;
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
    move_group_interface::MoveItErrorCode errorCode = armGroup->move();
    ROS_INFO("Finished plan and move");
    if (errorCode == moveit_msgs::MoveItErrorCodes::SUCCESS)
    {
      ROS_INFO("Succeeded");
      result.success = true;
    }
    else
    {
      ROS_INFO("Failed with MoveIt error code: %d", errorCode.val);
      result.success = false;
    }
  }
  else
  {
    ROS_INFO("IK service call failed with error code: %d", ikRes.error_code.val);
    result.success = false;
  }

  moveToPoseServer.setSucceeded(result);
}

void CarlMoveIt::moveToJointPose(const rail_manipulation_msgs::MoveToJointPoseGoalConstPtr &goal)
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
  rail_manipulation_msgs::MoveToJointPoseResult result;
  armGroup->setPlannerId("arm[KPIECEkConfigDefault]");
  if (goal->planningTime == 0.0)
    armGroup->setPlanningTime(5.0);
  else
    armGroup->setPlanningTime(goal->planningTime);
  armGroup->setStartStateToCurrentState();
  armGroup->setJointValueTarget(jointGoal);
  ROS_INFO("Planning and moving...");
  //armGroup->asyncMove();
  move_group_interface::MoveItErrorCode errorCode = armGroup->move();
  ROS_INFO("Finished plan and move");
  if (errorCode == moveit_msgs::MoveItErrorCodes::SUCCESS)
  {
    ROS_INFO("Succeeded");
    result.success = true;
  }
  else
  {
    ROS_INFO("Failed with MoveIt error code: %d", errorCode.val);
    result.success = false;
  }

  moveToJointPoseServer.setSucceeded(result);
}

bool CarlMoveIt::cartesianPathCallback(rail_manipulation_msgs::CartesianPath::Request &req, rail_manipulation_msgs::CartesianPath::Response &res)
{
  double eefStep = .05;
  double  jumpThreshold = 1.5;
  moveit_msgs::RobotTrajectory finalTraj;

  //convert waypoints to correct frame
  vector<geometry_msgs::Pose> convertedWaypoints;
  for (unsigned int i = 0; i < req.waypoints.size(); i ++)
  {
    geometry_msgs::PoseStamped tempPose;
    tempPose.header.frame_id = armGroup->getPoseReferenceFrame();
    tf.transformPose(armGroup->getPoseReferenceFrame(), req.waypoints[i], tempPose);
    convertedWaypoints.push_back(tempPose.pose);
  }

  //calculate trajectory
  moveit_msgs::RobotTrajectory tempTraj;
  double completion = armGroup->computeCartesianPath(convertedWaypoints, eefStep, jumpThreshold, tempTraj, req.avoidCollisions);
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
        newCompletion = armGroup->computeCartesianPath(convertedWaypoints, eefStep, jumpThreshold, tempTraj, req.avoidCollisions);
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

bool CarlMoveIt::ikCallback(rail_manipulation_msgs::CallIK::Request &req, rail_manipulation_msgs::CallIK::Response &res)
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

moveit_msgs::GetPositionIK::Response CarlMoveIt::callIK(geometry_msgs::PoseStamped pose)
{
  moveit_msgs::GetPositionIK::Request ikReq;
  moveit_msgs::GetPositionIK::Response ikRes;

  //robot_state::RobotStatePtr kinematicState(new robot_state::RobotState(kinematicModel));
  robot_state::RobotStatePtr kinematicState = armGroup->getCurrentState();
  const robot_state::JointModelGroup *jointModelGroup = kinematicState->getRobotModel()->getJointModelGroup("arm");
  //kinematicState->setVariableValues(jointState);

  ikReq.ik_request.group_name = "arm";
  ikReq.ik_request.pose_stamped = pose;
  ikReq.ik_request.ik_link_name = "jaco_link_eef";
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

void CarlMoveIt::recognizedObjectsCallback(const rail_manipulation_msgs::SegmentedObjectList &msg)
{
  //remove previously detected collision objects
  unattachedObjects.clear();  //clear list of unattached scene object names
  vector<string> previousObjects = planningSceneInterface->getKnownObjectNames();
  if (!attachedObjects.empty())
  {
    //don't remove the attached object
    for (unsigned int i = 0; i < previousObjects.size(); i ++)
    {
      for (unsigned int j = 0; j < attachedObjects.size(); j ++)
      if (previousObjects[i] == attachedObjects[j])
      {
        previousObjects.erase(previousObjects.begin() + i);
        i --;
        break;
      }
    }
  }
  planningSceneInterface->removeCollisionObjects(previousObjects);

  {
    boost::recursive_mutex::scoped_lock lock(api_mutex); //lock for the stored objects array

    //store objects
    objectList = msg;

    if (!msg.objects.empty())
    {
      //add all objects to the planning scene
      vector<moveit_msgs::CollisionObject> collisionObjects;
      collisionObjects.resize(msg.objects.size());
      for (unsigned int i = 0; i < collisionObjects.size(); i++)
      {
        //create collision object
        collisionObjects[i].header.frame_id = msg.objects[i].point_cloud.header.frame_id;
        stringstream ss;
        if (msg.objects[i].recognized)
          ss << msg.objects[i].name << i;
        else
          ss << "object" << i;
        //check for name collisions
        for (unsigned int j = 0; j < attachedObjects.size(); j ++)
        {
          if (ss.str() == attachedObjects[j])
            ss << "(2)";
        }
        collisionObjects[i].id = ss.str();
        unattachedObjects.push_back(ss.str());

        //convert point cloud to pcl point cloud
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr objectCloud(new pcl::PointCloud<pcl::PointXYZRGB>);
        pcl::PCLPointCloud2 converter;
        pcl_conversions::toPCL(msg.objects[i].point_cloud, converter);
        pcl::fromPCLPointCloud2(converter, *objectCloud);

        // compute principal direction
        Eigen::Matrix3f covariance;
        Eigen::Vector4f centroid;
        centroid[0] = msg.objects[i].centroid.x;
        centroid[1] = msg.objects[i].centroid.y;
        centroid[2] = msg.objects[i].centroid.z;
        pcl::computeCovarianceMatrixNormalized(*objectCloud, centroid, covariance);
        Eigen::SelfAdjointEigenSolver<Eigen::Matrix3f> eigen_solver(covariance, Eigen::ComputeEigenvectors);
        Eigen::Matrix3f eig_dx = eigen_solver.eigenvectors();
        eig_dx.col(2) = eig_dx.col(0).cross(eig_dx.col(1));

        //move the points to that reference frame
        Eigen::Matrix4f p2w(Eigen::Matrix4f::Identity());
        p2w.block(0, 0, 3, 3) = eig_dx.transpose();
        p2w.block(0, 3, 3, 1) = -1.f * (p2w.block(0, 0, 3, 3) * centroid.head(3));
        pcl::PointCloud<pcl::PointXYZRGB> c_points;
        pcl::transformPointCloud(*objectCloud, c_points, p2w);

        pcl::PointXYZRGB min_pt, max_pt;
        pcl::getMinMax3D(c_points, min_pt, max_pt);
        const Eigen::Vector3f mean_diag = 0.5f * (max_pt.getVector3fMap() + min_pt.getVector3fMap());

        //final transform
        const Eigen::Quaternionf qfinal(eig_dx);
        const Eigen::Vector3f tfinal = eig_dx * mean_diag + centroid.head(3);

        //set object shape
        shape_msgs::SolidPrimitive boundingVolume;
        boundingVolume.type = shape_msgs::SolidPrimitive::BOX;
        boundingVolume.dimensions.resize(3);
        boundingVolume.dimensions[shape_msgs::SolidPrimitive::BOX_X] = max_pt.x - min_pt.x;
        boundingVolume.dimensions[shape_msgs::SolidPrimitive::BOX_Y] = max_pt.y - min_pt.y;
        boundingVolume.dimensions[shape_msgs::SolidPrimitive::BOX_Z] = max_pt.z - min_pt.z;
        collisionObjects[i].primitives.push_back(boundingVolume);
        geometry_msgs::Pose pose;
        pose.position.x = tfinal[0];
        pose.position.y = tfinal[1];
        pose.position.z = tfinal[2];
        pose.orientation.w = qfinal.w();
        pose.orientation.x = qfinal.x();
        pose.orientation.y = qfinal.y();
        pose.orientation.z = qfinal.z();
        collisionObjects[i].primitive_poses.push_back(pose);
        collisionObjects[i].operation = moveit_msgs::CollisionObject::ADD;
      }

      planningSceneInterface->addCollisionObjects(collisionObjects);
    }
  }
}

bool CarlMoveIt::attachClosestSceneObject(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res)
{
  boost::recursive_mutex::scoped_lock lock(api_mutex);  //lock for the stored objects array

  //find the closest point to the gripper pose
  int closest = 0;
  if (objectList.objects.size() == 0)
  {
    ROS_INFO("No scene objects to attach.");
    return true;
  } else if (objectList.objects.size() > 1)
  {
    // find the closest point
    float min = numeric_limits<float>::infinity();
    //geometry_msgs::Vector3 &v = grasp.transform.translation;
    // check each segmented object
    for (size_t i = 0; i < objectList.objects.size(); i++)
    {
      geometry_msgs::TransformStamped eef_transform = tfBuffer.lookupTransform(
              objectList.objects[i].point_cloud.header.frame_id, armGroup->getEndEffectorLink(), ros::Time(0)
      );
      geometry_msgs::Vector3 &v = eef_transform.transform.translation;
      //convert PointCloud2 to PointCloud to access the data easily
      sensor_msgs::PointCloud cloud;
      sensor_msgs::convertPointCloud2ToPointCloud(objectList.objects[i].point_cloud, cloud);
      // check each point in the cloud
      for (size_t j = 0; j < cloud.points.size(); j++)
      {
        // euclidean distance to the point
        float dist = sqrt(
                pow(cloud.points[j].x - v.x, 2) + pow(cloud.points[j].y - v.y, 2) + pow(cloud.points[j].z - v.z, 2)
        );
        if (dist < min)
        {
          min = dist;
          closest = i;
        }
      }
    }

    if (min > SCENE_OBJECT_DST_THRESHOLD)
    {
      ROS_INFO("No scene objects are close enough to the end effector to be attached.");
      return true;
    }
  }

  vector<string> touchLinks;
  touchLinks.push_back("jaco_link_eef");
  touchLinks.push_back("jaco_link_finger_1");
  touchLinks.push_back("jaco_link_finger_2");
  touchLinks.push_back("jaco_link_finger_3");
  touchLinks.push_back("jaco_link_finger_tip_1");
  touchLinks.push_back("jaco_link_finger_tip_2");
  touchLinks.push_back("jaco_link_finger_tip_3");
  touchLinks.push_back("jaco_link_hand");
  armGroup->attachObject(unattachedObjects[closest], armGroup->getEndEffectorLink(), touchLinks);
  attachedObjects.push_back(unattachedObjects[closest]);
  unattachedObjects.erase(unattachedObjects.begin() + closest);

  return true;
}

bool CarlMoveIt::detachSceneObjects(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res)
{
  for (int i = 0; i < attachedObjects.size(); i ++)
  {
    armGroup->detachObject(attachedObjects[i]);
  }
  planningSceneInterface->removeCollisionObjects(attachedObjects);
  attachedObjects.clear();

  return true;
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "carl_moveit_wrapper");

  CarlMoveIt c;

  ros::spin();
}
