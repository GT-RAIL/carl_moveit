#include <carl_moveit/high_level_actions.h>

using namespace std;

HighLevelActions::HighLevelActions() :
    armClient("carl_moveit_wrapper/common_actions/arm_action"),
    pickupClient("carl_moveit_wrapper/common_actions/pickup"),
    storeClient("carl_moveit_wrapper/common_actions/store"),
    obtainObjectServer(n, "carl_moveit_wrapper/high_level_actions/obtain_object", boost::bind(&HighLevelActions::executeObtainObject, this, _1), false)
{
  recognizedObjectsCounter = 0;

  recognizedObjectsSubscriber = n.subscribe("/object_recognition_listener/recognized_objects", 1, &HighLevelActions::recognizedObjectsCallback, this);;

  segmentClient = n.serviceClient<std_srvs::Empty>("/rail_segmentation/segment");

  //start action server
  obtainObjectServer.start();
}

void HighLevelActions::executeObtainObject(const carl_moveit::ObtainObjectGoalConstPtr &goal)
{
  carl_moveit::ObtainObjectFeedback feedback;
  carl_moveit::ObtainObjectResult result;
  result.success = false;

  //retract arm
  carl_moveit::ArmGoal retractGoal;
  retractGoal.action = carl_moveit::ArmGoal::RETRACT;
  armClient.sendGoal(retractGoal);
  bool completed = armClient.waitForResult(ros::Duration(20.0));
  bool succeeded = (armClient.getState() == actionlib::SimpleClientGoalState::SUCCEEDED);
  bool success = armClient.getResult()->success;
  if (!completed || !succeeded || !success)
  {
    ROS_INFO("Could not retract arm for segmentation.");
    obtainObjectServer.setSucceeded(result, "Could not retract arm during search.");
    return;
  }

  //perform recognition
  feedback.message = "Attempting to segment the surface.";
  obtainObjectServer.publishFeedback(feedback);
  std_srvs::Empty segment;
  recognizedObjectsCounter = 0;
  if (!segmentClient.call(segment))
  {
    ROS_INFO("Could not call segment service.");
    obtainObjectServer.setSucceeded(result, "Could not call segment service.");
    return;
  }

  //spin and wait
  feedback.message = "Waiting for recognition results.";
  ROS_INFO("Waiting on recognition...");
  obtainObjectServer.publishFeedback(feedback);
  bool finished = false;
  while (!finished)
  {
    ROS_INFO("Recognized objects counter: %d", recognizedObjectsCounter);
    {
      boost::mutex::scoped_lock lock(recognizedObjectsMutex);
      finished = recognizedObjectsCounter == 2;
    }
  }

  //pickup the specified object
  string objectName = boost::to_upper_copy(goal->object_name);
  bool pickupSucceeded = false;
  ROS_INFO("Looking for object %s...", objectName.c_str());
  for (unsigned int i = 0; i < recognizedObjects.objects.size(); i ++)
  {
    if (recognizedObjects.objects[i].name == objectName)
    {
      ROS_INFO("Found object! Attempting pickup...");
      carl_moveit::PickupGoal pickupGoal;
      pickupGoal.lift = goal->lift;
      pickupGoal.verify = goal->verify;

      for (unsigned int j = 0; j < recognizedObjects.objects[i].grasps.size(); j ++)
      {
        ROS_INFO("ATTEMPTING PICKUP WITH GRASP %d", j);
        pickupGoal.pose = recognizedObjects.objects[i].grasps[j].grasp_pose;
        pickupClient.sendGoal(pickupGoal);
        pickupClient.waitForResult(ros::Duration(30.0));

        carl_moveit::PickupResultConstPtr pickupResult = pickupClient.getResult();
        if (!pickupResult->success)
        {
          ROS_INFO("PICKUP FAILED, moving on to a new grasp...");
        }
        else
        {
          ROS_INFO("PICKUP SUCCEEDED");
          pickupSucceeded = true;
          break;
        }
      }

      if (pickupSucceeded)
        break;
    }
  }

  if (!pickupSucceeded)
  {
    ROS_INFO("Could not find or pickup the specified object.");
    obtainObjectServer.setSucceeded(result, "Could not find or pickup the specified object.");
    return;
  }

  //Store object on robot
  carl_moveit::StoreGoal storeGoal;
  storeClient.sendGoal(storeGoal);
  completed = armClient.waitForResult(ros::Duration(30.0));
  succeeded = (armClient.getState() == actionlib::SimpleClientGoalState::SUCCEEDED);
  success = armClient.getResult()->success;
  if (!completed || !succeeded || !success)
  {
    ROS_INFO("Could not store object.");
    obtainObjectServer.setSucceeded(result, "Could not store object.");
    return;
  }

  ROS_INFO("Finished obtaining object successfully!");
  result.success = true;
  obtainObjectServer.setSucceeded(result);
}

void HighLevelActions::recognizedObjectsCallback(const rail_manipulation_msgs::SegmentedObjectList &objects)
{
  boost::mutex::scoped_lock lock(recognizedObjectsMutex);

  recognizedObjects = objects;
  recognizedObjectsCounter++;
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "carl_moveit_high_level_actions");

  HighLevelActions hla;

  ros::spin();
}
