#include <ros/ros.h>
#include <control_msgs/FollowJointTrajectoryAction.h>
#include <actionlib/client/simple_action_client.h>
#include <iostream>
#include <fstream>
#include "ReConBot.h"
#include <string>
#include <vector>
#include <cstdlib>
#include <map>
#include <pwd.h>
using namespace std;


int main(int argc,char **argv){
  ros::init(argc, argv, "l_arm_Trajectory_Publisher");
  ros::NodeHandle nh;
  control_msgs::FollowJointTrajectoryGoal goal;
  //ros::Rate loop_rate(5);
  //ros::AsyncSpinner spinner(1);/**<Two spinner are instantiated for managing 2 threats*/
  //spinner.start();
  ReConBotPub Publisher;
  Publisher.topicName = "/l_arm_reconbot_trajectory";
  int arg[] = {4,5,6};

  passwd* pw = getpwuid(getuid());
  std::string path(pw->pw_dir);

  Publisher.motorsState(arg,3);
  Publisher.trajectoryPublisherStart(nh, 1000);
  Publisher.nameSpace = "l_arm_reconbot_controller";
  Publisher.sourceFile = path += "/catkin_ws/src/reconbot/trajectory/l_arm_trajectory.txt";
  goal = Publisher.buildTrajectory();
  Publisher.publisher(goal);

  sleep(3);

  ROS_INFO("Good bye!");
  //spinner.stop();
  //loop_rate.sleep();

  //sleep(2);
  return 0;
}
