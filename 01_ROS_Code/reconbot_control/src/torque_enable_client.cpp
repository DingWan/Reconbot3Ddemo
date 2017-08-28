#include "ros/ros.h"
#include "reconbot_control/EnableTorque.h"
#include <cstdlib>

int main(int argc, char **argv)
{
  ros::init(argc, argv, "torque_client");
  ros::NodeHandle n;
  ros::ServiceClient client = n.serviceClient<reconbot_control::EnableTorque>("enable_torque");
  reconbot_control::EnableTorque srv;
  srv.request.motor_state = atoll(argv[1]);

  if (client.call(srv))
  {
    ROS_INFO("Done");
  }
  else
  {
    ROS_ERROR("Failed to call service add_two_ints");
    return 1;
  }

  return 0;
}
