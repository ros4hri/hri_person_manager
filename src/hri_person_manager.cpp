#include "ros/ros.h"
#include "hri/hri.h"
#include <thread>
#include <chrono>

using namespace ros;

int main(int argc, char **argv)
{
  ros::init(argc, argv, "hri_person_manager");
  ros::NodeHandle nh;

  ros::Rate loop_rate(10);

  hri::HRIListener hri_listener;

  while (ros::ok())
  {
    loop_rate.sleep();
    ros::spinOnce();
  }

  return 0;
}
