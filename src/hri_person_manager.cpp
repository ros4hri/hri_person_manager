#include <ros/node_handle.h>
#include <ros/publisher.h>
#include <ros/subscriber.h>
#include <ros/ros.h>
#include <hri/hri.h>
#include <hri/base.h>
#include <thread>
#include <chrono>

#include <hri_msgs/IdsMatch.h>

#include "person_matcher.h"

using namespace ros;
using namespace hri;


int main(int argc, char **argv)
{
  ros::init(argc, argv, "hri_person_manager");
  ros::NodeHandle nh;

  ros::Rate loop_rate(10);

  hri::HRIListener hri_listener;

  PersonMatcher person_matcher(&nh);

  ros::Subscriber candidates = nh.subscribe<hri_msgs::IdsMatch>(
      "/humans/candidate_matches", 1,
      bind(&PersonMatcher::onCandidateMatch, &person_matcher, _1));


  while (ros::ok())
  {
    loop_rate.sleep();
    ros::spinOnce();
  }

  return 0;
}
