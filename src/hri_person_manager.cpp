#include <ros/node_handle.h>
#include <ros/publisher.h>
#include <ros/subscriber.h>
#include <std_msgs/String.h>
#include <ros/ros.h>
#include <hri/hri.h>
#include <hri/base.h>
#include <thread>
#include <chrono>
#include <map>

#include <hri_msgs/IdsMatch.h>

#include "person_matcher.h"

using namespace ros;
using namespace hri;
using namespace std;

void onCandidateMatch(hri_msgs::IdsMatchConstPtr matches)
{
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "hri_person_manager");
  ros::NodeHandle nh;

  ros::Rate loop_rate(10);

  hri::HRIListener hri_listener;

  PersonMatcher person_matcher();

  ros::Subscriber candidates = nh.subscribe<hri_msgs::IdsMatch>(
      "/humans/candidate_matches", 1, bind(&onCandidateMatch, _1));

  map<ID, Publisher> person_pub;

  person_pub["test"] = nh.advertise<std_msgs::String>("/humans/persons/test/face_id", 1, true);

  while (ros::ok())
  {
    loop_rate.sleep();
    ros::spinOnce();
  }

  return 0;
}
