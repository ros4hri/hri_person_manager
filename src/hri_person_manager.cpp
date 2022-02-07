#include <std_msgs/String.h>
#include <hri_msgs/IdsMatch.h>
#include <hri_msgs/IdsList.h>
#include <ros/ros.h>
#include <hri/hri.h>
#include <hri/base.h>
#include <thread>
#include <chrono>
#include <map>
#include <array>

#include "person_matcher.h"

using namespace ros;
using namespace hri;
using namespace std;

// the 3 publishers are, in order: face_id, body_id, voice_id
map<ID, array<Publisher, 3>> persons_pub;


Publisher tracked_persons_pub;

PersonMatcher person_matcher;

bool dirty;

void onCandidateMatch(hri_msgs::IdsMatchConstPtr match)
{
  FeatureType type1, type2;
  ID id1, id2;


  if (!match->person_id.empty())
  {
    type1 = FeatureType::person;
    id1 = match->person_id;
  }

  if (!match->face_id.empty())
  {
    if (!id1.empty())
    {
      type2 = FeatureType::face;
      id2 = match->face_id;
    }
    else
    {
      type1 = FeatureType::face;
      id1 = match->face_id;
    }
  }

  if (!match->body_id.empty())
  {
    if (!id1.empty())
    {
      type2 = FeatureType::body;
      id2 = match->body_id;
    }
    else
    {
      type1 = FeatureType::body;
      id1 = match->body_id;
    }
  }

  if (!match->voice_id.empty())
  {
    if (!id1.empty())
    {
      type2 = FeatureType::voice;
      id2 = match->voice_id;
    }
    else
    {
      type1 = FeatureType::voice;
      id1 = match->voice_id;
    }
  }

  person_matcher.update({ { id1, type1, id2, type2, match->confidence } });

  dirty = true;
}

void initialize_person_publishers(NodeHandle& nh, ID id)
{
  persons_pub[id] = { {
      nh.advertise<std_msgs::String>(string("/humans/persons/") + id + "/face_id", 1, true),
      nh.advertise<std_msgs::String>(string("/humans/persons/") + id + "/body_id", 1, true),
      nh.advertise<std_msgs::String>(string("/humans/persons/") + id + "/voice_id", 1, true),
  } };

  // publish an updated list of tracked persons
  hri_msgs::IdsList persons_list;

  for (auto const& kv : persons_pub)
  {
    persons_list.ids.push_back(kv.first);
  }
  tracked_persons_pub.publish(persons_list);
}

void remove_person(ID id)
{
  // publish an updated list of tracked persons
  hri_msgs::IdsList persons_list;

  for (auto const& kv : persons_pub)
  {
    persons_list.ids.push_back(kv.first);
  }
  tracked_persons_pub.publish(persons_list);


  // shutdown the person's sub-topics and delete the person
  for (auto& pub : persons_pub.at(id))
  {
    pub.shutdown();
  }
  persons_pub.erase(id);
}

void publish_persons(NodeHandle& nh)
{
  auto persons = person_matcher.get_all_associations();

  for (const auto& kv : persons)
  {
    ID id = kv.first;
    auto association = kv.second;

    // new person? first, create the publishers
    if (persons_pub.find(id) == persons_pub.end())
    {
      initialize_person_publishers(nh, id);
    }

    std_msgs::String msg;
    if (association.find(FeatureType::face) != association.end())
    {
      msg.data = association.at(face);
      persons_pub[id][0].publish(msg);
    }
    if (association.find(FeatureType::body) != association.end())
    {
      msg.data = association.at(body);
      persons_pub[id][1].publish(msg);
    }
    if (association.find(FeatureType::voice) != association.end())
    {
      msg.data = association.at(voice);
      persons_pub[id][2].publish(msg);
    }
  }

  dirty = false;
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "hri_person_manager");
  ros::NodeHandle nh;


  float match_threshold;
  ros::param::param<float>("/humans/match_threshold", match_threshold, 0.5);
  person_matcher.set_threshold(match_threshold);

  ros::Rate loop_rate(10);

  HRIListener hri_listener;

  tracked_persons_pub = nh.advertise<hri_msgs::IdsList>("/humans/persons/tracked", 1, true);

  dirty = true;

  ros::Subscriber candidates =
      nh.subscribe<hri_msgs::IdsMatch>("/humans/candidate_matches", 1, onCandidateMatch);


  while (ros::ok())
  {
    if (dirty)
      publish_persons(nh);
    loop_rate.sleep();
    ros::spinOnce();
  }

  return 0;
}
