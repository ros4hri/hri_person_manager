#include <std_msgs/String.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Float32.h>
#include <hri_msgs/IdsMatch.h>
#include <hri_msgs/IdsList.h>
#include <std_srvs/Empty.h>
#include <ros/ros.h>
#include <geometry_msgs/TransformStamped.h>
#include <tf2_ros/transform_listener.h>
#include <hri/hri.h>
#include <hri/base.h>
#include <thread>
#include <chrono>
#include <map>
#include <array>

#include "person_matcher.h"
#include "managed_person.h"
#include "ros/node_handle.h"

using namespace ros;
using namespace hri;
using namespace std;


// after TIME_TO_DISAPPEAR seconds *without* actively seeing the person, the person tf
// frame is not published anymore.
const float TIME_TO_DISAPPEAR = 10.;  // secs

class PersonManager
{
public:
  PersonManager(NodeHandle& nh, const string& reference_frame)
    : nh(nh), reference_frame(reference_frame), tfListener(tfBuffer)
  {
    tracked_persons_pub = nh.advertise<hri_msgs::IdsList>("/humans/persons/tracked", 1, true);
    known_persons_pub = nh.advertise<hri_msgs::IdsList>("/humans/persons/known", 1, true);


    candidates = nh.subscribe<hri_msgs::IdsMatch>(
        "/humans/candidate_matches", 10, bind(&PersonManager::onCandidateMatch, this, _1));



    ROS_INFO("hri_person_manager ready. Waiting for candidate associations on /humans/candidate_matches");
  }

  bool reset(std_srvs::Empty::Request& req, std_srvs::Empty::Response& res)
  {
    ROS_WARN("Clearing all associations between persons, faces, bodies, voices");
    person_matcher.reset();

    persons.clear();
    previously_tracked.clear();
    anonymous_persons.clear();

    hri_msgs::IdsList persons_list;
    tracked_persons_pub.publish(persons_list);
    known_persons_pub.publish(persons_list);

    return true;
  }

  void onCandidateMatch(hri_msgs::IdsMatchConstPtr match)
  {
    FeatureType type1, type2;
    ID id1, id2;

    // if not overwritten (eg, only one specified id), id2 is 'anonymous'
    type2 = FeatureType::person;
    id2 = hri::ANONYMOUS;

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

    assert(!id1.empty());

    float confidence;
    // if we are describing an 'anonymous' person, set the confidence level to a
    // low value so that any better matching witll take precedence.
    confidence = (id2 == hri::ANONYMOUS) ? 0.01 : match->confidence;

    // if id1 is 'egbd4', id2 becomes 'anonymous_person_' -> 'anonymous_person_egbd4'
    // to create a 'unique' anonymous person for corresponding body part
    if (id2 == hri::ANONYMOUS)
    {
      id2 += id1;
    }

    person_matcher.update({ { id1, type1, id2, type2, confidence } });
  }

  void initialize_person(ID id)
  {
    persons[id] = make_shared<ManagedPerson>(nh, id, tfBuffer, reference_frame);

    // publish an updated list of all known persons
    hri_msgs::IdsList persons_list;

    for (auto const& kv : persons)
    {
      persons_list.ids.push_back(kv.first);
    }
    known_persons_pub.publish(persons_list);
  }

  void remove_person(ID id)
  {
    // delete the person (the ManagedPerson destructor will also shutdown the
    // corresponding topics)
    persons.erase(id);

    // publish an updated list of tracked persons
    hri_msgs::IdsList persons_list;

    for (auto const& kv : persons)
    {
      persons_list.ids.push_back(kv.first);
    }
    tracked_persons_pub.publish(persons_list);
  }

  void publish_persons(chrono::milliseconds elapsed_time)
  {
    auto person_associations = person_matcher.get_all_associations();

    for (const auto& kv : person_associations)
    {
      ID id = kv.first;

      // new person? first, create it (incl its publishers)
      if (persons.find(id) == persons.end())
      {
        initialize_person(id);
      }

      auto person = persons[id];

      ROS_INFO_STREAM("Processing person <" << person->id() << ">");

      auto association = kv.second;

      ID face_id, body_id, voice_id;

      if (association.find(FeatureType::face) != association.end())
      {
        face_id = association.at(face);
      }
      if (association.find(FeatureType::body) != association.end())
      {
        body_id = association.at(body);
      }
      if (association.find(FeatureType::voice) != association.end())
      {
        voice_id = association.at(voice);
      }

      ////////////////////////////////////////////
      // publish the face, body, voice id corresponding to the person
      person->update(face_id, body_id, voice_id, elapsed_time);
    }

    ////////////////////////////////////////////
    // publish the list of currently actively tracked persons
    hri_msgs::IdsList persons_list;
    vector<ID> actively_tracked;

    for (auto const& kv : persons)
    {
      if (kv.second->activelyTracked())
      {
        actively_tracked.push_back(kv.first);
        persons_list.ids.push_back(kv.first);
      }
    }

    if (actively_tracked != previously_tracked)
    {
      tracked_persons_pub.publish(persons_list);
      previously_tracked = actively_tracked;
    }
  }

  void set_threshold(float threshold)
  {
    person_matcher.set_threshold(threshold);
  }

private:
  NodeHandle& nh;

  map<ID, shared_ptr<ManagedPerson>> persons;
  vector<ID> previously_tracked;

  // actively tracked persons (eg, one of face_id, body_id or voice_id is not empty for that person)
  Publisher tracked_persons_pub;
  // known persons: either actively tracked ones, or not tracked anymore (but
  // still known to the robot)
  Publisher known_persons_pub;

  PersonMatcher person_matcher;

  tf2_ros::Buffer tfBuffer;
  tf2_ros::TransformListener tfListener;

  string reference_frame;

  ros::Subscriber candidates;
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "hri_person_manager");
  ros::NodeHandle nh;


  float match_threshold;
  ros::param::param<float>("/humans/match_threshold", match_threshold, 0.5);

  string reference_frame;
  ros::param::param<string>("/humans/reference_frame", reference_frame, "map");


  PersonManager pm(nh, reference_frame);

  pm.set_threshold(match_threshold);

  ros::ServiceServer reset_service =
      nh.advertiseService("/hri_person_manager/reset", &PersonManager::reset, &pm);

  ros::Rate loop_rate(10);

  auto t0 = ros::Time::now();
  while (ros::ok())
  {
    t0 = ros::Time::now();
    loop_rate.sleep();
    ros::spinOnce();

    chrono::nanoseconds elapsed_time((ros::Time::now() - t0).toNSec());
    pm.publish_persons(chrono::duration_cast<chrono::milliseconds>(elapsed_time));
  }

  return 0;
}
