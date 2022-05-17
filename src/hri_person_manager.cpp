#include <std_msgs/String.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Float32.h>
#include <hri_msgs/IdsMatch.h>
#include <hri_msgs/IdsList.h>
#include <ros/ros.h>
#include <geometry_msgs/TransformStamped.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/transform_broadcaster.h>
#include <hri/hri.h>
#include <hri/base.h>
#include <thread>
#include <chrono>
#include <map>
#include <array>

#include "person_matcher.h"
#include "ros/node_handle.h"

using namespace ros;
using namespace hri;
using namespace std;

const string ANONYMOUS("anonymous");

// after TIME_TO_DISAPPEAR seconds *without* actively seeing the person, the person tf
// frame is not published anymore.
const float TIME_TO_DISAPPEAR = 10.;  // secs

class PersonManager
{
public:
  PersonManager(NodeHandle& nh) : nh(nh), tfListener(tfBuffer)
  {
    tracked_persons_pub = nh.advertise<hri_msgs::IdsList>("/humans/persons/tracked", 1, true);
    known_persons_pub = nh.advertise<hri_msgs::IdsList>("/humans/persons/known", 1, true);

    dirty = true;

    candidates = nh.subscribe<hri_msgs::IdsMatch>(
        "/humans/candidate_matches", 1, bind(&PersonManager::onCandidateMatch, this, _1));
  }

  void onCandidateMatch(hri_msgs::IdsMatchConstPtr match)
  {
    FeatureType type1, type2;
    ID id1, id2;

    // if not overwritten (eg, only one specified id), id2 is 'anonymous'
    type2 = FeatureType::person;
    id2 = ANONYMOUS;

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

    float confidence;
    // if we are describing an 'anonymous' person, set the confidence level to a
    // low value so that any better matching witll take precedence.
    confidence = (id2 == ANONYMOUS) ? 0.01 : match->confidence;

    person_matcher.update({ { id1, type1, id2, type2, confidence } });

    dirty = true;
  }

  void initialize_person_publishers(ID id)
  {
    persons_pub[id] = { {
        nh.advertise<std_msgs::String>(string("/humans/persons/") + id + "/face_id", 1, true),
        nh.advertise<std_msgs::String>(string("/humans/persons/") + id + "/body_id", 1, true),
        nh.advertise<std_msgs::String>(string("/humans/persons/") + id + "/voice_id", 1, true),
        nh.advertise<std_msgs::Bool>(string("/humans/persons/") + id + "/anonymous", 1, true),
        nh.advertise<std_msgs::Float32>(string("/humans/persons/") + id + "/location_confidence", 1),
    } };

    // publish an updated list of all known persons
    hri_msgs::IdsList persons_list;

    for (auto const& kv : persons_pub)
    {
      persons_list.ids.push_back(kv.first);
    }
    known_persons_pub.publish(persons_list);
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

  void publish_persons()
  {
    // if (!dirty)
    //  return;

    auto persons = person_matcher.get_all_associations();

    for (const auto& kv : persons)
    {
      ID id = kv.first;
      auto association = kv.second;

      bool anonymous = (id == ANONYMOUS) ? true : false;

      ID face_id, body_id, voice_id;

      // new person? first, create the publishers
      if (persons_pub.find(id) == persons_pub.end())
      {
        initialize_person_publishers(id);
      }

      if (anonymous)
      {
        // TODO id =
      }
      else
      {
        std_msgs::Bool msg;
        msg.data = false;
        persons_pub[id][3].publish(msg);
      }

      actively_tracked_persons[id] = false;

      ////////////////////////////////////////////
      // publish the face, body, voice id corresponding to the person

      std_msgs::String msg;
      if (association.find(FeatureType::face) != association.end())
      {
        face_id = association.at(face);
        msg.data = face_id;
        persons_pub[id][0].publish(msg);
        actively_tracked_persons[id] = true;
      }
      if (association.find(FeatureType::body) != association.end())
      {
        body_id = association.at(body);
        msg.data = body_id;
        persons_pub[id][1].publish(msg);
        actively_tracked_persons[id] = true;
      }
      if (association.find(FeatureType::voice) != association.end())
      {
        voice_id = association.at(voice);
        msg.data = voice_id;
        persons_pub[id][2].publish(msg);
        actively_tracked_persons[id] = true;
      }

      /////////////////////////////////////////////
      // publish TF frame of the person


      string person_frame = string("person_") + id;

      // TODO: make that configurable
      string reference_frame = string("map");

      string target_frame;

      if (!face_id.empty())
      {
        target_frame = string("face_") + face_id;
      }
      else if (!body_id.empty())
      {
        target_frame = string("head_") + body_id;
      }
      else if (!voice_id.empty())
      {
        target_frame = string("voice_") + voice_id;
      }

      float location_confidence = 0.;

      if (!target_frame.empty())
      {
        geometry_msgs::TransformStamped transform;
        try
        {
          ROS_INFO_STREAM(" - get transform " << reference_frame << " <-> " << target_frame);
          transform = tfBuffer.lookupTransform(reference_frame, target_frame, ros::Time(0));

          transform.header.stamp = ros::Time::now();
          transform.child_frame_id = person_frame;

          persons_last_transform[id] = make_pair(transform, ros::Time::now());

          br.sendTransform(transform);
          location_confidence = 1.;
        }
        catch (tf2::TransformException ex)
        {
          ROS_WARN("%s", ex.what());
        }
      }
      else
      {
        if (persons_last_transform.count(id) != 1)
        {
          ROS_DEBUG_STREAM(" - No face or body TF frame published for person "
                           << id << ". Can not yet broadcast frame " << person_frame << ".");
        }
        else
        {
          auto time_last_transform = persons_last_transform[id].second;


          location_confidence = computeLocationConfidence(time_last_transform);

          if (location_confidence > 0.0)
          {
            ROS_INFO_STREAM(" - no transform available. Using last known transform");

            auto transform = persons_last_transform[id].first;
            transform.header.stamp = ros::Time::now();
            br.sendTransform(transform);
          }
          else
          {
            ROS_INFO_STREAM(" - not seen for more than "
                            << TIME_TO_DISAPPEAR << "s. Not publishing tf frame anymore.");
          }
        }
      }

      std_msgs::Float32 confidence;
      confidence.data = location_confidence;
      persons_pub[id][4].publish(confidence);
    }
    // publish the list of currently actively tracked persons
    hri_msgs::IdsList persons_list;
    for (auto const& kv : actively_tracked_persons)
    {
      if (kv.second)
      {
        persons_list.ids.push_back(kv.first);
      }
    }
    tracked_persons_pub.publish(persons_list);

    dirty = false;
  }

  float computeLocationConfidence(ros::Time last_seen)
  {
    auto time_since_last_seen = (ros::Time::now() - last_seen).toSec();

    if (time_since_last_seen > TIME_TO_DISAPPEAR)
    {
      return 0;
    }

    return max(0., 1 - time_since_last_seen / TIME_TO_DISAPPEAR);
  }

  void set_threshold(float threshold)
  {
    person_matcher.set_threshold(threshold);
  }

private:
  NodeHandle& nh;

  // the 4 publishers are, in order: face_id, body_id, voice_id, anonymous, location_confidence
  map<ID, array<Publisher, 5>> persons_pub;
  map<ID, bool> actively_tracked_persons;

  map<ID, pair<geometry_msgs::TransformStamped, ros::Time>> persons_last_transform;

  HRIListener hri_listener;

  // actively tracked persons (eg, one of face_id, body_id or voice_id is not empty for that person)
  Publisher tracked_persons_pub;
  // known persons: either actively tracked ones, or not tracked anymore (but
  // still known to the robot)
  Publisher known_persons_pub;

  PersonMatcher person_matcher;

  tf2_ros::Buffer tfBuffer;
  tf2_ros::TransformListener tfListener;
  tf2_ros::TransformBroadcaster br;

  bool dirty;
  ros::Subscriber candidates;
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "hri_person_manager");
  ros::NodeHandle nh;


  float match_threshold;
  ros::param::param<float>("/humans/match_threshold", match_threshold, 0.5);

  PersonManager pm(nh);

  pm.set_threshold(match_threshold);

  ros::Rate loop_rate(10);


  while (ros::ok())
  {
    pm.publish_persons();
    loop_rate.sleep();
    ros::spinOnce();
  }

  return 0;
}
