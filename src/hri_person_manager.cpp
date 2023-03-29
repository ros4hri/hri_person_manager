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
#include <functional>

#include "hri/body.h"
#include "hri/face.h"
#include "hri/voice.h"
#include "person_matcher.h"
#include "managed_person.h"
#include "ros/node_handle.h"

using namespace ros;
using namespace hri;
using namespace std;


// after TIME_TO_DISAPPEAR seconds *without* actively seeing the person, the person tf
// frame is not published anymore.
const float TIME_TO_DISAPPEAR = 10.;  // secs

enum UpdateType
{
  NEW_FEATURE,
  RELATION,
  REMOVE
};

typedef std::tuple<UpdateType, ID, FeatureType, ID, FeatureType, float> Association;

class PersonManager
{
public:
  PersonManager(NodeHandle& nh, const string& reference_frame,
                bool create_features_from_candidate_matches)
    : _nh(nh)
    , _reference_frame(reference_frame)
    , _create_features_from_candidate_matches(create_features_from_candidate_matches)
    , tfListener(tfBuffer)
  {
    tracked_persons_pub = _nh.advertise<hri_msgs::IdsList>("/humans/persons/tracked", 1, true);
    known_persons_pub = _nh.advertise<hri_msgs::IdsList>("/humans/persons/known", 1, true);
    humans_graph_pub = _nh.advertise<std_msgs::String>("/humans/graph", 1, true);


    candidates = _nh.subscribe<hri_msgs::IdsMatch>(
        "/humans/candidate_matches", 10, bind(&PersonManager::onCandidateMatch, this, _1));

    hri_listener.onFace(bind(&PersonManager::onFace, this, _1));
    hri_listener.onFaceLost(bind(&PersonManager::onFeatureLost, this, _1));
    hri_listener.onBody(bind(&PersonManager::onBody, this, _1));
    hri_listener.onBodyLost(bind(&PersonManager::onFeatureLost, this, _1));
    hri_listener.onVoice(bind(&PersonManager::onVoice, this, _1));
    hri_listener.onVoiceLost(bind(&PersonManager::onFeatureLost, this, _1));


    ROS_INFO("hri_person_manager ready. Waiting for candidate associations on /humans/candidate_matches or updates on /humans/*/tracked");
  }

  bool reset(std_srvs::Empty::Request& req, std_srvs::Empty::Response& res)
  {
    ROS_WARN("Clearing all associations between persons, faces, bodies, voices");
    person_matcher.reset();

    persons.clear();
    previously_known.clear();
    previously_tracked.clear();

    // publish an empty list of tracked/known persons
    hri_msgs::IdsList persons_list;
    tracked_persons_pub.publish(persons_list);
    known_persons_pub.publish(persons_list);

    return true;
  }

  void onCandidateMatch(hri_msgs::IdsMatchConstPtr match)
  {
    FeatureType type1, type2;
    ID id1, id2;

    id1 = match->id1;

    if (id1.empty())
    {
      ROS_ERROR("received an empty id for id1");
      return;
    }

    id2 = match->id2;

    if (id2.empty())
    {
      ROS_ERROR("received an empty id for id2");
      return;
    }

    if (id1 == id2)
    {
      ROS_ERROR("candidate_matches with identical id1 and id2. Skipping.");
      return;
    }

    switch (match->id1_type)
    {
      case hri_msgs::IdsMatch::PERSON:
        type1 = FeatureType::person;
        break;

      case hri_msgs::IdsMatch::FACE:
        type1 = FeatureType::face;
        break;

      case hri_msgs::IdsMatch::BODY:
        type1 = FeatureType::body;
        break;

      case hri_msgs::IdsMatch::VOICE:
        type1 = FeatureType::voice;
        break;

      default:
        ROS_ERROR_STREAM("received an invalid type for id1: " << match->id1_type);
        return;
    }

    switch (match->id2_type)
    {
      case hri_msgs::IdsMatch::PERSON:
        type2 = FeatureType::person;
        break;

      case hri_msgs::IdsMatch::FACE:
        type2 = FeatureType::face;
        break;

      case hri_msgs::IdsMatch::BODY:
        type2 = FeatureType::body;
        break;

      case hri_msgs::IdsMatch::VOICE:
        type2 = FeatureType::voice;
        break;

      default:
        ROS_ERROR_STREAM("received an invalid type for id2: " << match->id2_type);
        return;
    }


    {
      updates.push_back({ RELATION, id1, type1, id2, type2, match->confidence });
    }
  }

  void onFace(FaceWeakConstPtr face)
  {
    ID id;

    if (auto face_ptr = face.lock())
    {
      id = face_ptr->id();

      if (id.size() == 0)
      {
        ROS_ERROR("got invalid new face: empty id! skipping");
        return;
      }

      // create a new node in the graph
      updates.push_back({ NEW_FEATURE, id, FeatureType::face, id, FeatureType::face, 0. });
    }
  }

  void onBody(BodyWeakConstPtr body)
  {
    ID id;

    if (auto body_ptr = body.lock())
    {
      id = body_ptr->id();

      if (id.size() == 0)
      {
        ROS_ERROR("got invalid new body: empty id! skipping");
        return;
      }

      // create a new node in the graph
      updates.push_back({ NEW_FEATURE, id, FeatureType::body, id, FeatureType::body, 0. });
    }
  }

  void onVoice(VoiceWeakConstPtr voice)
  {
    ID id;

    if (auto voice_ptr = voice.lock())
    {
      id = voice_ptr->id();

      if (id.size() == 0)
      {
        ROS_ERROR("got invalid new voice: empty id! skipping");
        return;
      }

      // create a new node in the graph
      updates.push_back({ NEW_FEATURE, id, FeatureType::voice, id, FeatureType::voice, 0. });
    }
  }


  void onFeatureLost(ID id)
  {
    if (id.size() == 0)
    {
      ROS_ERROR("a feature was removed, but empty id! skipping");
      return;
    }

    updates.push_back({ REMOVE, id, FeatureType::invalid, id, FeatureType::invalid, 0. });
  }


  void initialize_person(ID id)
  {
    persons[id] = make_shared<ManagedPerson>(_nh, id, tfBuffer, _reference_frame);

    publish_known_persons();
  }

  void publish_known_persons()
  {
    // publish an updated list of all known persons
    hri_msgs::IdsList persons_list;
    vector<ID> known;

    for (auto const& kv : persons)
    {
      persons_list.ids.push_back(kv.first);
      known.push_back(kv.first);
    }

    if (known != previously_known)
    {
      persons_list.header.stamp = ros::Time::now();
      known_persons_pub.publish(persons_list);
      previously_known = known;
    }
  }


  void publish_persons(chrono::milliseconds elapsed_time)
  {
    ///////////////////////////////////
    // first: housekeeping -> update the graph with all the last changes
    UpdateType update_type;
    ID id1, id2;
    FeatureType type1, type2;
    float likelihood;

    if (!updates.empty())
    {
      ROS_INFO_STREAM("Updating graph:");
    }
    for (auto u : updates)
    {
      std::tie(update_type, id1, type1, id2, type2, likelihood) = u;

      switch (update_type)
      {
        case NEW_FEATURE:
        {
          ROS_INFO_STREAM("- New feature: " << id1 << " (" << type1 << ")");
          // create a single 'orphan' node. At the end of the next update cycle,
          // this orphan node will be associated to an anonymous_persons if it
          // is not linked to any other node
          person_matcher.update({ { id1, type1, id2, type2, 1.0 } });
        }
        break;

        case REMOVE:
        {
          ROS_INFO_STREAM("- Remove ID: " << id1);
          person_matcher.erase(id1);
        }
        break;

        case RELATION:
        {
          ROS_INFO_STREAM("- Update relation: " << id1 << " (" << type1 << ") <--> " << id2 << " ("
                                                << type2 << "); likelihood=" << likelihood);
          person_matcher.update({ { id1, type1, id2, type2, likelihood } },
                                _create_features_from_candidate_matches);
        }
        break;
      }
    }
    updates.clear();
    //////////////////////////

    //////////////////////////
    //   MAIN ALGORITHM     //
    //////////////////////////
    auto person_associations = person_matcher.get_all_associations();
    //////////////////////////

    ////////////////////////////////////////////
    // if someone needs it, publish the graphviz model of the relationship graph
    if (humans_graph_pub.getNumSubscribers() > 0)
    {
      std_msgs::String graphviz;
      graphviz.data = person_matcher.get_graphviz();
      humans_graph_pub.publish(graphviz);
    }

    ////////////////////////////////////////////
    // go over all the persons in the graph, and process their
    // associations

    for (const auto& kv : person_associations)
    {
      ID id = kv.first;

      // new person? first, create it (incl its publishers)
      if (!persons.count(id))
      {
        initialize_person(id);
      }

      auto person = persons.at(id);

      const auto& association = kv.second;

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
    // if an anonymous person is *not* present anymore in the associations, it
    // has disappeared, and must be removed from the system.
    vector<ID> to_delete;
    for (auto const& kv : persons)
    {
      if (kv.second->anonymous() && !person_associations.count(kv.first))
      {
        to_delete.push_back(kv.first);
      }
    }
    for (const auto& p : to_delete)
    {
      persons.erase(p);
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
      persons_list.header.stamp = ros::Time::now();
      tracked_persons_pub.publish(persons_list);
      previously_tracked = actively_tracked;
    }

    publish_known_persons();
  }

  void set_threshold(float threshold)
  {
    person_matcher.set_threshold(threshold);
  }

private:
  NodeHandle& _nh;

  map<ID, shared_ptr<ManagedPerson>> persons;
  vector<ID> previously_known, previously_tracked;

  vector<Association> updates;

  HRIListener hri_listener;

  // actively tracked persons (eg, one of face_id, body_id or voice_id is not empty for that person)
  Publisher tracked_persons_pub;
  // known persons: either actively tracked ones, or not tracked anymore (but
  // still known to the robot)
  Publisher known_persons_pub;

  Publisher humans_graph_pub;

  PersonMatcher person_matcher;

  tf2_ros::Buffer tfBuffer;
  tf2_ros::TransformListener tfListener;

  string _reference_frame;

  ros::Subscriber candidates;
  bool _create_features_from_candidate_matches;
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "hri_person_manager");
  ros::NodeHandle nh;


  float match_threshold;
  ros::param::param<float>("/humans/match_threshold", match_threshold, 0.5);

  string reference_frame;
  ros::param::param<string>("/humans/reference_frame", reference_frame, "map");

  bool create_features_from_candidate_matches;
  ros::param::param<bool>("~features_from_matches", create_features_from_candidate_matches, false);

  if (create_features_from_candidate_matches)
  {
    ROS_INFO("~features_from_matches: True. New features (faces, bodies, voices) will be created if referred to in /humans/candidate_matches, even if not explicitely tracked. While this is the correct REP-155 semantic, it might cause unexpected 'ghost' features if the feature matchers publish candidate matches even after the feature is not tracked anymore.");
  }
  else
  {
    ROS_INFO("~features_from_matches: False (default). New features (faces, bodies, voices) will be only created if explicitely tracked (ie appear in /humans/*/tracked)");
  }

  PersonManager pm(nh, reference_frame, create_features_from_candidate_matches);

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
