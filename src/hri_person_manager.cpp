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
  PersonManager(NodeHandle& nh, const string& reference_frame)
    : _nh(nh), _reference_frame(reference_frame), tfListener(tfBuffer)
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

    id1 = match->id1;

    if (id1.empty())
    {
      ROS_ERROR("received an empty id for id1");
      return;
    }

    id2 = match->id2;

    if (id2.empty() && match->id2_type != hri_msgs::IdsMatch::UNSET)
    {
      ROS_ERROR_STREAM("received an empty id for id2, with type set to " << match->id2_type);
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

      case hri_msgs::IdsMatch::UNSET:
        type2 = FeatureType::person;
        id2 = hri::ANONYMOUS;
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
      // if already associated, do nothing
      if (associated_faces.count(id) != 0)
      {
        return;
      }

      // ...otherwise, create a new node in the graph
      updates.push_back({ NEW_FEATURE, id, FeatureType::face, id, FeatureType::face, 0. });
    }
  }

  void onBody(BodyWeakConstPtr body)
  {
    ID id;

    if (auto body_ptr = body.lock())
    {
      id = body_ptr->id();

      // if already associated, do nothing
      if (associated_bodies.count(id) != 0)
      {
        return;
      }

      // ...otherwise, create a new node in the graph
      updates.push_back({ NEW_FEATURE, id, FeatureType::body, id, FeatureType::body, 0. });
    }
  }

  void onVoice(VoiceWeakConstPtr voice)
  {
    ID id;

    if (auto voice_ptr = voice.lock())
    {
      id = voice_ptr->id();

      // if already associated, do nothing
      if (associated_voices.count(id) != 0)
      {
        return;
      }

      // ...otherwise, create a new node in the graph
      updates.push_back({ NEW_FEATURE, id, FeatureType::voice, id, FeatureType::voice, 0. });
    }
  }


  void onFeatureLost(ID id)
  {
    updates.push_back({ REMOVE, id, FeatureType::person, id, FeatureType::person, 0. });
  }

  void update(ID id1, FeatureType type1, ID id2, FeatureType type2, float confidence)
  {
    person_matcher.update({ { id1, type1, id2, type2, confidence } });

    // after an update, we might have new orphaned nodes (if the update sets a confidence of 0)
    if (confidence == 0.0)
    {
      auto removed_persons = person_matcher.clear_orphans();

      for (auto const& id : removed_persons)
      {
        remove_person(id);
      }
    }


    // TODO:
    // If you have the following associations:
    // f1 -> anon_p1
    // b2 -> f1
    // and you add:
    // b2 -> p2
    // then anon_p1 should be removed (since f1 is now indirectly associated with p2)
    //
    // This is not handled yet.
  }

  void initialize_person(ID id)
  {
    persons[id] = make_shared<ManagedPerson>(_nh, id, tfBuffer, _reference_frame);

    publishKnownPersons();
  }

  void publishKnownPersons()
  {
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
    if (!persons.count(id))
    {
      return;
    }

    // unlike anonymous persons, non-anonymous person can not be removed -- they
    // can merely become untracked.
    if (persons[id]->anonymous())
    {
      // delete the person (the ManagedPerson destructor will also shutdown the
      // corresponding topics)
      persons.erase(id);

      anonymous_persons.erase(id);
    }

    // publish an updated list of known/tracked persons
    hri_msgs::IdsList persons_list;
    for (auto const& kv : persons)
    {
      if (kv.second->activelyTracked())
      {
        persons_list.ids.push_back(kv.first);
      }
    }

    tracked_persons_pub.publish(persons_list);

    publishKnownPersons();
  }

  void publish_persons(chrono::milliseconds elapsed_time)
  {
    ///////////////////////////////////
    // first: housekeeping -> update the graph with all the last changes
    UpdateType update_type;
    ID id1, id2;
    FeatureType type1, type2;
    float p;

    if (!updates.empty())
    {
      ROS_INFO_STREAM("Updating graph:");
    }
    for (auto u : updates)
    {
      std::tie(update_type, id1, type1, id2, type2, p) = u;

      switch (update_type)
      {
        case NEW_FEATURE:
        {
          ROS_INFO_STREAM("- New feature: " << id1 << " (" << type1 << ")");
          // create a single 'orphan' node. At the end of the next update cycle,
          // this orphan node will be associated to an anonymous_persons if it
          // is not linked to any other node
          update(id1, type1, id1, type1, 0.);
        }
        break;
        case REMOVE:
        {
          ROS_INFO_STREAM("- Remove ID: " << id1);

          // while erasing the id id1, the person matcher might create
          // orphans that are as well deleted by the person_matcher.
          // The ids of the orphans that happened to be persons
          // are returned by PersonMatcher::erase to then remove these
          // persons from eg /humans/persons/tracked
          auto removed_persons = person_matcher.erase(id1);

          for (auto const& id : removed_persons)
          {
            remove_person(id);
          }
        }
        break;
        case RELATION:
          ROS_INFO_STREAM("- Update relation: " << id1 << " (" << type1 << ") <--> "
                                                << id2 << " (" << type2 << "); p=" << p);
          update(id1, type1, id2, type2, p);
          break;
      }
    }
    updates.clear();
    //////////////////////////

    auto res = person_matcher.get_all_associations();
    auto person_associations = res.first;
    auto orphan_features = res.second;

    std_msgs::String graphviz;
    graphviz.data = person_matcher.get_graphviz();
    humans_graph_pub.publish(graphviz);

    associated_faces.clear();
    associated_bodies.clear();
    associated_voices.clear();

    for (const auto& kv : person_associations)
    {
      ID id = kv.first;

      // new person? first, create it (incl its publishers)
      if (persons.find(id) == persons.end())
      {
        initialize_person(id);
      }

      auto person = persons[id];

      auto association = kv.second;

      ID face_id, body_id, voice_id;

      if (association.find(FeatureType::face) != association.end())
      {
        face_id = association.at(face);
        associated_faces.insert(face_id);
      }
      if (association.find(FeatureType::body) != association.end())
      {
        body_id = association.at(body);
        associated_bodies.insert(body_id);
      }
      if (association.find(FeatureType::voice) != association.end())
      {
        voice_id = association.at(voice);
        associated_voices.insert(voice_id);
      }

      ////////////////////////////////////////////
      // publish the face, body, voice id corresponding to the person
      person->update(face_id, body_id, voice_id, elapsed_time);
    }

    // for each orphan feature, we create an anonymous person
    for (auto feature : orphan_features)
    {
      auto id = feature.first;
      auto type = feature.second;
      ID face_id, body_id, voice_id;
      switch (type)
      {
        case face:
          face_id = id;
          break;
        case body:
          body_id = id;
          break;
        case voice:
          voice_id = id;
          break;
        default:
          assert(false)
      }

      if (!anonymous_persons.count(id))
      {
        anonymous_persons.insert(id1);

        initialize_person(hri::ANONYMOUS + id);
      }
      persons[hri::ANONYMOUS + id]->update(face_id, body_id, voice_id, elapsed_time);
    }


    // now, remove the anonymous persons that are not needed anymore
    for (const auto id : anonymous_persons)
    {
      bool found = false;
      for (auto feature : orphan_features)
      {
        if (feature.first == id)
        {
          found = true;
          break;
        }
      }
      if (!found)
      {
        // the feature the anonymous person was associated to is not
        // orphan anymore or does not exist anymore
        // -> remove the anonymous person
        ROS_WARN_STREAM("removing anonymous person "
                        << hri::ANONYMOUS + id2 << " as it is not anonymous anymore");
        anonymous_persons.erase(id);
        remove_person(id);
      }
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
  NodeHandle& _nh;

  map<ID, shared_ptr<ManagedPerson>> persons;
  vector<ID> previously_tracked;
  set<ID> anonymous_persons;

  vector<Association> updates;

  // hold the list of faces/bodies/voices that are already associated to a person
  // (so that we do not create un-needed anonymous persons)
  set<ID> associated_faces;
  set<ID> associated_bodies;
  set<ID> associated_voices;

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
