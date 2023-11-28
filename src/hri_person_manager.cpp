#include <std_msgs/String.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Float32.h>
#include <hri_msgs/IdsMatch.h>
#include <hri_msgs/IdsList.h>
#include <std_srvs/Empty.h>
#include <ros/ros.h>
#include <geometry_msgs/TransformStamped.h>
#include <tf2_ros/transform_listener.h>
#include <diagnostic_msgs/DiagnosticStatus.h>
#include <diagnostic_updater/diagnostic_updater.h>
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
  PersonManager(
    NodeHandle & nh,
    const string & reference_frame,
    const string & robot_reference_frame,
    bool create_features_from_candidate_matches,
    float distance_personal_space,
    float distance_social_space,
    float distance_public_space)
  : _nh(nh),
    _reference_frame(reference_frame),
    _robot_reference_frame(robot_reference_frame),
    _distance_personal_space(distance_personal_space),
    _distance_social_space(distance_social_space),
    _distance_public_space(distance_public_space),
    _create_features_from_candidate_matches(create_features_from_candidate_matches),
    tfListener(tfBuffer),
    diag_updater(
      nh, ros::NodeHandle("~"),
      " Social perception")               // adding initial space in 'node_name' string
                                          // since diagnostic_updater removes the first char
  {
    tracked_persons_pub = _nh.advertise<hri_msgs::IdsList>("/humans/persons/tracked", 1, true);
    known_persons_pub = _nh.advertise<hri_msgs::IdsList>("/humans/persons/known", 1, true);
    humans_graph_pub = _nh.advertise<std_msgs::String>("/humans/graph", 1, true);

    personal_space_pub = _nh.advertise<hri_msgs::IdsList>(
      "/humans/persons/in_personal_space", 1, true);
    social_space_pub = _nh.advertise<hri_msgs::IdsList>(
      "/humans/persons/in_social_space", 1, true);
    public_space_pub = _nh.advertise<hri_msgs::IdsList>(
      "/humans/persons/in_public_space", 1, true);


    candidates = _nh.subscribe<hri_msgs::IdsMatch>(
      "/humans/candidate_matches", 10, bind(&PersonManager::onCandidateMatch, this, _1));

    hri_listener.onFace(bind(&PersonManager::onFace, this, _1));
    hri_listener.onFaceLost(bind(&PersonManager::onFeatureLost, this, _1));
    hri_listener.onBody(bind(&PersonManager::onBody, this, _1));
    hri_listener.onBodyLost(bind(&PersonManager::onFeatureLost, this, _1));
    hri_listener.onVoice(bind(&PersonManager::onVoice, this, _1));
    hri_listener.onVoiceLost(bind(&PersonManager::onFeatureLost, this, _1));

    diag_updater.setHardwareID("none");
    diag_updater.add("Data fusion", this, &PersonManager::updateDiagnostics);

    ROS_INFO(
      "hri_person_manager ready. Waiting for candidate associations on /humans/candidate_matches or updates on /humans/*/tracked");
  }

  bool reset(std_srvs::Empty::Request & req, std_srvs::Empty::Response & res)
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

    personal_space_pub.publish(persons_list);
    social_space_pub.publish(persons_list);
    public_space_pub.publish(persons_list);

    return true;
  }

  void onCandidateMatch(hri_msgs::IdsMatchConstPtr match)
  {
    FeatureType type1, type2;
    ID id1, id2;

    id1 = match->id1;

    if (id1.empty()) {
      ROS_ERROR("received an empty id for id1");
      return;
    }

    id2 = match->id2;

    if (id2.empty()) {
      ROS_ERROR("received an empty id for id2");
      return;
    }

    if (id1 == id2) {
      ROS_ERROR("candidate_matches with identical id1 and id2. Skipping.");
      return;
    }

    switch (match->id1_type) {
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

    switch (match->id2_type) {
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
      updates.push_back({RELATION, id1, type1, id2, type2, match->confidence});
    }
  }

  void onFace(FaceWeakConstPtr face)
  {
    ID id;

    if (auto face_ptr = face.lock()) {
      id = face_ptr->id();

      if (id.size() == 0) {
        ROS_ERROR("got invalid new face: empty id! skipping");
        return;
      }

      // create a new node in the graph
      updates.push_back({NEW_FEATURE, id, FeatureType::face, id, FeatureType::face, 0.});
    }
  }

  void onBody(BodyWeakConstPtr body)
  {
    ID id;

    if (auto body_ptr = body.lock()) {
      id = body_ptr->id();

      if (id.size() == 0) {
        ROS_ERROR("got invalid new body: empty id! skipping");
        return;
      }

      // create a new node in the graph
      updates.push_back({NEW_FEATURE, id, FeatureType::body, id, FeatureType::body, 0.});
    }
  }

  void onVoice(VoiceWeakConstPtr voice)
  {
    ID id;

    if (auto voice_ptr = voice.lock()) {
      id = voice_ptr->id();

      if (id.size() == 0) {
        ROS_ERROR("got invalid new voice: empty id! skipping");
        return;
      }

      // create a new node in the graph
      updates.push_back({NEW_FEATURE, id, FeatureType::voice, id, FeatureType::voice, 0.});
    }
  }


  void onFeatureLost(ID id)
  {
    if (id.size() == 0) {
      ROS_ERROR("a feature was removed, but empty id! skipping");
      return;
    }

    updates.push_back({REMOVE, id, FeatureType::invalid, id, FeatureType::invalid, 0.});
  }


  void updateDiagnostics(diagnostic_updater::DiagnosticStatusWrapper & status)
  {
    status.summary(diagnostic_msgs::DiagnosticStatus::OK, "");
    status.add("Package name", "hri_person_manager");
    status.add("Currently tracked persons", previously_tracked.size());
    status.add("Known persons", previously_known.size());
    status.add("Last known person ID", last_known_person);
    status.add("Processing time", to_string(proc_time_ms) + "ms");
  }

  void initialize_person(ID id)
  {
    persons[id] = make_shared<ManagedPerson>(
      _nh,
      id,
      tfBuffer,
      _reference_frame,
      _robot_reference_frame,
      _distance_personal_space,
      _distance_social_space,
      _distance_public_space);
    last_known_person = id;

    publish_known_persons();
  }

  void publish_known_persons()
  {
    // publish an updated list of all known persons
    hri_msgs::IdsList persons_list;
    vector<ID> known;

    for (auto const & kv : persons) {
      persons_list.ids.push_back(kv.first);
      known.push_back(kv.first);
    }

    if (known != previously_known) {
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
    ros::Time proc_start_time = ros::Time::now();

    if (!updates.empty()) {
      ROS_INFO_STREAM("Updating graph:");
    }
    for (auto u : updates) {
      std::tie(update_type, id1, type1, id2, type2, likelihood) = u;

      switch (update_type) {
        case NEW_FEATURE:
          {
            ROS_INFO_STREAM("- New feature: " << id1 << " (" << type1 << ")");
            // create a single 'orphan' node. At the end of the next update cycle,
            // this orphan node will be associated to an anonymous_persons if it
            // is not linked to any other node
            person_matcher.update({{id1, type1, id2, type2, 1.0}});
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
            ROS_INFO_STREAM(
              "- Update relation: " << id1 << " (" << type1 << ") <--> " << id2 << " (" <<
                type2 << "); likelihood=" << likelihood);
            person_matcher.update(
              {{id1, type1, id2, type2, likelihood}},
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
    if (humans_graph_pub.getNumSubscribers() > 0) {
      std_msgs::String graphviz;
      graphviz.data = person_matcher.get_graphviz();
      humans_graph_pub.publish(graphviz);
    }

    ////////////////////////////////////////////
    // go over all the persons in the graph, and process their
    // associations

    for (const auto & kv : person_associations) {
      ID id = kv.first;

      // new person? first, create it (incl its publishers)
      if (!persons.count(id)) {
        initialize_person(id);
      }

      auto person = persons.at(id);

      const auto & association = kv.second;

      ID face_id, body_id, voice_id;

      if (association.find(FeatureType::face) != association.end()) {
        face_id = association.at(face);
      }
      if (association.find(FeatureType::body) != association.end()) {
        body_id = association.at(body);
      }
      if (association.find(FeatureType::voice) != association.end()) {
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
    for (auto const & kv : persons) {
      if (kv.second->anonymous() && !person_associations.count(kv.first)) {
        to_delete.push_back(kv.first);
      }
    }
    for (const auto & p : to_delete) {
      persons.erase(p);
    }


    ////////////////////////////////////////////
    // publish the list of currently actively tracked persons
    hri_msgs::IdsList persons_list;
    vector<ID> actively_tracked;
    hri_msgs::IdsList persons_personal_space_list;
    hri_msgs::IdsList persons_social_space_list;
    hri_msgs::IdsList persons_public_space_list;
    vector<ID> in_personal_space, in_social_space, in_public_space;

    for (auto const & kv : persons) {
      if (kv.second->activelyTracked()) {
        actively_tracked.push_back(kv.first);
        persons_list.ids.push_back(kv.first);

        switch (kv.second->proxemicZone()) {
          case hri::Proxemics::PROXEMICS_UNKNOWN:
            break;
          case Proxemics::PROXEMICS_PERSONAL:
            persons_personal_space_list.ids.push_back(kv.first);
            in_personal_space.push_back(kv.first);
            break;
          case Proxemics::PROXEMICS_SOCIAL:
            persons_social_space_list.ids.push_back(kv.first);
            in_social_space.push_back(kv.first);
            break;
          case Proxemics::PROXEMICS_PUBLIC:
            persons_public_space_list.ids.push_back(kv.first);
            in_public_space.push_back(kv.first);
            break;
        }
      }
    }

    auto stamp = ros::Time::now();

    if (actively_tracked != previously_tracked) {
      persons_list.header.stamp = stamp;
      tracked_persons_pub.publish(persons_list);
      previously_tracked = actively_tracked;
    }

    if (in_personal_space != previously_in_personal_space) {
      persons_personal_space_list.header.stamp = stamp;
      personal_space_pub.publish(persons_personal_space_list);
      previously_in_personal_space = in_personal_space;
    }

    if (in_social_space != previously_in_social_space) {
      persons_social_space_list.header.stamp = stamp;
      social_space_pub.publish(persons_social_space_list);
      previously_in_social_space = in_social_space;
    }

    if (in_public_space != previously_in_public_space) {
      persons_public_space_list.header.stamp = stamp;
      public_space_pub.publish(persons_public_space_list);
      previously_in_public_space = in_public_space;
    }

    proc_time_ms = (ros::Time::now() - proc_start_time).toSec() * 1000;
    publish_known_persons();
    diag_updater.update();
  }

  void set_threshold(float threshold)
  {
    person_matcher.set_threshold(threshold);
  }

private:
  NodeHandle & _nh;

  map<ID, shared_ptr<ManagedPerson>> persons;
  vector<ID> previously_known, previously_tracked;
  vector<ID> previously_in_personal_space, previously_in_social_space, previously_in_public_space;
  ID last_known_person;

  vector<Association> updates;

  HRIListener hri_listener;

  // actively tracked persons (eg, one of face_id, body_id or voice_id is not empty for that person)
  Publisher tracked_persons_pub;
  // known persons: either actively tracked ones, or not tracked anymore (but
  // still known to the robot)
  Publisher known_persons_pub;

  Publisher humans_graph_pub;

  // Proxemics publishers
  Publisher personal_space_pub;
  Publisher social_space_pub;
  Publisher public_space_pub;

  PersonMatcher person_matcher;

  tf2_ros::Buffer tfBuffer;
  tf2_ros::TransformListener tfListener;

  string _reference_frame;
  string _robot_reference_frame;

  float _distance_personal_space;
  float _distance_social_space;
  float _distance_public_space;

  diagnostic_updater::Updater diag_updater;
  double proc_time_ms;

  ros::Subscriber candidates;
  bool _create_features_from_candidate_matches;
};

int main(int argc, char ** argv)
{
  ros::init(argc, argv, "hri_person_manager");
  ros::NodeHandle nh;


  float match_threshold;
  ros::param::param<float>("/humans/match_threshold", match_threshold, 0.5);

  // reference frame in which persons' TF frame are republished. Typically, '/map'.
  // Should not move with the robot, otherwise people not visible to the robot
  // would 'move' along with the robot.
  string reference_frame;
  ros::param::param<string>("/humans/reference_frame", reference_frame, "map");

  // reference frame *on the robot* to compute the distance to the persons,
  // used eg for proxemics calculations. Typically, '/base_link'
  string robot_reference_frame;
  ros::param::param<string>("/humans/robot_reference_frame", robot_reference_frame, "base_link");

  bool create_features_from_candidate_matches;
  ros::param::param<bool>("~features_from_matches", create_features_from_candidate_matches, false);

  if (create_features_from_candidate_matches) {
    ROS_INFO(
      "~features_from_matches: True. New features (faces, bodies, voices) will be created if referred to in /humans/candidate_matches, even if not explicitly tracked. While this is the correct REP-155 semantic, it might cause unexpected 'ghost' features if the feature matchers publish candidate matches even after the feature is not tracked anymore.");
  } else {
    ROS_INFO(
      "~features_from_matches: False (default). New features (faces, bodies, voices) will be only created if explicitly tracked (ie appear in /humans/*/tracked)");
  }

  float distance_personal_space;
  float distance_social_space;
  float distance_public_space;
  ros::param::param<float>("/humans/proxemics/personal_distance", distance_personal_space, 1.2);
  ros::param::param<float>("/humans/proxemics/social_distance", distance_social_space, 3.6);
  ros::param::param<float>("/humans/proxemics/public_distance", distance_public_space, 20);

  ROS_INFO("Proxemics ranges:");
  ROS_INFO_STREAM("- Personal space: from 0m to " << distance_personal_space << "m");
  ROS_INFO_STREAM(
    "- Social space: from " << distance_personal_space << "m to " << distance_social_space << "m");
  ROS_INFO_STREAM(
    "- Public space: from " << distance_social_space << "m to " << distance_public_space << "m");

  PersonManager pm(nh,
    reference_frame,
    robot_reference_frame,
    create_features_from_candidate_matches,
    distance_personal_space,
    distance_social_space,
    distance_public_space);

  pm.set_threshold(match_threshold);

  ros::ServiceServer reset_service =
    nh.advertiseService("/hri_person_manager/reset", &PersonManager::reset, &pm);

  ros::Rate loop_rate(10);

  auto t0 = ros::Time::now();
  while (ros::ok()) {
    t0 = ros::Time::now();
    loop_rate.sleep();
    ros::spinOnce();

    chrono::nanoseconds elapsed_time((ros::Time::now() - t0).toNSec());
    pm.publish_persons(chrono::duration_cast<chrono::milliseconds>(elapsed_time));
  }

  return 0;
}
