// Copyright (c) 2024 PAL Robotics S.L. All rights reserved.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.


#include "hri_person_manager/node_person_manager.hpp"

#include <chrono>
#include <functional>
#include <string>
#include <tuple>
#include <vector>

#include "diagnostic_msgs/msg/diagnostic_array.hpp"
#include "diagnostic_updater/diagnostic_status_wrapper.hpp"
#include "hri/body.hpp"
#include "hri/face.hpp"
#include "hri/types.hpp"
#include "hri/voice.hpp"
#include "hri_msgs/msg/ids_list.hpp"
#include "hri_msgs/msg/ids_match.hpp"
#include "lifecycle_msgs/msg/state.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_srvs/srv/empty.hpp"
#include "tf2_ros/transform_listener.h"

namespace hri_person_manager
{

using std::placeholders::_1;
using std::placeholders::_2;

std::ostream & operator<<(std::ostream & os, const hri::FeatureType & obj)
{
  os << static_cast<std::underlying_type<hri::FeatureType>::type>(obj);
  return os;
}

NodePersonManager::NodePersonManager(const rclcpp::NodeOptions & options)
: rclcpp_lifecycle::LifecycleNode("hri_person_manager", "", options)
{
  auto descriptor = rcl_interfaces::msg::ParameterDescriptor{};

  descriptor.description = "Minimum likelihood to associate a face/body/voice to a given person";
  this->declare_parameter("match_threshold", 0.5, descriptor);

  descriptor.description = "Reference frame for published persons' TF";
  this->declare_parameter("reference_frame", "map", descriptor);

  descriptor.description =
    "Whether to consider matching features not yet published in '/humans/*/tracked'";
  this->declare_parameter("features_from_matches", true, descriptor);

  RCLCPP_INFO(this->get_logger(), "State: Unconfigured");
}

LifecycleCallbackReturn NodePersonManager::on_configure(const rclcpp_lifecycle::State &)
{
  reference_frame_ = this->get_parameter("reference_frame").as_string();
  features_from_matches_ = this->get_parameter("features_from_matches").as_bool();

  person_matcher_.setThreshold(this->get_parameter("match_threshold").as_double());

  RCLCPP_INFO(this->get_logger(), "State: Inactive");
  return LifecycleCallbackReturn::SUCCESS;
}

LifecycleCallbackReturn NodePersonManager::on_cleanup(const rclcpp_lifecycle::State &)
{
  internal_cleanup();
  RCLCPP_INFO(this->get_logger(), "State: Unconfigured");
  return LifecycleCallbackReturn::SUCCESS;
}

LifecycleCallbackReturn NodePersonManager::on_activate(const rclcpp_lifecycle::State &)
{
  auto latched_qos = rclcpp::SystemDefaultsQoS().transient_local().reliable();

  tracked_persons_pub_ = this->create_publisher<hri_msgs::msg::IdsList>(
    "/humans/persons/tracked", latched_qos);
  known_persons_pub_ = this->create_publisher<hri_msgs::msg::IdsList>(
    "/humans/persons/known", latched_qos);
  humans_graph_pub_ = this->create_publisher<std_msgs::msg::String>(
    "/humans/graph", latched_qos);
  diagnostics_pub_ = this->create_publisher<diagnostic_msgs::msg::DiagnosticArray>(
    "/diagnostics", 1);

  tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

  hri_listener_ = hri::HRIListener::create(shared_from_this());
  hri_listener_->onFace(std::bind(&NodePersonManager::onFace, this, _1));
  hri_listener_->onFaceLost(std::bind(&NodePersonManager::onFeatureLost, this, _1));
  hri_listener_->onBody(std::bind(&NodePersonManager::onBody, this, _1));
  hri_listener_->onBodyLost(std::bind(&NodePersonManager::onFeatureLost, this, _1));
  hri_listener_->onVoice(std::bind(&NodePersonManager::onVoice, this, _1));
  hri_listener_->onVoiceLost(std::bind(&NodePersonManager::onFeatureLost, this, _1));

  candidates_sub_ = this->create_subscription<hri_msgs::msg::IdsMatch>(
    "/humans/candidate_matches", 10, bind(&NodePersonManager::onCandidateMatch, this, _1));

  proc_start_time_ = this->get_clock()->now();
  persons_timer_ = rclcpp::create_timer(
    this, this->get_clock(), std::chrono::milliseconds(100),
    std::bind(&NodePersonManager::publishPersons, this));
  diagnostics_timer_ = rclcpp::create_timer(
    this, this->get_clock(), std::chrono::seconds(1),
    std::bind(&NodePersonManager::updateDiagnostics, this));

  reset_srv_ = this->create_service<std_srvs::srv::Empty>(
    "~/reset", std::bind(&NodePersonManager::reset, this, _1, _2));

  RCLCPP_INFO(this->get_logger(), "State: Active");
  RCLCPP_INFO_STREAM(
    this->get_logger(),
    "Waiting for candidate associations on /humans/candidate_matches or updates on "
      << "/humans/*/tracked");
  return LifecycleCallbackReturn::SUCCESS;
}

LifecycleCallbackReturn NodePersonManager::on_deactivate(const rclcpp_lifecycle::State &)
{
  internal_deactivate();
  RCLCPP_INFO(this->get_logger(), "State: Inactive");
  return LifecycleCallbackReturn::SUCCESS;
}

LifecycleCallbackReturn NodePersonManager::on_shutdown(const rclcpp_lifecycle::State & state)
{
  if (state.id() == lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE) {
    internal_deactivate();
  }
  internal_cleanup();
  RCLCPP_INFO(this->get_logger(), "State: Finalized");
  return LifecycleCallbackReturn::SUCCESS;
}

void NodePersonManager::internal_deactivate()
{
  reset_srv_.reset();
  diagnostics_timer_.reset();
  persons_timer_.reset();
  candidates_sub_.reset();
  tf_listener_.reset();
  hri_listener_.reset();
  tracked_persons_pub_.reset();
  known_persons_pub_.reset();
  humans_graph_pub_.reset();
  diagnostics_pub_.reset();
}

void NodePersonManager::internal_cleanup()
{}

void NodePersonManager::reset(
  const std_srvs::srv::Empty::Request::SharedPtr,
  std_srvs::srv::Empty::Response::SharedPtr)
{
  RCLCPP_WARN(
    this->get_logger(), "Clearing all associations between persons, faces, bodies, voices");
  hri_listener_->clearData();
  person_matcher_.reset();

  persons_.clear();
  previously_known_.clear();
  previously_tracked_.clear();

  // publish an empty list of tracked/known persons
  hri_msgs::msg::IdsList persons_list;
  tracked_persons_pub_->publish(persons_list);
  known_persons_pub_->publish(persons_list);
}

void NodePersonManager::onCandidateMatch(const hri_msgs::msg::IdsMatch & match)
{
  hri::FeatureType type1, type2;
  hri::ID id1, id2;

  id1 = match.id1;

  if (id1.empty()) {
    RCLCPP_ERROR(this->get_logger(), "Received an empty id for id1");
    return;
  }

  id2 = match.id2;

  if (id2.empty()) {
    RCLCPP_ERROR(this->get_logger(), "Received an empty id for id2");
    return;
  }

  if (id1 == id2) {
    RCLCPP_ERROR(this->get_logger(), "Candidate_matches with identical id1 and id2. Skipping.");
    return;
  }

  switch (match.id1_type) {
    case hri_msgs::msg::IdsMatch::PERSON:
      type1 = hri::FeatureType::kPerson;
      break;

    case hri_msgs::msg::IdsMatch::FACE:
      type1 = hri::FeatureType::kFace;
      break;

    case hri_msgs::msg::IdsMatch::BODY:
      type1 = hri::FeatureType::kBody;
      break;

    case hri_msgs::msg::IdsMatch::VOICE:
      type1 = hri::FeatureType::kVoice;
      break;

    default:
      RCLCPP_ERROR_STREAM(
        this->get_logger(), "Received an invalid type for id1: " << match.id1_type);
      return;
  }

  switch (match.id2_type) {
    case hri_msgs::msg::IdsMatch::PERSON:
      type2 = hri::FeatureType::kPerson;
      break;

    case hri_msgs::msg::IdsMatch::FACE:
      type2 = hri::FeatureType::kFace;
      break;

    case hri_msgs::msg::IdsMatch::BODY:
      type2 = hri::FeatureType::kBody;
      break;

    case hri_msgs::msg::IdsMatch::VOICE:
      type2 = hri::FeatureType::kVoice;
      break;

    default:
      RCLCPP_ERROR_STREAM(
        this->get_logger(), "Received an invalid type for id2: " << match.id2_type);
      return;
  }

  updates_.push_back({UpdateType::kRelation, id1, type1, id2, type2, match.confidence});
}

void NodePersonManager::onFace(hri::ConstFacePtr face)
{
  auto id = face->id();
  if (id.empty()) {
    RCLCPP_ERROR(this->get_logger(), "Got invalid new face: empty id! Skipping");
  } else {
    // create a new node in the graph
    updates_.push_back(
      {UpdateType::kNewFeature, id, hri::FeatureType::kFace, id, hri::FeatureType::kFace, 0.});
  }
}

void NodePersonManager::onBody(hri::ConstBodyPtr body)
{
  auto id = body->id();
  if (id.empty()) {
    RCLCPP_ERROR(this->get_logger(), "Got invalid new body: empty id! Skipping");
  } else {
    // create a new node in the graph
    updates_.push_back(
      {UpdateType::kNewFeature, id, hri::FeatureType::kBody, id, hri::FeatureType::kBody, 0.});
  }
}

void NodePersonManager::onVoice(hri::ConstVoicePtr voice)
{
  auto id = voice->id();
  if (id.empty()) {
    RCLCPP_ERROR(this->get_logger(), "Got invalid new voice: empty id! Skipping");
  } else {
    // create a new node in the graph
    updates_.push_back(
      {UpdateType::kNewFeature, id, hri::FeatureType::kVoice, id, hri::FeatureType::kVoice, 0.});
  }
}

void NodePersonManager::onFeatureLost(hri::ID id)
{
  if (id.empty()) {
    RCLCPP_ERROR(this->get_logger(), "A feature was removed, but empty id! Skipping");
  } else {
    updates_.push_back(
      {UpdateType::kRemove, id, hri::FeatureType::kInvalid, id, hri::FeatureType::kInvalid, 0.});
  }
}

void NodePersonManager::updateDiagnostics()
{
  diagnostic_updater::DiagnosticStatusWrapper status;
  status.name = "Social perception: Data fusion";
  status.hardware_id = "none";
  status.summary(diagnostic_msgs::msg::DiagnosticStatus::OK, "OK");
  status.add("Package name", "hri_person_manager");
  status.add("Currently tracked persons", previously_tracked_.size());
  status.add("Known persons", previously_known_.size());
  status.add("Last known person ID", last_known_person_);
  status.add("Processing time", std::to_string(proc_time_ms_.count()) + "ms");

  diagnostic_msgs::msg::DiagnosticArray msg;
  msg.header.stamp = this->get_clock()->now();
  msg.status.push_back(status);
  diagnostics_pub_->publish(msg);
}

void NodePersonManager::initializePerson(hri::ID id)
{
  persons_[id] = std::make_shared<ManagedPerson>(
    shared_from_this(), id, tf_buffer_, reference_frame_);
  last_known_person_ = id;

  publishKnownPersons();
}

void NodePersonManager::publishKnownPersons()
{
  // publish an updated list of all known persons
  hri_msgs::msg::IdsList persons_list;
  std::vector<hri::ID> known;

  for (auto const & kv : persons_) {
    persons_list.ids.push_back(kv.first);
    known.push_back(kv.first);
  }

  if (known != previously_known_) {
    persons_list.header.stamp = this->get_clock()->now();
    known_persons_pub_->publish(persons_list);
    previously_known_ = known;
  }
}

void NodePersonManager::publishPersons()
{
  ///////////////////////////////////
  // first: housekeeping -> update the graph with all the last changes
  UpdateType update_type;
  hri::ID id1, id2;
  hri::FeatureType type1, type2;
  float likelihood;
  std::chrono::milliseconds elapsed_time = (
    this->get_clock()->now() - proc_start_time_).to_chrono<std::chrono::milliseconds>();
  proc_start_time_ = this->get_clock()->now();

  if (!updates_.empty()) {
    RCLCPP_INFO(this->get_logger(), "Updating graph:");
  }
  for (auto u : updates_) {
    std::tie(update_type, id1, type1, id2, type2, likelihood) = u;

    switch (update_type) {
      case UpdateType::kNewFeature:
        RCLCPP_INFO_STREAM(this->get_logger(), "- New feature: " << id1 << " (" << type1 << ")");
        // create a single 'orphan' node. At the end of the next update cycle,
        // this orphan node will be associated to an anonymous_persons if it
        // is not linked to any other node
        person_matcher_.update({{id1, type1, id2, type2, 1.0}});
        break;

      case UpdateType::kRemove:
        RCLCPP_INFO_STREAM(this->get_logger(), "- Remove ID: " << id1);
        person_matcher_.erase(id1);
        break;

      case UpdateType::kRelation:
        RCLCPP_INFO_STREAM(
          this->get_logger(),
          "- Update relation: " << id1 << " (" << type1 << ") <--> " << id2 << " (" << type2
                                << "); likelihood=" << likelihood);
        person_matcher_.update({{id1, type1, id2, type2, likelihood}}, features_from_matches_);
        break;
    }
  }
  updates_.clear();
  //////////////////////////

  //////////////////////////
  //   MAIN ALGORITHM     //
  //////////////////////////
  auto person_associations = person_matcher_.getAllAssociations();
  //////////////////////////

  ////////////////////////////////////////////
  // if someone needs it, publish the graphviz model of the relationship graph
  if (humans_graph_pub_->get_subscription_count() > 0) {
    std_msgs::msg::String graphviz;
    graphviz.data = person_matcher_.getGraphviz();
    humans_graph_pub_->publish(graphviz);
  }

  ////////////////////////////////////////////
  // go over all the persons in the graph, and process their
  // associations
  for (const auto & kv : person_associations) {
    hri::ID id = kv.first;

    // new person? first, create it (incl its publishers)
    if (!persons_.count(id)) {
      initializePerson(id);
    }

    auto person = persons_.at(id);
    const auto & association = kv.second;

    hri::ID face_id, body_id, voice_id;
    if (association.find(hri::FeatureType::kFace) != association.end()) {
      face_id = association.at(hri::FeatureType::kFace);
    }
    if (association.find(hri::FeatureType::kBody) != association.end()) {
      body_id = association.at(hri::FeatureType::kBody);
    }
    if (association.find(hri::FeatureType::kVoice) != association.end()) {
      voice_id = association.at(hri::FeatureType::kVoice);
    }

    ////////////////////////////////////////////
    // publish the face, body, voice id corresponding to the person
    person->update(face_id, body_id, voice_id, elapsed_time);
  }

  ////////////////////////////////////////////
  // if an anonymous person is *not* present anymore in the associations, it
  // has disappeared, and must be removed from the system.
  std::vector<hri::ID> to_delete;
  for (auto const & kv : persons_) {
    if (kv.second->anonymous() && !person_associations.count(kv.first)) {
      to_delete.push_back(kv.first);
    }
  }
  for (const auto & p : to_delete) {
    persons_.erase(p);
  }

  ////////////////////////////////////////////
  // publish the list of currently actively tracked persons
  hri_msgs::msg::IdsList persons_list;
  std::vector<hri::ID> actively_tracked;

  for (auto const & kv : persons_) {
    if (kv.second->activelyTracked()) {
      actively_tracked.push_back(kv.first);
      persons_list.ids.push_back(kv.first);
    }
  }

  if (actively_tracked != previously_tracked_) {
    persons_list.header.stamp = this->get_clock()->now();
    tracked_persons_pub_->publish(persons_list);
    previously_tracked_ = actively_tracked;
  }

  proc_time_ms_ = (
    this->get_clock()->now() - proc_start_time_).to_chrono<std::chrono::milliseconds>();
  publishKnownPersons();
}

}  // namespace hri_person_manager
