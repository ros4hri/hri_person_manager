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


#include "hri_person_manager/managed_person.hpp"

#include <chrono>
#include <cmath>
#include <string>
#include <variant>

#include "hri/types.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/float32.hpp"
#include "std_msgs/msg/string.hpp"
#include "tf2/exceptions.h"
#include "tf2/time.h"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_broadcaster.h"

namespace hri_person_manager
{

ManagedPerson::ManagedPerson(
  hri::NodeLikeSharedPtr node_like, hri::ID id, std::shared_ptr<const tf2::BufferCore> tf_buffer,
  const std::string & reference_frame, const std::string & robot_reference_frame,
  float proxemics_dist_personal, float proxemics_dist_social, float proxemics_dist_public)
: node_interfaces_(node_like),
  kId_(id),
  actively_tracked_(false),
  reference_frame_(reference_frame),
  robot_reference_frame_(robot_reference_frame),
  tf_buffer_(tf_buffer),
  had_transform_at_least_once_(false),
  had_computed_distance_at_least_once_(false),
  last_tf_broadcast_successful_(false),
  need_log_tf_broadcast_(true),
  last_distance_successful_(false),
  need_log_distance_(true),
  loc_confidence_(0.),
  loc_confidence_dirty_(false),
  anonymous_(false),
  kProxemicsDistPersonal_(proxemics_dist_personal),
  kProxemicsDistSocial_(proxemics_dist_social),
  kProxemicsDistPublic_(proxemics_dist_public),
  proxemic_zone_(Proxemics::kUnknown),
  time_since_last_seen_(0)
{
  auto default_qos = rclcpp::SystemDefaultsQoS();
  auto latched_qos = rclcpp::SystemDefaultsQoS().transient_local().reliable();

  face_id_pub_ = rclcpp::create_publisher<std_msgs::msg::String>(
    node_interfaces_.get_node_parameters_interface(), node_interfaces_.get_node_topics_interface(),
    "/humans/persons/" + kId_ + "/face_id", latched_qos);
  body_id_pub_ = rclcpp::create_publisher<std_msgs::msg::String>(
    node_interfaces_.get_node_parameters_interface(), node_interfaces_.get_node_topics_interface(),
    "/humans/persons/" + kId_ + "/body_id", latched_qos);
  voice_id_pub_ = rclcpp::create_publisher<std_msgs::msg::String>(
    node_interfaces_.get_node_parameters_interface(), node_interfaces_.get_node_topics_interface(),
    "/humans/persons/" + kId_ + "/voice_id", latched_qos);
  alias_pub_ = rclcpp::create_publisher<std_msgs::msg::String>(
    node_interfaces_.get_node_parameters_interface(), node_interfaces_.get_node_topics_interface(),
    "/humans/persons/" + kId_ + "/alias", latched_qos);
  anonymous_pub_ = rclcpp::create_publisher<std_msgs::msg::Bool>(
    node_interfaces_.get_node_parameters_interface(), node_interfaces_.get_node_topics_interface(),
    "/humans/persons/" + kId_ + "/anonymous", latched_qos);
  loc_confidence_pub_ = rclcpp::create_publisher<std_msgs::msg::Float32>(
    node_interfaces_.get_node_parameters_interface(), node_interfaces_.get_node_topics_interface(),
    "/humans/persons/" + kId_ + "/location_confidence", default_qos);
  proxemics_pub_ = rclcpp::create_publisher<std_msgs::msg::String>(
    node_interfaces_.get_node_parameters_interface(), node_interfaces_.get_node_topics_interface(),
    "/humans/persons/" + kId_ + "/proxemic_space", latched_qos);

  setAnonymous((kId_.rfind(kAnonymous, 0) == 0) ? true : false);

  tf_frame_ = anonymous_ ? kId_ : kPerson + kId_;

  std::visit(
    [&](auto && node) {
      tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(node);
    }, node_like);
}

ManagedPerson::~ManagedPerson()
{
  RCLCPP_DEBUG_STREAM(
    node_interfaces_.get_node_logging_interface()->get_logger(), "Forgetting person " << kId_);
}

void ManagedPerson::setFaceId(hri::ID id)
{
  if (id != face_id_) {
    RCLCPP_INFO_STREAM(
      node_interfaces_.get_node_logging_interface()->get_logger(),
      "[person <" << kId_ << ">] face_id updated to <" << id << ">");
  }

  face_id_ = id;
  std_msgs::msg::String id_msg;
  id_msg.data = id;
  face_id_pub_->publish(id_msg);
}

void ManagedPerson::setBodyId(hri::ID id)
{
  if (id != body_id_) {
    RCLCPP_INFO_STREAM(
      node_interfaces_.get_node_logging_interface()->get_logger(),
      "[person <" << kId_ << ">] body_id updated to <" << id << ">");
  }

  body_id_ = id;
  std_msgs::msg::String id_msg;
  id_msg.data = id;
  body_id_pub_->publish(id_msg);
}

void ManagedPerson::setVoiceId(hri::ID id)
{
  if (id != voice_id_) {
    RCLCPP_INFO_STREAM(
      node_interfaces_.get_node_logging_interface()->get_logger(),
      "[person <" << kId_ << ">] voice_id updated to <" << id << ">");
  }
  voice_id_ = id;
  std_msgs::msg::String id_msg;
  id_msg.data = id;
  voice_id_pub_->publish(id_msg);
}

void ManagedPerson::setAnonymous(bool anonymous)
{
  if (anonymous && anonymous_ != anonymous) {
    RCLCPP_WARN_STREAM(
      node_interfaces_.get_node_logging_interface()->get_logger(),
      "new anonymous person " << kId_);
  }
  anonymous_ = anonymous;
  std_msgs::msg::Bool bool_msg;
  bool_msg.data = anonymous;
  anonymous_pub_->publish(bool_msg);
}

void ManagedPerson::setAlias(hri::ID id)
{
  if (id != alias_) {
    RCLCPP_INFO_STREAM(
      node_interfaces_.get_node_logging_interface()->get_logger(),
      "[person <" << kId_ << ">] set to be alias of <" << alias_ << ">");
  }
  alias_ = id;
  std_msgs::msg::String id_msg;
  id_msg.data = id;
  alias_pub_->publish(id_msg);
}

void ManagedPerson::setLocationConfidence(float confidence)
{
  loc_confidence_ = confidence;
  std_msgs::msg::Float32 float_msg;
  float_msg.data = confidence;
  loc_confidence_pub_->publish(float_msg);
}

void ManagedPerson::setProxemics(const std::string & target_frame)
{
  float distance = 0.f;
  try {
    auto transform = tf_buffer_->lookupTransform(
      robot_reference_frame_, target_frame, tf2::TimePointZero);

    distance = sqrt(
      transform.transform.translation.x * transform.transform.translation.x +
      transform.transform.translation.y * transform.transform.translation.y +
      transform.transform.translation.z * transform.transform.translation.z);

    if (distance <= kProxemicsDistPersonal_) {
      proxemic_zone_ = Proxemics::kPersonal;
    } else {
      if (distance <= kProxemicsDistSocial_) {
        proxemic_zone_ = Proxemics::kSocial;
      } else {
        if (kProxemicsDistPublic_ <= 0 || distance <= kProxemicsDistPublic_) {
          proxemic_zone_ = Proxemics::kPublic;
        } else {
          proxemic_zone_ = Proxemics::kUnknown;
        }
      }
    }
  } catch (tf2::TransformException & ex) {
    RCLCPP_WARN_STREAM(
      node_interfaces_.get_node_logging_interface()->get_logger(),
      "Could not compute proxemics for person <" << kId_ << ">: " << ex.what());
    proxemic_zone_ = Proxemics::kUnknown;
  }

  std_msgs::msg::String proxemic_msg;
  proxemic_msg.data = proxemic_zone_.toString();
  proxemics_pub_->publish(proxemic_msg);
}

void ManagedPerson::update(
  hri::ID face_id, hri::ID body_id, hri::ID voice_id, std::chrono::milliseconds elapsed_time)
{
  // a person is considered 'actively tracked' if at least one of its face/body/voice is tracked
  actively_tracked_ = !face_id.empty() || !body_id.empty() || !voice_id.empty();

  setFaceId(face_id);
  setBodyId(body_id);
  setVoiceId(voice_id);

  if (actively_tracked_) {
    time_since_last_seen_ = std::chrono::milliseconds(0);
    if (!compare_floats(loc_confidence_, 1.f)) {
      loc_confidence_ = 1.;
      loc_confidence_dirty_ = true;
    }
  } else {  // *not* actively tracked
    if (time_since_last_seen_ > kLifetimeUntrackedPerson) {
      if (!compare_floats(loc_confidence_, 0.f)) {
        loc_confidence_ = 0.;
        loc_confidence_dirty_ = true;
        RCLCPP_WARN_STREAM(
          node_interfaces_.get_node_logging_interface()->get_logger(),
          "[person <" << kId_ << ">] not seen for more than " << kLifetimeUntrackedPerson.count()
                      << ". Not publishing tf frame anymore.");
      }
    } else {  // not tracked, but lifetime *not yet expired*
      time_since_last_seen_ += elapsed_time;
      loc_confidence_ = 1. - time_since_last_seen_ / kLifetimeUntrackedPerson;
      loc_confidence_dirty_ = true;
    }
  }

  if (loc_confidence_dirty_) {
    setLocationConfidence(loc_confidence_);
    loc_confidence_dirty_ = false;
  }

  publishFrame();
}

void ManagedPerson::publishFrame()
{
  /////////////////////////////////////////////
  // publish TF frame of the person
  std::string target_frame;

  if (!face_id_.empty()) {
    target_frame = std::string("face_") + face_id_;
  } else if (!body_id_.empty()) {
    target_frame = std::string("head_") + body_id_;
  } else if (!voice_id_.empty()) {
    target_frame = std::string("voice_") + voice_id_;
  }

  if (!target_frame.empty()) {
    if (tf_buffer_->canTransform(reference_frame_, target_frame, tf2::TimePointZero)) {
      // log management
      if (!last_tf_broadcast_successful_) {
        need_log_tf_broadcast_ = true;
        last_tf_broadcast_successful_ = true;
      }
      if (need_log_tf_broadcast_) {
        RCLCPP_INFO_STREAM(
          node_interfaces_.get_node_logging_interface()->get_logger(),
          "[person <" << kId_ << ">] broadcast transform " << reference_frame_ << " <-> "
                      << target_frame);
        need_log_tf_broadcast_ = false;
      }

      try {
        transform_ = tf_buffer_->lookupTransform(
          reference_frame_, target_frame, tf2::TimePointZero);
        transform_.header.stamp = node_interfaces_.clock->get_clock()->now();
        transform_.child_frame_id = tf_frame_;

        tf_broadcaster_->sendTransform(transform_);
        had_transform_at_least_once_ = true;
      } catch (const tf2::TransformException & ex) {
        RCLCPP_WARN_STREAM(
          node_interfaces_.get_node_logging_interface()->get_logger(),
          "failed to transform " << target_frame << " to " << reference_frame_ << ". "
                                 << ex.what());
      }
    } else {
      // log management
      if (last_tf_broadcast_successful_) {
        need_log_tf_broadcast_ = true;
        last_tf_broadcast_successful_ = false;
      }
      if (need_log_tf_broadcast_) {
        RCLCPP_WARN_STREAM(
          node_interfaces_.get_node_logging_interface()->get_logger(),
          "[person <" << kId_ << ">] can not publish person "
                      << "transform (either reference frame <"
                      << reference_frame_ << "> or target frame <"
                      << target_frame << "> are not available)");
        need_log_tf_broadcast_ = false;
      }
    }
  } else {
    if (!had_transform_at_least_once_) {
      RCLCPP_INFO_STREAM(
        node_interfaces_.get_node_logging_interface()->get_logger(),
        "[person <" << kId_ << ">] no face, body or voice TF frame available. "
                    << "Can not yet broadcast frame <" << tf_frame_ << ">.");
    } else {
      // publish the last known transform, until loc_confidence == 0
      if (loc_confidence_ > 0.) {
        transform_.header.stamp = node_interfaces_.clock->get_clock()->now();
        tf_broadcaster_->sendTransform(transform_);
      }
    }
  }

  // computation of distance to robot
  if (!target_frame.empty()) {
    if (tf_buffer_->canTransform(robot_reference_frame_, target_frame, tf2::TimePointZero)) {
      // log management
      if (!last_distance_successful_) {
        need_log_distance_ = true;
        last_distance_successful_ = true;
      }
      if (need_log_distance_) {
        RCLCPP_INFO_STREAM(
          node_interfaces_.get_node_logging_interface()->get_logger(),
          "[person <" << kId_ << ">] distance to robot computed as "
                      << robot_reference_frame_ << " <-> " << target_frame);
        need_log_distance_ = false;
      }

      setProxemics(target_frame);
      had_computed_distance_at_least_once_ = true;
    } else {
      proxemic_zone_ = Proxemics::kUnknown;

      // log management
      if (last_distance_successful_) {
        need_log_distance_ = true;
        last_distance_successful_ = false;
      }
      if (need_log_distance_) {
        RCLCPP_WARN_STREAM(
          node_interfaces_.get_node_logging_interface()->get_logger(),
          "[person <" << kId_ << ">] can not compute distance (either reference frame "
                      << robot_reference_frame_ << "> or target frame <"
                      << target_frame << "> are not available)");
        need_log_distance_ = false;
      }
    }
  } else {
    if (!had_computed_distance_at_least_once_) {
      RCLCPP_INFO_STREAM(
        node_interfaces_.get_node_logging_interface()->get_logger(),
        "[person <" << kId_ << ">] no face, body or voice TF frame avail. "
                    << "Can not yet compute distance to robot.");
    } else {
      // publish the last known transform, until loc_confidence == 0
      if (
        loc_confidence_ > 0. &&
        tf_buffer_->canTransform(robot_reference_frame_, tf_frame_, tf2::TimePointZero))
      {
        setProxemics(tf_frame_);
      }
    }
  }
}

}  // namespace hri_person_manager
