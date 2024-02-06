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


#ifndef HRI_PERSON_MANAGER__MANAGED_PERSON_HPP_
#define HRI_PERSON_MANAGER__MANAGED_PERSON_HPP_

#include <array>
#include <cmath>
#include <chrono>
#include <limits>
#include <map>
#include <memory>
#include <string>

#include "geometry_msgs/msg/transform_stamped.hpp"
#include "hri/types.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/float32.hpp"
#include "std_msgs/msg/string.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_broadcaster.h"

namespace hri_person_manager
{

const char kNs[]{"/humans/persons/"};

// after that period of time, the location confidence of the person will be reduced to 0,
// and its TF transform won't be published anymore
const std::chrono::seconds kLifetimeUntrackedPerson(10);

const char kPerson[]{"person_"};
const char kAnonymous[]{"anonymous_person_"};

class Proxemics
{
public:
  enum Value : uint8_t
  {
    kUnknown,
    kPersonal,
    kSocial,
    kPublic
  };

  Proxemics() = default;
  explicit constexpr Proxemics(Value value)
  : value_(value) {}
  Proxemics & operator=(Value value) {value_ = value; return *this;}
  constexpr operator Value() const {return value_;}  // allow switch and comparison
  constexpr const char * toString() const
  {
    switch (value_) {
      case kPersonal: return "personal";
      case kSocial: return "social";
      case kPublic: return "public";
      case kUnknown:
      default: return "unknown";
    }
  }

private:
  Value value_;
};

template<class T>
bool compare_floats(T l1, T l2)
{
  return std::fabs(l1 - l2) <= std::numeric_limits<T>::epsilon();
}

class ManagedPerson
{
public:
  ManagedPerson(
    hri::NodeLikeSharedPtr node_like, hri::ID id, std::shared_ptr<const tf2::BufferCore> tf_buffer,
    const std::string & reference_frame, const std::string & robot_reference_frame,
    float proxemics_dist_personal, float proxemics_dist_social, float proxemics_dist_public);

  ~ManagedPerson();

  void setFaceId(hri::ID id);
  void setBodyId(hri::ID id);
  void setVoiceId(hri::ID id);

  void setAnonymous(bool anonymous);
  bool anonymous() const {return anonymous_;}

  void setAlias(hri::ID id);
  hri::ID alias() const {return alias_;}

  void setLocationConfidence(float confidence);
  float locationConfidence() const {return loc_confidence_;}

  hri::ID id() const {return kId_;}

  std::string tfFrame() const {return tf_frame_;}

  bool activelyTracked() const {return actively_tracked_;}
  Proxemics proxemicZone() const {return proxemic_zone_;}

  void update(
    hri::ID face_id, hri::ID body_id, hri::ID voice_id, std::chrono::milliseconds elapsed_time);

private:
  void setProxemics(const std::string & target_frame);
  void publishFrame();

  hri::NodeInterfaces node_interfaces_;
  const hri::ID kId_;

  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr face_id_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr body_id_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr voice_id_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr alias_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr anonymous_pub_;
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr loc_confidence_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr proxemics_pub_;

  bool actively_tracked_;

  std::string tf_frame_;
  std::string reference_frame_;
  std::string robot_reference_frame_;

  std::shared_ptr<const tf2::BufferCore> tf_buffer_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

  geometry_msgs::msg::TransformStamped transform_;
  bool had_transform_at_least_once_;
  bool had_computed_distance_at_least_once_;

  // helper variables to track whether or not we should log
  // TF broadcasting/distance to robot computation
  // (to avoid spamming the console in case TF transforms
  // are not available)
  bool last_tf_broadcast_successful_;
  bool need_log_tf_broadcast_;
  bool last_distance_successful_;
  bool need_log_distance_;

  hri::ID face_id_;
  hri::ID body_id_;
  hri::ID voice_id_;
  float loc_confidence_;
  bool loc_confidence_dirty_;
  bool anonymous_;

  hri::ID alias_;

  const float kProxemicsDistPersonal_;
  const float kProxemicsDistSocial_;
  const float kProxemicsDistPublic_;
  Proxemics proxemic_zone_;

  std::chrono::milliseconds time_since_last_seen_;
};

}  // namespace hri_person_manager

#endif  // HRI_PERSON_MANAGER__MANAGED_PERSON_HPP_
