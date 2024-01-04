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

#include <cmath>
#include <chrono>
#include <limits>
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

template<class T>
bool compare_floats(T l1, T l2)
{
  return std::fabs(l1 - l2) <= std::numeric_limits<T>::epsilon();
}

// 0 < personal space =< DEFAULT_PERSONAL_DISTANCE
const float DEFAULT_PERSONAL_DISTANCE = 1.2;  // m
// DEFAULT_PERSONAL_DISTANCE < social space =< DEFAULT_SOCIAL_DISTANCE
const float DEFAULT_SOCIAL_DISTANCE = 3.6;  // m
// DEFAULT_SOCIAL_DISTANCE < public space =< DEFAULT_PUBLIC_DISTANCE
const float DEFAULT_PUBLIC_DISTANCE = 20.;  // m

enum Proxemics
{
  PROXEMICS_UNKNOWN,
  PROXEMICS_PERSONAL,
  PROXEMICS_SOCIAL,
  PROXEMICS_PUBLIC
};

const std::map<Proxemics, std::string> PROXEMICS{
  { Proxemics::PROXEMICS_UNKNOWN, "unknown" },
  { Proxemics::PROXEMICS_PERSONAL, "personal" },
  { Proxemics::PROXEMICS_SOCIAL, "social" },
  { Proxemics::PROXEMICS_PUBLIC, "public" },
};


class ManagedPerson
{
public:
  ManagedPerson(
    hri::NodeLikeSharedPtr node_like, hri::ID id, std::shared_ptr<const tf2::BufferCore> tf_buffer,
    const std::string & reference_frame);

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

  void update(
    hri::ID face_id, hri::ID body_id, hri::ID voice_id, std::chrono::milliseconds elapsed_time);

private:
  void publishFrame();

  hri::NodeInterfaces node_interfaces_;
  const hri::ID kId_;

  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr face_id_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr body_id_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr voice_id_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr alias_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr anonymous_pub_;
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr loc_confidence_pub_;

  bool actively_tracked_;

  std::string tf_frame_;
  std::string tf_reference_frame_;

  std::shared_ptr<const tf2::BufferCore> tf_buffer_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

  geometry_msgs::msg::TransformStamped transform_;
  bool had_transform_at_least_once_;

  hri::ID face_id_;
  hri::ID body_id_;
  hri::ID voice_id_;
  float loc_confidence_;
  bool loc_confidence_dirty_;
  bool anonymous_;

  hri::ID alias_;

  std::chrono::milliseconds time_since_last_seen_;
};

}  // namespace hri_person_manager

#endif  // HRI_PERSON_MANAGER__MANAGED_PERSON_HPP_
