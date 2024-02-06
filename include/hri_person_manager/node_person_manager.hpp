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


#ifndef HRI_PERSON_MANAGER__NODE_PERSON_MANAGER_HPP_
#define HRI_PERSON_MANAGER__NODE_PERSON_MANAGER_HPP_

#include <chrono>
#include <map>
#include <memory>
#include <sstream>
#include <string>
#include <tuple>
#include <vector>

#include "diagnostic_msgs/msg/diagnostic_array.hpp"
#include "hri/body.hpp"
#include "hri/face.hpp"
#include "hri/hri.hpp"
#include "hri/types.hpp"
#include "hri/voice.hpp"
#include "hri_msgs/msg/ids_list.hpp"
#include "hri_msgs/msg/ids_match.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_srvs/srv/empty.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"

#include "hri_person_manager/managed_person.hpp"
#include "hri_person_manager/person_matcher.hpp"

namespace hri_person_manager
{

const float kDefaultPersonalDistance = 1.2;
const float kDefaultSocialDistance = 3.6;
const float kDefaultPublicDistance = 20.;

enum class UpdateType
{
  kNewFeature,
  kRelation,
  kRemove
};

using Association =
  std::tuple<UpdateType, hri::ID, hri::FeatureType, hri::ID, hri::FeatureType, float>;
using EmptyReq = std_srvs::srv::Empty::Request::SharedPtr;
using EmptyResp = std_srvs::srv::Empty::Response::SharedPtr;
using LifecycleCallbackReturn =
  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

class NodePersonManager : public rclcpp_lifecycle::LifecycleNode
{
public:
  explicit NodePersonManager(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

  LifecycleCallbackReturn on_configure(const rclcpp_lifecycle::State &);
  LifecycleCallbackReturn on_cleanup(const rclcpp_lifecycle::State &);
  LifecycleCallbackReturn on_activate(const rclcpp_lifecycle::State &);
  LifecycleCallbackReturn on_deactivate(const rclcpp_lifecycle::State &);
  LifecycleCallbackReturn on_shutdown(const rclcpp_lifecycle::State &);

private:
  void internal_cleanup();
  void internal_deactivate();
  void onCandidateMatch(const hri_msgs::msg::IdsMatch & match);
  void onFace(hri::ConstFacePtr face);
  void onBody(hri::ConstBodyPtr body);
  void onVoice(hri::ConstVoicePtr voice);
  void onFeatureLost(hri::ID id);
  void updateDiagnostics();
  void reset();
  void reset(const EmptyReq, EmptyResp) {reset();}
  void initializePerson(hri::ID id);
  void publishKnownPersons();
  void publishPersons();

  // reference frame in which persons' TF frame are republished. Typically, '/map'.
  // Should not move with the robot, otherwise people not visible to the robot
  // would 'move' along with the robot.
  std::string reference_frame_;
  // reference frame *on the robot* to compute the distance to the persons,
  // used eg for proxemics calculations. Typically, '/base_link'
  std::string robot_reference_frame_;
  bool features_from_matches_;
  float personal_distance_;
  float social_distance_;
  float public_distance_;

  PersonMatcher person_matcher_;
  std::vector<hri::ID> previously_known_;
  std::vector<hri::ID> previously_tracked_;
  std::vector<hri::ID> previously_in_personal_space_;
  std::vector<hri::ID> previously_in_social_space_;
  std::vector<hri::ID> previously_in_public_space_;
  hri::ID last_known_person_;
  std::vector<Association> updates_;
  rclcpp::Time proc_start_time_;
  std::chrono::milliseconds proc_time_ms_;

  std::shared_ptr<hri::HRIListener> hri_listener_;
  std::map<hri::ID, std::shared_ptr<ManagedPerson>> persons_;

  // actively tracked persons (eg, one of face_id, body_id or voice_id is not empty for that person)
  rclcpp::Publisher<hri_msgs::msg::IdsList>::SharedPtr tracked_persons_pub_;
  // known persons: either actively tracked ones, or not tracked anymore (but
  // still known to the robot)
  rclcpp::Publisher<hri_msgs::msg::IdsList>::SharedPtr known_persons_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr humans_graph_pub_;
  rclcpp::Publisher<hri_msgs::msg::IdsList>::SharedPtr personal_space_pub_;
  rclcpp::Publisher<hri_msgs::msg::IdsList>::SharedPtr social_space_pub_;
  rclcpp::Publisher<hri_msgs::msg::IdsList>::SharedPtr public_space_pub_;
  rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr diagnostics_pub_;

  rclcpp::Subscription<hri_msgs::msg::IdsMatch>::SharedPtr candidates_sub_;

  tf2_ros::Buffer::SharedPtr tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

  std::shared_ptr<rclcpp::TimerBase> diagnostics_timer_;
  std::shared_ptr<rclcpp::TimerBase> persons_timer_;

  rclcpp::Service<std_srvs::srv::Empty>::SharedPtr reset_srv_;
};

}  // namespace hri_person_manager

#endif  // HRI_PERSON_MANAGER__NODE_PERSON_MANAGER_HPP_
