// Copyright 2024 PAL Robotics S.L.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
//    * Redistributions of source code must retain the above copyright
//      notice, this list of conditions and the following disclaimer.
//
//    * Redistributions in binary form must reproduce the above copyright
//      notice, this list of conditions and the following disclaimer in the
//      documentation and/or other materials provided with the distribution.
//
//    * Neither the name of the PAL Robotics S.L. nor the names of its
//      contributors may be used to endorse or promote products derived from
//      this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.

#include <iostream>
#include <chrono>
#include <string>
#include <map>
#include <geometry_msgs/TransformStamped.h>
#include <ros/node_handle.h>
#include <ros/publisher.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_broadcaster.h>
#include <hri/base.h>

#include <std_msgs/String.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Float32.h>

namespace hri
{
const std::string NS("/humans/persons/");

// after that period of time, the location confidence of the person will be reduced to 0,
// and its TF transform won't be published anymore
const std::chrono::seconds LIFETIME_UNTRACKED_PERSON(10);

const std::string PERSON("person_");
const std::string ANONYMOUS("anonymous_person_");

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
  ManagedPerson(ros::NodeHandle& nh, hri::ID id, tf2_ros::Buffer& tf_buffer,
                const std::string& reference_frame,
                const std::string& robot_reference_frame, float proxemics_dist_personal,
                float proxemics_dist_social, float proxemics_dist_public);

  ~ManagedPerson();

  void setFaceId(hri::ID id);
  void setBodyId(hri::ID id);
  void setVoiceId(hri::ID id);

  void setAnonymous(bool anonymous);
  bool anonymous() const
  {
    return _anonymous;
  }

  void setAlias(hri::ID id);
  hri::ID alias() const
  {
    return _alias;
  }


  void setLocationConfidence(float confidence);
  float locationConfidence() const
  {
    return _loc_confidence;
  }

  hri::ID id() const
  {
    return _id;
  }

  std::string tfFrame() const
  {
    return _tf_frame;
  }

  bool activelyTracked() const
  {
    return _actively_tracked;
  }

  Proxemics proxemicZone() const
  {
    return _proxemic_zone;
  }

  void update(hri::ID face_id, hri::ID body_id, hri::ID voice_id,
              std::chrono::milliseconds elapsed_time);

private:
  void publishFrame();

  ros::NodeHandle* _nh;

  hri::ID _id;

  ros::Publisher face_id_pub;
  ros::Publisher body_id_pub;
  ros::Publisher voice_id_pub;
  ros::Publisher alias_pub;
  ros::Publisher anonymous_pub;
  ros::Publisher loc_confidence_pub;
  ros::Publisher proxemics_pub;

  bool _actively_tracked;

  std::string _tf_frame;
  std::string _tf_reference_frame;
  std::string _tf_robot_reference_frame;

  tf2_ros::Buffer* _tf_buffer;
  tf2_ros::TransformBroadcaster _tf_br;

  geometry_msgs::TransformStamped _transform;
  bool _had_transform_at_least_once;
  bool _had_computed_distance_at_least_once;

  hri::ID _face_id;
  hri::ID _body_id;
  hri::ID _voice_id;
  float _loc_confidence;
  bool _loc_confidence_dirty;
  bool _anonymous;

  std_msgs::String id_msg;
  std_msgs::Float32 float_msg;
  std_msgs::Bool bool_msg;

  hri::ID _alias;

  std::chrono::milliseconds _time_since_last_seen;

  float _proxemics_dist_personal;
  float _proxemics_dist_social;
  float _proxemics_dist_public;
  Proxemics _proxemic_zone;

  void setProxemics(const std::string& target_frame);
};


}  // namespace hri
