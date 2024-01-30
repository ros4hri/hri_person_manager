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

#include "managed_person.h"
#include <chrono>
#include <cmath>

using namespace std;
using namespace ros;
using namespace hri;

ManagedPerson::ManagedPerson(NodeHandle& nh, ID id, tf2_ros::Buffer& tf_buffer,
                             const string& reference_frame, const string& robot_reference_frame,
                             float proxemics_dist_personal, float proxemics_dist_social,
                             float proxemics_dist_public)
  : _nh(&nh)
  , _id(id)
  , _actively_tracked(false)
  , _tf_reference_frame(reference_frame)
  , _tf_robot_reference_frame(robot_reference_frame)
  , _tf_buffer(&tf_buffer)
  , _had_transform_at_least_once(false)
  , _had_computed_distance_at_least_once(false)
  , _loc_confidence(0.)
  , _loc_confidence_dirty(false)
  , _anonymous(false)
  , _last_tf_broadcast_successful(false)
  , _need_log_tf_broadcast(true)
  , _last_distance_successful(false)
  , _need_log_distance(true)
  , _time_since_last_seen(0)
  , _proxemics_dist_personal(proxemics_dist_personal)
  , _proxemics_dist_social(proxemics_dist_social)
  , _proxemics_dist_public(proxemics_dist_public)
  , _proxemic_zone(Proxemics::PROXEMICS_UNKNOWN)
{
  face_id_pub = _nh->advertise<std_msgs::String>(NS + id + "/face_id", 1, true);
  body_id_pub = _nh->advertise<std_msgs::String>(NS + id + "/body_id", 1, true);
  voice_id_pub = _nh->advertise<std_msgs::String>(NS + id + "/voice_id", 1, true);
  alias_pub = _nh->advertise<std_msgs::String>(NS + id + "/alias", 1, true);
  anonymous_pub = _nh->advertise<std_msgs::Bool>(NS + id + "/anonymous", 1, true);
  loc_confidence_pub = _nh->advertise<std_msgs::Float32>(NS + id + "/location_confidence", 1);
  proxemics_pub = _nh->advertise<std_msgs::String>(NS + id + "/proxemic_space", 1, true);

  setAnonymous((id.rfind(hri::ANONYMOUS, 0) == 0) ? true : false);

  _tf_frame = _anonymous ? id : hri::PERSON + id;
}

ManagedPerson::~ManagedPerson()
{
  ROS_DEBUG_STREAM("Closing all topics related to person <" << _id);
  face_id_pub.shutdown();
  body_id_pub.shutdown();
  voice_id_pub.shutdown();
  alias_pub.shutdown();
  anonymous_pub.shutdown();
  loc_confidence_pub.shutdown();
  proxemics_pub.shutdown();
}

void ManagedPerson::setFaceId(ID id)
{
  if (id != _face_id)
  {
    ROS_INFO_STREAM("[person <" << _id << ">] face_id updated to <" << id << ">");
  }

  _face_id = id;
  id_msg.data = id;
  face_id_pub.publish(id_msg);
}

void ManagedPerson::setBodyId(ID id)
{
  if (id != _body_id)
  {
    ROS_INFO_STREAM("[person <" << _id << ">] body_id updated to <" << id << ">");
  }

  _body_id = id;
  id_msg.data = id;
  body_id_pub.publish(id_msg);
}

void ManagedPerson::setVoiceId(ID id)
{
  if (id != _voice_id)
  {
    ROS_INFO_STREAM("[person <" << _id << ">] voice_id updated to <" << id << ">");
  }
  _voice_id = id;
  id_msg.data = id;
  voice_id_pub.publish(id_msg);
}

void ManagedPerson::setAnonymous(bool anonymous)
{
  if (anonymous && _anonymous != anonymous)
  {
    ROS_WARN_STREAM("new anonymous person " << _id);
  }
  _anonymous = anonymous;
  bool_msg.data = anonymous;
  anonymous_pub.publish(bool_msg);
}

void ManagedPerson::setAlias(ID id)
{
  if (id != _alias)
  {
    ROS_INFO_STREAM("[person <" << _id << ">] set to be alias of <" << _alias << ">");
  }
  _alias = id;
  id_msg.data = id;
  alias_pub.publish(id_msg);
}

void ManagedPerson::setLocationConfidence(float confidence)
{
  _loc_confidence = confidence;
  float_msg.data = confidence;
  loc_confidence_pub.publish(float_msg);
}


void ManagedPerson::update(ID face_id, ID body_id, ID voice_id, chrono::milliseconds elapsed_time)
{
  // a person is considered 'actively tracked' if at least one of its face/body/voice is tracked
  _actively_tracked = !face_id.empty() || !body_id.empty() || !voice_id.empty();

  setFaceId(face_id);
  setBodyId(body_id);
  setVoiceId(voice_id);

  if (_actively_tracked)
  {
    _time_since_last_seen = chrono::milliseconds(0);
    if (_loc_confidence != 1.)
    {
      _loc_confidence = 1.;
      _loc_confidence_dirty = true;
    }
  }
  else  // *not* actively tracked
  {
    if (_time_since_last_seen > LIFETIME_UNTRACKED_PERSON)
    {
      if (_loc_confidence != 0.)
      {
        _loc_confidence = 0.;
        _loc_confidence_dirty = true;
        ROS_WARN_STREAM("[person <" << _id << ">] not seen for more than "
                                    << LIFETIME_UNTRACKED_PERSON.count()
                                    << ". Not publishing tf frame anymore.");
      }
    }
    else  // not tracked, but lifetime *not yet expired*
    {
      _time_since_last_seen += elapsed_time;
      _loc_confidence = 1. - _time_since_last_seen / LIFETIME_UNTRACKED_PERSON;
      _loc_confidence_dirty = true;
    }
  }



  if (_loc_confidence_dirty)
  {
    setLocationConfidence(_loc_confidence);
    _loc_confidence_dirty = false;
  }

  publishFrame();
}

void ManagedPerson::setProxemics(const string& target_frame)
{
  float distance = 0.f;
  try
  {
    auto transform =
        _tf_buffer->lookupTransform(_tf_robot_reference_frame, target_frame, ros::Time(0));

    distance = sqrt(transform.transform.translation.x * transform.transform.translation.x +
                    transform.transform.translation.y * transform.transform.translation.y +
                    transform.transform.translation.z * transform.transform.translation.z);

    if (distance < _proxemics_dist_personal)
    {
      _proxemic_zone = Proxemics::PROXEMICS_PERSONAL;
    }
    else
    {
      if (distance < _proxemics_dist_social)
      {
        _proxemic_zone = Proxemics::PROXEMICS_SOCIAL;
      }
      else
      {
        if (_proxemics_dist_public <= 0 || distance < _proxemics_dist_public)
        {
          _proxemic_zone = Proxemics::PROXEMICS_PUBLIC;
        }
        else
        {
          _proxemic_zone = Proxemics::PROXEMICS_UNKNOWN;
        }
      }
    }
  }
  catch (tf2::TransformException& ex)
  {
    ROS_WARN("%s", ex.what());
    _proxemic_zone = Proxemics::PROXEMICS_UNKNOWN;
  }

  std_msgs::String proxemic_msg;
  proxemic_msg.data = PROXEMICS.at(_proxemic_zone);
  proxemics_pub.publish(proxemic_msg);
}

void ManagedPerson::publishFrame()
{
  /////////////////////////////////////////////
  // publish TF frame of the person


  string target_frame;

  if (!_face_id.empty())
  {
    target_frame = string("face_") + _face_id;
  }
  else if (!_body_id.empty())
  {
    target_frame = string("head_") + _body_id;
  }
  else if (!_voice_id.empty())
  {
    target_frame = string("voice_") + _voice_id;
  }

  /////////////// BROADCASTING OF PERSON FRAME
  if (!target_frame.empty())
  {
    if (_tf_buffer->canTransform(_tf_reference_frame, target_frame, ros::Time(0)))
    {
      //     log management
      if (!_last_tf_broadcast_successful)
      {
        _need_log_tf_broadcast = true;
        _last_tf_broadcast_successful = true;
      }
      if (_need_log_tf_broadcast)
      {
        ROS_INFO_STREAM("[person <" << _id << ">] broadcast transform "
                                    << _tf_reference_frame << " <-> " << target_frame);
        _need_log_tf_broadcast = false;
      }
      //////////////////////////

      try
      {
        _transform =
            _tf_buffer->lookupTransform(_tf_reference_frame, target_frame, ros::Time(0));

        _transform.header.stamp = ros::Time::now();
        _transform.child_frame_id = _tf_frame;

        _tf_br.sendTransform(_transform);
        _had_transform_at_least_once = true;
      }
      catch (tf2::TransformException& ex)
      {
        ROS_WARN("%s", ex.what());
      }
    }
    else
    {
      //     log management
      if (_last_tf_broadcast_successful)
      {
        _need_log_tf_broadcast = true;
        _last_tf_broadcast_successful = false;
      }
      if (_need_log_tf_broadcast)
      {
        ROS_WARN_STREAM("[person <" << _id << ">] can not publish person "
                                    << "transform (either reference frame <"
                                    << _tf_reference_frame << "> or target frame <"
                                    << target_frame << "> are not available)");
        _need_log_tf_broadcast = false;
      }
      //////////////////////////
    }
  }
  else
  {
    if (!_had_transform_at_least_once)
    {
      ROS_INFO_STREAM("[person <" << _id << ">] no face, body or voice TF frame avail. Can not yet broadcast frame <"
                                  << _tf_frame << ">.");
    }
    else
    {
      // publish the last known transform, until loc_confidence == 0
      if (_loc_confidence > 0)
      {
        _transform.header.stamp = ros::Time::now();
        _tf_br.sendTransform(_transform);
      }
    }
  }

  /////////////// COMPUTATION OF DISTANCE TO ROBOT
  if (!target_frame.empty())
  {
    if (_tf_buffer->canTransform(_tf_robot_reference_frame, target_frame, ros::Time(0)))
    {
      //     log management
      if (!_last_distance_successful)
      {
        _need_log_distance = true;
        _last_distance_successful = true;
      }
      if (_need_log_distance)
      {
        ROS_INFO_STREAM("[person <" << _id << ">] distance to robot computed as "
                                    << _tf_robot_reference_frame << " <-> " << target_frame);
        _need_log_distance = false;
      }
      ///////////////////////////////

      try
      {
        setProxemics(target_frame);

        _had_computed_distance_at_least_once = true;
      }
      catch (tf2::TransformException& ex)
      {
        _proxemic_zone = Proxemics::PROXEMICS_UNKNOWN;
        ROS_WARN("%s", ex.what());
      }
    }
    else
    {
      _proxemic_zone = Proxemics::PROXEMICS_UNKNOWN;

      //     log management
      if (_last_distance_successful)
      {
        _need_log_distance = true;
        _last_distance_successful = false;
      }
      if (_need_log_distance)
      {
        ROS_WARN_STREAM("[person <" << _id << ">] can not compute distance (either reference frame <"
                                    << _tf_robot_reference_frame << "> or target frame <"
                                    << target_frame << "> are not available)");
        _need_log_distance = false;
      }
      /////////////////////////////
    }
  }
  else
  {
    if (!_had_computed_distance_at_least_once)
    {
      ROS_INFO_STREAM("[person <"
                      << _id << ">] no face, body or voice TF frame avail. Can not yet compute distance to robot.");
    }
    else
    {
      // publish the last known transform, until loc_confidence == 0
      if (_loc_confidence > 0 &&
          _tf_buffer->canTransform(_tf_robot_reference_frame, _tf_frame, ros::Time(0)))
      {
        setProxemics(_tf_frame);
      }
    }
  }
}
