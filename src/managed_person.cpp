#include "managed_person.h"
#include <chrono>

using namespace std;
using namespace ros;
using namespace hri;

ManagedPerson::ManagedPerson(NodeHandle& nh, ID id, tf2_ros::Buffer& tf_buffer,
                             const string& reference_frame)
  : _nh(&nh)
  , _id(id)
  , _actively_tracked(false)
  , _tf_reference_frame(reference_frame)
  , _tf_buffer(&tf_buffer)
  , _had_transform_at_least_once(false)
  , _loc_confidence(0.)
  , _loc_confidence_dirty(false)
  , _anonymous(false)
  , _time_since_last_seen(0)
{
  face_id_pub = _nh->advertise<std_msgs::String>(NS + id + "/face_id", 1, true);
  body_id_pub = _nh->advertise<std_msgs::String>(NS + id + "/body_id", 1, true);
  voice_id_pub = _nh->advertise<std_msgs::String>(NS + id + "/voice_id", 1, true);
  alias_pub = _nh->advertise<std_msgs::String>(NS + id + "/alias", 1, true);
  anonymous_pub = _nh->advertise<std_msgs::Bool>(NS + id + "/anonymous", 1, true);
  loc_confidence_pub = _nh->advertise<std_msgs::Float32>(NS + id + "/location_confidence", 1);


  setAnonymous((id.substr(0, ANONYMOUS.size()) == ANONYMOUS) ? true : false);

  _tf_frame = _anonymous ? id : hri::PERSON + id;
}

ManagedPerson::~ManagedPerson()
{
  face_id_pub.shutdown();
  body_id_pub.shutdown();
  voice_id_pub.shutdown();
  alias_pub.shutdown();
  anonymous_pub.shutdown();
  loc_confidence_pub.shutdown();
}

void ManagedPerson::setFaceId(ID id)
{
  _face_id = id;
  id_msg.data = id;
  face_id_pub.publish(id_msg);

  ROS_INFO_STREAM(" - face_id: " << id);
}

void ManagedPerson::setBodyId(ID id)
{
  _body_id = id;
  id_msg.data = id;
  body_id_pub.publish(id_msg);

  ROS_INFO_STREAM(" - body_id: " << id);
}

void ManagedPerson::setVoiceId(ID id)
{
  _voice_id = id;
  id_msg.data = id;
  voice_id_pub.publish(id_msg);

  ROS_INFO_STREAM(" - voice_id: " << id);
}

void ManagedPerson::setAnonymous(bool anonymous)
{
  _anonymous = anonymous;
  bool_msg.data = anonymous;
  anonymous_pub.publish(bool_msg);

  if (anonymous)
  {
    ROS_WARN_STREAM("new anonymous person " << _id);
  }
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
        ROS_INFO_STREAM("Not seen person " << _id << " for more than "
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

  if (!target_frame.empty())
  {
    if (_tf_buffer->canTransform(_tf_reference_frame, target_frame, ros::Time(0)))
    {
      ROS_INFO_STREAM(" - broadcast transform " << _tf_reference_frame << " <-> " << target_frame);
      try
      {
        _transform =
            _tf_buffer->lookupTransform(_tf_reference_frame, target_frame, ros::Time(0));

        _transform.header.stamp = ros::Time::now();
        _transform.child_frame_id = _tf_frame;

        _tf_br.sendTransform(_transform);
        _had_transform_at_least_once = true;
      }
      catch (tf2::TransformException ex)
      {
        ROS_WARN("%s", ex.what());
      }
    }
    else
    {
      ROS_INFO_STREAM(" - can not publish transform\n   (either reference frame "
                      << _tf_reference_frame << " or target frame " << target_frame
                      << " are not yet available)");
    }
  }
  else
  {
    if (!_had_transform_at_least_once)
    {
      ROS_DEBUG_STREAM(" - No face, body or voice TF frame published for person "
                       << _id << ". Can not yet broadcast frame " << _tf_frame << ".");
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
}

