#include "managed_person.h"
#include <chrono>

using namespace std;
using namespace ros;
using namespace hri;

ManagedPerson::ManagedPerson(ros::NodeHandle& nh, hri::ID id, ros::Publisher face_id_pub,
                             ros::Publisher body_id_pub, ros::Publisher voice_id_pub,
                             ros::Publisher alias_pub, ros::Publisher anonymous_pub,
                             ros::Publisher loc_confidence_pub, tf2_ros::Buffer& tf_buffer,
                             const std::string& reference_frame)
  : _nh(&nh)
  , _id(id)
  , face_id_pub(face_id_pub)
  , body_id_pub(body_id_pub)
  , voice_id_pub(voice_id_pub)
  , alias_pub(alias_pub)
  , anonymous_pub(anonymous_pub)
  , loc_confidence_pub(loc_confidence_pub)
  , _actively_tracked(false)
  , _tf_reference_frame(reference_frame)
  , _tf_buffer(&tf_buffer)
  , _had_transform_at_least_once(false)
  , _loc_confidence(0.)
  , _loc_confidence_dirty(false)
  , _anonymous(false)
  , _time_since_last_seen(0)
{
  id_msg.header.id = _id;
  float_msg.header.id = _id;
  bool_msg.header.id = _id;

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

  id_msg.header.header.stamp = ros::Time::now();
  float_msg.header.header.stamp = ros::Time::now();
  bool_msg.header.header.stamp = ros::Time::now();

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
      ROS_INFO_STREAM_ONCE("[person <" << _id << ">] broadcast transform "
                                       << _tf_reference_frame << " <-> " << target_frame);
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
      ROS_INFO_STREAM_ONCE("[person <" << _id << ">] can not publish transform (either reference frame <"
                                       << _tf_reference_frame << "> or target frame <"
                                       << target_frame << "> are not available)");
    }
  }
  else
  {
    if (!_had_transform_at_least_once)
    {
      ROS_INFO_STREAM_ONCE("[person <" << _id << ">] no face, body or voice TF frame avail. Can not yet broadcast frame <"
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
}

