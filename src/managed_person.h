#include <iostream>
#include <chrono>
#include <string>
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

class ManagedPerson
{
public:
  ManagedPerson(ros::NodeHandle& nh, hri::ID id, tf2_ros::Buffer& tf_buffer,
                const std::string& reference_frame);

  ~ManagedPerson();

  void setFaceId(hri::ID id);
  void setBodyId(hri::ID id);
  void setVoiceId(hri::ID id);

  void setAnonymous(bool anonymous);
  bool anonymous() const
  {
    return _anonymous;
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

  bool _actively_tracked;

  std::string _tf_frame;
  std::string _tf_reference_frame;

  tf2_ros::Buffer* _tf_buffer;
  tf2_ros::TransformBroadcaster _tf_br;

  geometry_msgs::TransformStamped _transform;
  bool _had_transform_at_least_once;

  hri::ID _face_id;
  hri::ID _body_id;
  hri::ID _voice_id;
  float _loc_confidence;
  bool _loc_confidence_dirty;
  bool _anonymous;

  std_msgs::String id_msg;
  std_msgs::Float32 float_msg;
  std_msgs::Bool bool_msg;

  hri::ID alias;

  std::chrono::milliseconds _time_since_last_seen;
};


}  // namespace hri
