// Copyright 2021 PAL Robotics S.L.
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

#include <gtest/gtest.h>
#include <gmock/gmock.h>
#include <thread>
#include <chrono>

#include <ros/ros.h>
#include <std_srvs/Empty.h>
#include <geometry_msgs/TransformStamped.h>
#include <tf2_ros/transform_broadcaster.h>

#include "hri/hri.h"
#include "hri/base.h"
#include "hri_msgs/IdsMatch.h"
#include "person_matcher.h"
#include "managed_person.h"


using namespace hri;
using namespace std;
using namespace ros;
using namespace tf2_ros;

// waiting time for the libhri callback to process their inputs
#define WAIT(X) std::this_thread::sleep_for(std::chrono::milliseconds(X))

#define DEBUG_WAIT(X)                                                                    \
  {                                                                                      \
    ROS_WARN("waiting...");                                                              \
    std::this_thread::sleep_for(std::chrono::milliseconds(X));                           \
    ROS_WARN("done waiting");                                                            \
  }

// defined in mermaid.cpp
void run_mermaid_tests(string path);

//////////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////

void publish_feature_position(tf2_ros::TransformBroadcaster& br, const std::string& frame,
                              float x, float y = 0., float z = 0.,
                              const std::string& base_frame = "base_link")
{
  // publish the face's pose
  geometry_msgs::TransformStamped transform;
  transform.header.stamp = ros::Time::now() - ros::Duration(0.1);
  transform.header.frame_id = base_frame;
  transform.child_frame_id = frame;
  transform.transform.translation.x = x;
  transform.transform.translation.y = y;
  transform.transform.translation.z = z;
  transform.transform.rotation.w = 1;


  // need to send the transform *2 times* so that TF can interpolate
  // the timestamp of future lookups *must* be in the published interval -> we
  // set the timestamp of the 2nd transform one second in the future to leave
  // time for the unittest to complete & query TF as much as needed.
  br.sendTransform(transform);
  WAIT(50);
  transform.header.stamp = ros::Time::now() + ros::Duration(1.);
  br.sendTransform(transform);
}



TEST(hri_person_matcher, BasicAssociationModel)
{
  {
    PersonMatcher model(0.4);

    EXPECT_ANY_THROW({ model.get_association("p1"); });

    Relations data = { { "p1", person, "f1", face, 1.0 } };
    model.update(data);
    auto association = model.get_association("p1");
    EXPECT_EQ(association, (map<FeatureType, ID>{ { hri::face, "f1" } }));
    EXPECT_TRUE(association.find(hri::voice) == association.end());
    EXPECT_TRUE(association.find(hri::body) == association.end());

    model.update({ { "p1", person, "f1", face, 0.9 } });
    association = model.get_association("p1");
    EXPECT_EQ(association, (map<FeatureType, ID>{ { hri::face, "f1" } }));
    EXPECT_TRUE(association.find(hri::voice) == association.end());
    EXPECT_TRUE(association.find(hri::body) == association.end());

    model.update({ { "p1", person, "f1", face, 0.4 } });
    association = model.get_association("p1");
    EXPECT_EQ(association, (map<FeatureType, ID>{ { hri::face, "f1" } }));
    EXPECT_TRUE(association.find(hri::voice) == association.end());
    EXPECT_TRUE(association.find(hri::body) == association.end());

    model.update({ { "p1", person, "f1", face, 0.35 } });
    association = model.get_association("p1");
    EXPECT_TRUE(association.empty());

    model.update({ { "p1", person, "f1", face, 0.1 } });
    association = model.get_association("p1");
    EXPECT_TRUE(association.empty());

    model.update({ { "p1", person, "f1", face, 0.0 } });
    association = model.get_association("p1");
    EXPECT_TRUE(association.empty());

    model.update({ { "p1", person, "f1", face, 0.9 } });
    association = model.get_association("p1");
    EXPECT_EQ(association, (map<FeatureType, ID>{ { hri::face, "f1" } }));
    EXPECT_TRUE(association.find(hri::voice) == association.end());
    EXPECT_TRUE(association.find(hri::body) == association.end());
  }

  {
    PersonMatcher model(0.05);

    model.update({ { "p1", person, "f1", face, 0.1 } });
    auto association = model.get_association("p1");
    EXPECT_EQ(association, (map<FeatureType, ID>{ { hri::face, "f1" } }));
    EXPECT_TRUE(association.find(hri::voice) == association.end());
    EXPECT_TRUE(association.find(hri::body) == association.end());

    model.update({ { "p1", person, "f1", face, 0.01 } });
    association = model.get_association("p1");
    EXPECT_TRUE(association.empty());

    model.update({ { "p1", person, "f1", face, 0.0 } });
    association = model.get_association("p1");
    EXPECT_TRUE(association.empty());
  }
}


//////////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////

TEST(hri_person_matcher, AssociationNetwork)
{
  auto model = PersonMatcher(0.4);


  // Test small transitive network, with p1 -> {f1, b1} more likely than p1 -> {f1, b2}
  Relations data = { { "f1", face, "b1", body, 0.7 },
                     { "f1", face, "b2", body, 0.6 },
                     { "p1", person, "f1", face, 0.9 } };
  model.update(data);
  auto association = model.get_association("p1");
  EXPECT_EQ(association, (map<FeatureType, ID>{ { hri::face, "f1" }, { hri::body, "b1" } }));
  EXPECT_TRUE(association.find(hri::voice) == association.end());

  // Test *updating* an edge. The previous value should be replaced
  // A: a computed edge f1 -- 0.63 -- p1 was added -> p1 should still be
  // associated to f1 and b1
  data = { { "f1", face, "b1", body, 0.0 } };
  model.update(data);
  association = model.get_association("p1");
  EXPECT_EQ(association, (map<FeatureType, ID>{ { hri::face, "f1" }, { hri::body, "b1" } }));
  EXPECT_TRUE(association.find(hri::voice) == association.end());

  data = { { "f1", face, "b2", body, 0.64 } };
  // B: now, the relation f1 <-> b2 is stronger than f1 <-> b1: p1 should be
  // associated to f1 and b2
  model.update(data);
  association = model.get_association("p1");
  EXPECT_EQ(association, (map<FeatureType, ID>{ { hri::face, "f1" }, { hri::body, "b2" } }));
  EXPECT_TRUE(association.find(hri::voice) == association.end());


  // this time, the likelihood of p1 being associated to b2 is < threshold (0.9
  // * 0.4 < 0.4)
  // p1 should be associated back to b1
  data = { { "f1", face, "b2", body, 0.4 } };
  model.update(data);
  association = model.get_association("p1");
  EXPECT_EQ(association, (map<FeatureType, ID>{ { hri::face, "f1" }, { hri::body, "b1" } }));
  EXPECT_TRUE(association.find(hri::voice) == association.end());
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////

TEST(hri_person_matcher, ResetEraseIds)
{
  auto model = PersonMatcher(0.4);

  Relations data = { { "p1", person, "f1", face, 0.9 } };
  model.update(data);
  auto association = model.get_association("p1");
  EXPECT_EQ(association, (map<FeatureType, ID>{ { hri::face, "f1" } }));

  model.erase("f1");
  association = model.get_association("p1");
  EXPECT_TRUE(association.empty());

  model.erase("p1");
  EXPECT_ANY_THROW({ model.get_association("p1"); });

  data = { { "p1", person, "f1", face, 0.9 },
           { "f1", face, "b2", body, 0.6 },
           { "b1", body, "f1", face, 0.7 } };

  model.update(data);
  association = model.get_association("p1");
  EXPECT_EQ(association, (map<FeatureType, ID>{ { hri::face, "f1" }, { hri::body, "b1" } }));

  model.erase("b1");
  association = model.get_association("p1");
  EXPECT_EQ(association, (map<FeatureType, ID>{ { hri::face, "f1" }, { hri::body, "b2" } }));

  model.update({ { "f1", face, "b1", body, 0.7 } });
  association = model.get_association("p1");
  EXPECT_EQ(association, (map<FeatureType, ID>{ { hri::face, "f1" }, { hri::body, "b1" } }));

  model.reset();
  EXPECT_ANY_THROW({ model.get_association("p1"); });

  model.update(data);
  association = model.get_association("p1");
  EXPECT_EQ(association, (map<FeatureType, ID>{ { hri::face, "f1" }, { hri::body, "b1" } }));

  model.reset();
  EXPECT_ANY_THROW({ model.get_association("p1"); });

  model.update(data);
  association = model.get_association("p1");
  EXPECT_EQ(association, (map<FeatureType, ID>{ { hri::face, "f1" }, { hri::body, "b1" } }));
}


//////////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////
TEST(hri_person_manager, mermaid_graphs)
{
  run_mermaid_tests("test-graphs.md");
}


//////////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////

TEST(hri_person_manager, ROSNode)
{
  NodeHandle nh;

  ros::AsyncSpinner spinner(1);
  spinner.start();

  HRIListener hri_listener;
  WAIT(100);

  Publisher pub = nh.advertise<hri_msgs::IdsMatch>("/humans/candidate_matches", 1);

  // wait for the hri_person_manager node to be up
  WAIT(500);

  ASSERT_EQ(pub.getNumSubscribers(), 1)
      << "hri_person_manager should be the one and only node subscribed to this topic";

  ASSERT_EQ(hri_listener.getPersons().size(), 0);


  hri_msgs::IdsMatch match;
  match.id1 = "f1";
  match.id1_type = hri_msgs::IdsMatch::FACE;
  match.id2 = "p1";
  match.id2_type = hri_msgs::IdsMatch::PERSON;

  match.confidence = 0.7;

  pub.publish(match);

  WAIT(200);

  ASSERT_EQ(hri_listener.getPersons().size(), 1);

  auto persons = hri_listener.getPersons();
  ASSERT_TRUE(persons.find("p1") != persons.end());

  auto p1 = persons["p1"].lock();
  ASSERT_FALSE(p1->face().lock())
      << "the face has not yet been published -> can not be associated to the person yet.";

  ASSERT_FALSE(p1->body().lock());
  ASSERT_FALSE(p1->voice().lock());


  // publish the face
  auto face_pub = nh.advertise<hri_msgs::IdsList>("/humans/faces/tracked", 1);
  auto ids = hri_msgs::IdsList();
  ids.ids = { "f1" };
  face_pub.publish(ids);

  // wait for lihri to pick up the new face
  WAIT(250);

  ASSERT_TRUE(p1->face().lock())
      << "the face has been published -> should now be associated to the person.";


  ASSERT_FALSE(p1->face().expired());
  ASSERT_EQ(p1->face().lock()->id(), "f1");

  ids.ids = { "f1", "f2" };
  face_pub.publish(ids);

  WAIT(200);

  auto face_f1 = p1->face();

  ASSERT_FALSE(face_f1.expired());
  ASSERT_EQ(face_f1.lock()->id(), "f1");

  match.id1 = "f2";
  match.id1_type = hri_msgs::IdsMatch::FACE;
  match.id2 = "p1";
  match.id2_type = hri_msgs::IdsMatch::PERSON;
  match.confidence = 0.8;

  pub.publish(match);

  WAIT(100);

  auto face_f2 = p1->face();

  ASSERT_FALSE(face_f1.expired()) << "face 'f1' still exists";
  ASSERT_FALSE(face_f2.expired()) << "face 'f2' should now be the mostly likely face of 'p1'";
  ASSERT_EQ(face_f2.lock()->id(), "f2");

  spinner.stop();
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////

TEST(hri_person_manager, Proxemics)
{
  NodeHandle nh;

  ros::AsyncSpinner spinner(1);
  spinner.start();

  ros::ServiceClient reset_srv = nh.serviceClient<std_srvs::Empty>("/hri_person_manager/reset");
  // clear the hri_person_manager
  std_srvs::Empty empty;
  reset_srv.call(empty);
  WAIT(100);

  tf2_ros::TransformBroadcaster br;

  vector<string> personal_space;
  vector<string> social_space;
  vector<string> public_space;

  Subscriber personal_prox_sub =
      nh.subscribe<hri_msgs::IdsList>("/humans/persons/in_personal_space", 1,
                                      [&personal_space](hri_msgs::IdsListConstPtr people) -> void
                                      { personal_space = people->ids; });
  Subscriber social_prox_sub =
      nh.subscribe<hri_msgs::IdsList>("/humans/persons/in_social_space", 1,
                                      [&social_space](hri_msgs::IdsListConstPtr people) -> void
                                      { social_space = people->ids; });
  Subscriber public_prox_sub =
      nh.subscribe<hri_msgs::IdsList>("/humans/persons/in_public_space", 1,
                                      [&public_space](hri_msgs::IdsListConstPtr people) -> void
                                      { public_space = people->ids; });



  HRIListener hri_listener;
  auto face_pub = nh.advertise<hri_msgs::IdsList>("/humans/faces/tracked", 1, true);
  auto ids = hri_msgs::IdsList();

  WAIT(100);

  map<hri::ID, tuple<float, Proxemics>> test_faces = {
    { "f1", { DEFAULT_PERSONAL_DISTANCE - 0.1, PROXEMICS_PERSONAL } },
    { "f2", { DEFAULT_PERSONAL_DISTANCE, PROXEMICS_PERSONAL } },
    { "f3", { DEFAULT_PERSONAL_DISTANCE + 0.1, PROXEMICS_SOCIAL } },
    { "f4", { DEFAULT_SOCIAL_DISTANCE - 0.1, PROXEMICS_SOCIAL } },
    { "f5", { DEFAULT_SOCIAL_DISTANCE, PROXEMICS_SOCIAL } },
    { "f6", { DEFAULT_SOCIAL_DISTANCE + 0.1, PROXEMICS_PUBLIC } },
    { "f7", { DEFAULT_PUBLIC_DISTANCE - 0.1, PROXEMICS_PUBLIC } },
    { "f8", { DEFAULT_PUBLIC_DISTANCE + 0.1, PROXEMICS_UNKNOWN } },
  };

  for (auto const& face : test_faces)
  {
    auto& name = face.first;
    auto person_name = "anonymous_person_" + generate_hash_id(name);
    auto frame = "face_" + name;
    float dist = get<0>(face.second);
    auto proxemics = get<1>(face.second);

    ROS_WARN_STREAM("Testing face " << name << " at distance " << dist
                                    << "m from the robot -> expecting proximal zone '"
                                    << PROXEMICS.at(proxemics) << "'.");

    ids.ids = { name };
    face_pub.publish(ids);
    // wait for lihri to pick up the new face
    WAIT(200);
    publish_feature_position(br, frame, dist);
    WAIT(100);

    switch (proxemics)
    {
      case PROXEMICS_PERSONAL:
        ASSERT_TRUE(find(personal_space.begin(), personal_space.end(), person_name) !=
                    personal_space.end());
        ASSERT_FALSE(find(social_space.begin(), social_space.end(), person_name) !=
                     social_space.end());
        ASSERT_FALSE(find(public_space.begin(), public_space.end(), person_name) !=
                     public_space.end());
        break;
      case PROXEMICS_SOCIAL:
        ASSERT_FALSE(find(personal_space.begin(), personal_space.end(), person_name) !=
                     personal_space.end());
        ASSERT_TRUE(find(social_space.begin(), social_space.end(), person_name) !=
                    social_space.end());
        ASSERT_FALSE(find(public_space.begin(), public_space.end(), person_name) !=
                     public_space.end());
        break;
      case PROXEMICS_PUBLIC:
        ASSERT_FALSE(find(personal_space.begin(), personal_space.end(), person_name) !=
                     personal_space.end());
        ASSERT_FALSE(find(social_space.begin(), social_space.end(), person_name) !=
                     social_space.end());
        ASSERT_TRUE(find(public_space.begin(), public_space.end(), person_name) !=
                    public_space.end());
        break;
      case PROXEMICS_UNKNOWN:
        ASSERT_FALSE(find(personal_space.begin(), personal_space.end(), person_name) !=
                     personal_space.end());
        ASSERT_FALSE(find(social_space.begin(), social_space.end(), person_name) !=
                     social_space.end());
        ASSERT_FALSE(find(public_space.begin(), public_space.end(), person_name) !=
                     public_space.end());
        break;
    }

    personal_space.clear();
    social_space.clear();
    public_space.clear();
  }

  spinner.stop();
}


//////////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////

TEST(hri_person_manager, ROSReset)
{
  NodeHandle nh;

  ros::ServiceClient reset_srv = nh.serviceClient<std_srvs::Empty>("/hri_person_manager/reset");

  ros::AsyncSpinner spinner(1);
  spinner.start();


  HRIListener hri_listener;
  WAIT(100);

  // clear the hri_person_manager
  std_srvs::Empty empty;
  reset_srv.call(empty);


  Publisher pub = nh.advertise<hri_msgs::IdsMatch>("/humans/candidate_matches", 1);

  // publish a face
  auto face_pub = nh.advertise<hri_msgs::IdsList>("/humans/faces/tracked", 1);
  auto ids = hri_msgs::IdsList();
  ids.ids = { "f1" };
  face_pub.publish(ids);


  // wait for the hri_person_manager node to be up
  WAIT(500);

  hri_msgs::IdsMatch match;
  match.id1 = "f1";
  match.id1_type = hri_msgs::IdsMatch::FACE;
  match.id2 = "p1";
  match.id2_type = hri_msgs::IdsMatch::PERSON;
  match.confidence = 0.7;

  hri_msgs::IdsMatch match2;
  match2.id1 = "f2";
  match2.id1_type = hri_msgs::IdsMatch::FACE;
  match2.id2 = "p2";
  match2.id2_type = hri_msgs::IdsMatch::PERSON;
  match2.confidence = 0.7;

  pub.publish(match);
  pub.publish(match2);

  // long wait because hri_person_manager 'slow-ish' ROS loop
  WAIT(400);

  ASSERT_EQ(hri_listener.getPersons().size(), 2);
  ASSERT_EQ(hri_listener.getTrackedPersons().size(), 2);
  auto persons = hri_listener.getPersons();
  ASSERT_TRUE(persons.find("p1") != persons.end());

  {
    auto p1 = persons["p1"].lock();
    ASSERT_TRUE(p1->face().lock());
  }

  // clear the hri_person_manager
  reset_srv.call(empty);

  WAIT(400);
  ASSERT_EQ(hri_listener.getPersons().size(), 0);

  ASSERT_FALSE(persons["p1"].lock()) << "the old pointer should now be invalid";

  // re-publishing one match
  pub.publish(match);
  WAIT(400);

  ASSERT_EQ(hri_listener.getPersons().size(), 1);
  persons = hri_listener.getPersons();
  ASSERT_TRUE(persons.find("p1") != persons.end());

  {
    auto p1 = persons["p1"].lock();
    ASSERT_TRUE(p1->face().lock());
  }

  // clear the hri_person_manager
  reset_srv.call(empty);

  WAIT(100);
  ASSERT_EQ(hri_listener.getPersons().size(), 0);



  spinner.stop();
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////

TEST(hri_person_manager, AnonymousPersons)
{
  NodeHandle nh;

  ros::ServiceClient reset_srv = nh.serviceClient<std_srvs::Empty>("/hri_person_manager/reset");

  ros::AsyncSpinner spinner(1);
  spinner.start();


  HRIListener hri_listener;
  WAIT(100);

  Publisher pub = nh.advertise<hri_msgs::IdsMatch>("/humans/candidate_matches", 1);
  Publisher faces_pub = nh.advertise<hri_msgs::IdsList>("/humans/faces/tracked", 1);
  Publisher bodies_pub = nh.advertise<hri_msgs::IdsList>("/humans/bodies/tracked", 1);
  Publisher voices_pub = nh.advertise<hri_msgs::IdsList>("/humans/voices/tracked", 1);


  // wait for the hri_person_manager node to be up
  WAIT(500);
  //
  // clear the hri_person_manager
  std_srvs::Empty empty;
  reset_srv.call(empty);

  ////////////////////////////////////////////////////////////////////////////////////////////////////

  WAIT(400);
  auto persons = hri_listener.getPersons();
  ASSERT_EQ(persons.size(), 0) << "no one should be there yet";


  // publish a face
  auto ids = hri_msgs::IdsList();
  ids.ids = { "f1" };
  faces_pub.publish(ids);

  WAIT(400);

  persons = hri_listener.getPersons();
  ASSERT_EQ(persons.size(), 1) << "exactly one anonymous person should have been created";

  auto anon_id = persons.begin()->first;

  ASSERT_TRUE(anon_id.rfind(hri::ANONYMOUS, 0) == 0)
      << "the anonymous person ID should start with " << hri::ANONYMOUS << ". Got: " << anon_id;

  ASSERT_TRUE(persons[anon_id].lock());
  auto p1 = persons[anon_id].lock();
  ASSERT_TRUE(p1);
  ASSERT_TRUE(p1->anonymous());
  ASSERT_TRUE(p1->face().lock()) << "the anonymous person should be associated to its face";
  ASSERT_EQ(p1->face().lock()->id(), "f1");

  // remove the face
  ids = hri_msgs::IdsList();
  ids.ids = {};
  faces_pub.publish(ids);

  WAIT(400);

  persons = hri_listener.getPersons();
  ASSERT_EQ(persons.size(), 0) << "the anonymous person should have disappeared since its face is not detected anymore";

  ////////////////////////////////////////////////////////////////////////////////////////////////////

  ids = hri_msgs::IdsList();
  ids.ids = { "f1" };
  faces_pub.publish(ids);

  WAIT(400);

  ASSERT_EQ(hri_listener.getPersons().size(), 1);

  hri_msgs::IdsMatch match;
  match.id1 = "f1";
  match.id1_type = hri_msgs::IdsMatch::FACE;
  match.id2 = "p1";
  match.id2_type = hri_msgs::IdsMatch::PERSON;
  match.confidence = 0.8;

  pub.publish(match);

  WAIT(400);

  persons = hri_listener.getPersons();
  ASSERT_EQ(persons.size(), 1) << "the anonymous person associated to 'f1' should have disappeared, since face 'f1' is now associated to a person";

  ASSERT_TRUE(persons.find("p1") != persons.end());
  ASSERT_TRUE(persons.find(anon_id) == persons.end())
      << "the anonymous person associated to 'f1' should have disappeared, since face 'f1' is now associated to a person";

  ////////////////////////////////////////////////////////////////////////////////////////////////////

  match.id1 = "f1";
  match.id1_type = hri_msgs::IdsMatch::FACE;
  match.id2 = "p1";
  match.id2_type = hri_msgs::IdsMatch::PERSON;
  match.confidence = 0.0;

  pub.publish(match);

  WAIT(400);

  ASSERT_EQ(hri_listener.getPersons().size(), 2)
      << "face 'f1' is not associated to an actual person; it should re-create an anonymous person";


  ids.ids = { "f1", "f2" };
  faces_pub.publish(ids);
  WAIT(400);
  ASSERT_EQ(hri_listener.getPersons().size(), 3)
      << "2 anonymous persons are expected, one for each face f1 and f2";

  ids.ids = { "b1" };
  bodies_pub.publish(ids);
  WAIT(400);
  ASSERT_EQ(hri_listener.getPersons().size(), 4)
      << "a 3rd anonymous person should have been created for body b1";

  match.id1 = "f2";
  match.id1_type = hri_msgs::IdsMatch::FACE;
  match.id2 = "b1";
  match.id2_type = hri_msgs::IdsMatch::BODY;
  match.confidence = 0.8;

  pub.publish(match);

  WAIT(400);

  persons = hri_listener.getPersons();

  ASSERT_EQ(persons.size(), 3) << "f2 and b1 are now associated: one of the 2 anonymous persons should have disappeared";

  ids.ids = { "f2" };
  faces_pub.publish(ids);
  WAIT(400);
  ASSERT_EQ(hri_listener.getPersons().size(), 2)
      << "only one anonymous person, associated to f2 and b1 should remain, in addition to p1";

  ////////////////////////////////////////////////////////////////////////////////////////////////////

  match.id1 = "b1";
  match.id1_type = hri_msgs::IdsMatch::BODY;
  match.id2 = "p2";
  match.id2_type = hri_msgs::IdsMatch::PERSON;
  match.confidence = 0.8;

  pub.publish(match);

  WAIT(400);
  persons = hri_listener.getPersons();
  ASSERT_EQ(persons.size(), 2) << "two non-anonymous person p1 and p2 (associated to f2 and b1) should remain";

  ASSERT_TRUE(persons.find("p1") != persons.end());
  ASSERT_TRUE(persons.find("p2") != persons.end());

  spinner.stop();
}

TEST(hri_person_manager, AnonymousPersons2)
{
  NodeHandle nh;

  ros::ServiceClient reset_srv = nh.serviceClient<std_srvs::Empty>("/hri_person_manager/reset");

  ros::AsyncSpinner spinner(1);
  spinner.start();


  HRIListener hri_listener;
  WAIT(100);

  Publisher pub = nh.advertise<hri_msgs::IdsMatch>("/humans/candidate_matches", 1);
  Publisher faces_pub = nh.advertise<hri_msgs::IdsList>("/humans/faces/tracked", 1);


  // wait for the hri_person_manager node to be up
  WAIT(500);


  // clear the hri_person_manager
  std_srvs::Empty empty;
  reset_srv.call(empty);

  auto ids = hri_msgs::IdsList();
  ids.ids = { "f1", "f2" };
  faces_pub.publish(ids);

  WAIT(400);

  auto persons = hri_listener.getPersons();
  ASSERT_EQ(persons.size(), 2);

  ids = hri_msgs::IdsList();
  ids.ids = { "f1" };
  faces_pub.publish(ids);

  WAIT(400);

  persons = hri_listener.getPersons();
  ASSERT_EQ(persons.size(), 1);

  spinner.stop();
}

TEST(hri_person_manager, AnonymousPersons3)
{
  NodeHandle nh;

  ros::ServiceClient reset_srv = nh.serviceClient<std_srvs::Empty>("/hri_person_manager/reset");

  ros::AsyncSpinner spinner(1);
  spinner.start();


  HRIListener hri_listener;
  WAIT(100);

  Publisher faces_pub = nh.advertise<hri_msgs::IdsList>("/humans/faces/tracked", 1);
  Publisher bodies_pub = nh.advertise<hri_msgs::IdsList>("/humans/bodies/tracked", 1);

  // wait for the hri_person_manager node to be up
  WAIT(500);


  // clear the hri_person_manager
  std_srvs::Empty empty;
  reset_srv.call(empty);


  auto ids = hri_msgs::IdsList();
  ids.ids = { "f1" };
  faces_pub.publish(ids);

  WAIT(400);

  auto persons = hri_listener.getPersons();
  ASSERT_EQ(persons.size(), 1);

  auto anon_id_1 = persons.begin()->first;

  {
    auto p_f1 = hri_listener.getPersons()[anon_id_1].lock();
    ASSERT_FALSE(p_f1->body().lock());
    ASSERT_TRUE(p_f1->face().lock());
  }

  ids.ids = {};
  faces_pub.publish(ids);
  ids.ids = { "b1" };
  bodies_pub.publish(ids);

  WAIT(400);

  persons = hri_listener.getPersons();
  ASSERT_EQ(persons.size(), 1);

  auto anon_id_2 = persons.begin()->first;
  ASSERT_NE(anon_id_1, anon_id_2) << "two different anonymous persons should have been created";

  {
    auto p_b1 = hri_listener.getPersons()[anon_id_2].lock();
    ASSERT_TRUE(p_b1->body().lock());
    ASSERT_FALSE(p_b1->face().lock());
  }

  spinner.stop();
}


TEST(hri_person_manager, AnonymousPersonsAdvanced)
{
  NodeHandle nh;

  ros::ServiceClient reset_srv = nh.serviceClient<std_srvs::Empty>("/hri_person_manager/reset");

  ros::AsyncSpinner spinner(1);
  spinner.start();


  HRIListener hri_listener;
  WAIT(100);

  Publisher pub = nh.advertise<hri_msgs::IdsMatch>("/humans/candidate_matches", 1);
  Publisher faces_pub = nh.advertise<hri_msgs::IdsList>("/humans/faces/tracked", 1);
  Publisher bodies_pub = nh.advertise<hri_msgs::IdsList>("/humans/bodies/tracked", 1);
  Publisher voices_pub = nh.advertise<hri_msgs::IdsList>("/humans/voices/tracked", 1);


  // wait for the hri_person_manager node to be up
  WAIT(500);
  //
  // clear the hri_person_manager
  std_srvs::Empty empty;
  reset_srv.call(empty);

  auto ids = hri_msgs::IdsList();
  ids.ids = { "f1" };
  faces_pub.publish(ids);

  WAIT(200);

  auto persons = hri_listener.getPersons();
  ASSERT_EQ(persons.size(), 1);

  auto anon_id = persons.begin()->first;

  persons = hri_listener.getTrackedPersons();
  ASSERT_EQ(persons.size(), 1);
  ASSERT_TRUE(persons.find(anon_id) != persons.end());

  ids.ids = { "b1" };
  bodies_pub.publish(ids);

  WAIT(400);

  persons = hri_listener.getPersons();
  ASSERT_EQ(persons.size(), 2);

  persons = hri_listener.getTrackedPersons();
  ASSERT_EQ(persons.size(), 2);

  hri_msgs::IdsMatch match;
  match.id1 = "f1";
  match.id1_type = hri_msgs::IdsMatch::FACE;
  match.id2 = "b1";
  match.id2_type = hri_msgs::IdsMatch::BODY;
  match.confidence = 1.0;

  pub.publish(match);

  WAIT(400);

  persons = hri_listener.getPersons();

  auto p_f1 = persons[anon_id].lock();
  ASSERT_TRUE(p_f1);
  ASSERT_TRUE(p_f1->anonymous());
  ASSERT_TRUE(p_f1->face().lock()) << "the anonymous person should be associated to its face";
  ASSERT_EQ(p_f1->face().lock()->id(), "f1");

  // remove the face
  ids = hri_msgs::IdsList();
  ids.ids = {};
  faces_pub.publish(ids);

  WAIT(400);

  persons = hri_listener.getPersons();
  ASSERT_EQ(persons.size(), 0) << "the anonymous person should have disappeared since its face is not detected anymore";

  ids = hri_msgs::IdsList();
  ids.ids = { "f1" };
  faces_pub.publish(ids);

  WAIT(400);

  ASSERT_EQ(hri_listener.getPersons().size(), 1);

  match.id1 = "f1";
  match.id1_type = hri_msgs::IdsMatch::FACE;
  match.id2 = "p1";
  match.id2_type = hri_msgs::IdsMatch::PERSON;
  match.confidence = 0.8;

  pub.publish(match);

  WAIT(400);

  persons = hri_listener.getPersons();
  ASSERT_EQ(persons.size(), 1) << "the anonymous 'f1' person should have disappeared, since face 'f1' is now associated to a person";

  ASSERT_TRUE(persons.find("p1") != persons.end());
  ASSERT_TRUE(persons.find(anon_id) == persons.end())
      << "the anonymous 'f1' person should have disappeared, since face 'f1' is now associated to a person";


  ids.ids = { "f1" };
  faces_pub.publish(ids);
  WAIT(400);
  ASSERT_EQ(hri_listener.getPersons().size(), 1);

  ids.ids = { "f1", "f2" };
  faces_pub.publish(ids);
  WAIT(400);
  ASSERT_EQ(hri_listener.getPersons().size(), 2);

  ids.ids = { "b1" };
  bodies_pub.publish(ids);
  WAIT(400);
  ASSERT_EQ(hri_listener.getPersons().size(), 3);

  match.id1 = "f2";
  match.id1_type = hri_msgs::IdsMatch::FACE;
  match.id2 = "b1";
  match.id2_type = hri_msgs::IdsMatch::BODY;
  match.confidence = 0.8;

  pub.publish(match);

  WAIT(400);

  persons = hri_listener.getPersons();

  ASSERT_EQ(persons.size(), 2);

  ASSERT_TRUE(persons.find("p1") != persons.end());
  ASSERT_TRUE(persons.find("p2") != persons.end());


  ids.ids = { "f2" };
  faces_pub.publish(ids);
  WAIT(400);
  ASSERT_EQ(hri_listener.getPersons().size(), 1);

  match.id1 = "b1";
  match.id1_type = hri_msgs::IdsMatch::BODY;
  match.id2 = "p2";
  match.id2_type = hri_msgs::IdsMatch::PERSON;
  match.confidence = 0.8;

  pub.publish(match);

  spinner.stop();
}

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);
  ros::Time::init();  // needed for ros::Time::now()
  ros::init(argc, argv, "test_hri_person_manager");
  ros::NodeHandle nh;
  ROS_INFO("Starting HRI person manager tests");
  return RUN_ALL_TESTS();
}

