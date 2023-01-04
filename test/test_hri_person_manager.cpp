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

#include "hri/hri.h"
#include "hri/base.h"
#include "hri_msgs/IdsMatch.h"
#include "person_matcher.h"
#include "managed_person.h"

using namespace hri;
using namespace std;
using namespace ros;

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
  data = { { "f1", face, "b1", body, 0.0 } };
  model.update(data);
  association = model.get_association("p1");
  EXPECT_EQ(association, (map<FeatureType, ID>{ { hri::face, "f1" }, { hri::body, "b2" } }));
  EXPECT_TRUE(association.find(hri::voice) == association.end());


  // this time, the likelihood of p1 being associated to b2 is < threshold (0.9
  // * 0.4 < 0.4) => no association should be returned.
  data = { { "f1", face, "b2", body, 0.4 } };
  model.update(data);
  association = model.get_association("p1");
  EXPECT_EQ(association, (map<FeatureType, ID>{ { hri::face, "f1" } }));
  EXPECT_TRUE(association.find(hri::body) == association.end());
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

  // publish a face
  auto ids = hri_msgs::IdsList();
  ids.ids = { "f1" };
  faces_pub.publish(ids);

  WAIT(400);

  auto persons = hri_listener.getPersons();
  ASSERT_EQ(persons.size(), 1) << "an anonymous person should have been created";

  auto anon_id = hri::ANONYMOUS + "f1";
  ASSERT_TRUE(persons.find(anon_id) != persons.end())
      << "the anonymous person 'anonymous_person_f1' should be published";

  ASSERT_TRUE(persons[anon_id].lock());
  auto f1 = persons[anon_id].lock();
  ASSERT_TRUE(f1);
  ASSERT_TRUE(f1->anonymous());
  ASSERT_TRUE(f1->face().lock()) << "the anonymous person should be associated to its face";
  ASSERT_EQ(f1->face().lock()->id(), "f1");

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
  ASSERT_EQ(persons.size(), 1) << "the anonymous 'f1' person should have disappeared, since face 'f1' is now associated to a person";

  ASSERT_TRUE(persons.find("p1") != persons.end());
  ASSERT_TRUE(persons.find(anon_id) == persons.end())
      << "the anonymous 'f1' person should have disappeared, since face 'f1' is now associated to a person";

  ////////////////////////////////////////////////////////////////////////////////////////////////////

  match.id1 = "f1";
  match.id1_type = hri_msgs::IdsMatch::FACE;
  match.id2 = "p1";
  match.id2_type = hri_msgs::IdsMatch::PERSON;
  match.confidence = 0.0;

  pub.publish(match);

  WAIT(400);

  ASSERT_EQ(hri_listener.getPersons().size(), 1)
      << "face 'f1' is not associated to an actual person; it should re-create an anonymous person";


  ids.ids = { "f1", "f2" };
  faces_pub.publish(ids);
  WAIT(400);
  ASSERT_EQ(hri_listener.getPersons().size(), 2)
      << "2 anonymous persons are expected, one for each face f1 and f2";

  ids.ids = { "b1" };
  bodies_pub.publish(ids);
  WAIT(400);
  ASSERT_EQ(hri_listener.getPersons().size(), 3)
      << "a 3rd anonymous person should have been created for body b1";

  match.id1 = "f2";
  match.id1_type = hri_msgs::IdsMatch::FACE;
  match.id2 = "b1";
  match.id2_type = hri_msgs::IdsMatch::BODY;
  match.confidence = 0.8;

  pub.publish(match);

  WAIT(400);

  persons = hri_listener.getPersons();

  ASSERT_EQ(persons.size(), 2) << "f2 and b1 are now associated: one of the 2 anonymous persons should have disappeared";

  ids.ids = { "f2" };
  faces_pub.publish(ids);
  WAIT(400);
  ASSERT_EQ(hri_listener.getPersons().size(), 1)
      << "only one anonymous person, associated to f2 and b1 should remain";

  ////////////////////////////////////////////////////////////////////////////////////////////////////

  match.id1 = "b1";
  match.id1_type = hri_msgs::IdsMatch::BODY;
  match.id2 = "p2";
  match.id2_type = hri_msgs::IdsMatch::PERSON;
  match.confidence = 0.8;

  pub.publish(match);

  WAIT(400);
  persons = hri_listener.getPersons();
  ASSERT_EQ(persons.size(), 1) << "only one non-anonymous person p2, associated to f2 and b1 should remain";

  ASSERT_TRUE(persons.find("p1") == persons.end());
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


  // publish (latched) faces before the hri_person_manager is fully started
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

  auto anon_id = hri::ANONYMOUS + "f1";
  ASSERT_TRUE(persons.find(anon_id) != persons.end());

  ids.ids = { "b1" };
  bodies_pub.publish(ids);

  WAIT(400);

  persons = hri_listener.getPersons();
  ASSERT_EQ(persons.size(), 2);

  anon_id = hri::ANONYMOUS + "f1";
  ASSERT_TRUE(persons.find(anon_id) != persons.end());
  anon_id = hri::ANONYMOUS + "b1";
  ASSERT_TRUE(persons.find(anon_id) != persons.end());


  ids.ids = { "f2" };
  faces_pub.publish(ids);

  WAIT(400);

  persons = hri_listener.getPersons();
  ASSERT_EQ(persons.size(), 2);

  anon_id = hri::ANONYMOUS + "f2";
  ASSERT_TRUE(persons.find(anon_id) != persons.end());
  {
    auto p_f2 = hri_listener.getPersons()[anon_id].lock();
    ASSERT_FALSE(p_f2->body().lock());
    ASSERT_TRUE(p_f2->face().lock());
  }

  anon_id = hri::ANONYMOUS + "b1";
  ASSERT_TRUE(persons.find(anon_id) != persons.end());
  {
    auto p_b1 = hri_listener.getPersons()[anon_id].lock();
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

  auto anon_id = hri::ANONYMOUS + "f1";
  ASSERT_TRUE(persons.find(anon_id) != persons.end());

  persons = hri_listener.getTrackedPersons();
  ASSERT_EQ(persons.size(), 1);
  ASSERT_TRUE(persons.find(anon_id) != persons.end());

  ids.ids = { "b1" };
  bodies_pub.publish(ids);

  WAIT(200);

  persons = hri_listener.getPersons();
  ASSERT_EQ(persons.size(), 2);

  anon_id = hri::ANONYMOUS + "b1";
  ASSERT_TRUE(persons.find(anon_id) != persons.end());

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



  ASSERT_TRUE(persons[anon_id].lock());
  auto f1 = persons[anon_id].lock();
  ASSERT_TRUE(f1);
  ASSERT_TRUE(f1->anonymous());
  ASSERT_TRUE(f1->face().lock()) << "the anonymous person should be associated to its face";
  ASSERT_EQ(f1->face().lock()->id(), "f1");

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

