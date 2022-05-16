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
#include <thread>
#include <chrono>

#include <ros/ros.h>

#include "hri/hri.h"
#include "hri/base.h"
#include "hri_msgs/IdsMatch.h"
#include "person_matcher.h"

using namespace hri;
using namespace std;
using namespace ros;

// waiting time for the libhri callback to process their inputs
#define WAIT(X) std::this_thread::sleep_for(std::chrono::milliseconds(X))

const string ANONYMOUS("anonymous");

TEST(hri_person_matcher, BasicAssociationModel)
{
  auto model = PersonMatcher(0.4);

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


  model = PersonMatcher(0.05);

  model.update({ { "p1", person, "f1", face, 0.1 } });
  association = model.get_association("p1");
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

TEST(hri_person_matcher, RemoveAddIds)
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
}


TEST(hri_person_manager, ROSNode)
{
  NodeHandle nh;

  ros::AsyncSpinner spinner(1);
  spinner.start();

  HRIListener hri_listener;

  Publisher pub = nh.advertise<hri_msgs::IdsMatch>("/humans/candidate_matches", 1);

  // wait for the hri_person_manager node to be up
  WAIT(500);

  ASSERT_EQ(pub.getNumSubscribers(), 1)
      << "hri_person_manager should be the one and only node subscribed to this topic";

  ASSERT_EQ(hri_listener.getPersons().size(), 0);


  hri_msgs::IdsMatch match;
  match.face_id = "f1";
  match.person_id = "p1";
  match.confidence = 0.7;

  pub.publish(match);

  WAIT(100);

  ASSERT_EQ(hri_listener.getPersons().size(), 1);

  auto persons = hri_listener.getPersons();
  ASSERT_TRUE(persons.find("p1") != persons.end());

  ASSERT_EQ(persons["p1"]->face().lock(), nullptr)
      << "the face has not yet been published -> can not be associated to the person yet.";

  ASSERT_EQ(persons["p1"]->body().lock(), nullptr);
  ASSERT_EQ(persons["p1"]->voice().lock(), nullptr);


  // publish the face
  auto face_pub = nh.advertise<hri_msgs::IdsList>("/humans/faces/tracked", 1);
  auto ids = hri_msgs::IdsList();
  ids.ids = { "f1" };
  face_pub.publish(ids);

  // wait for lihri to pick up the new face
  WAIT(200);

  ASSERT_NE(persons["p1"]->face().lock(), nullptr)
      << "the face has been published -> should now be associated to the person.";


  ASSERT_FALSE(persons["p1"]->face().expired());
  ASSERT_EQ(persons["p1"]->face().lock()->id(), "f1");

  ids.ids = { "f1", "f2" };
  face_pub.publish(ids);

  WAIT(200);

  auto face_f1 = persons["p1"]->face();

  ASSERT_FALSE(face_f1.expired());
  ASSERT_EQ(face_f1.lock()->id(), "f1");

  match.face_id = "f2";
  match.person_id = "p1";
  match.confidence = 0.8;

  pub.publish(match);

  WAIT(100);

  auto face_f2 = persons["p1"]->face();

  ASSERT_FALSE(face_f1.expired()) << "face 'f1' still exists";
  ASSERT_FALSE(face_f2.expired()) << "face 'f2' should now be the mostly likely face of 'p1'";
  ASSERT_EQ(face_f2.lock()->id(), "f2");

  spinner.stop();
}

TEST(hri_person_manager, AnonymousPersons)
{
  NodeHandle nh;

  ros::AsyncSpinner spinner(1);
  spinner.start();

  HRIListener hri_listener;

  Publisher pub = nh.advertise<hri_msgs::IdsMatch>("/humans/candidate_matches", 1);

  // wait for the hri_person_manager node to be up
  WAIT(500);

  hri_msgs::IdsMatch match;
  match.face_id = "f1";

  pub.publish(match);

  WAIT(100);

  ASSERT_EQ(hri_listener.getPersons().size(), 1);

  auto persons = hri_listener.getPersons();
  ASSERT_TRUE(persons.find("f1") != persons.end());

  ASSERT_TRUE(persons["f1"]->anonymous());

  match.face_id = "f1";
  match.person_id = "p1";
  match.confidence = 0.8;

  pub.publish(match);

  WAIT(100);

  ASSERT_EQ(hri_listener.getPersons().size(), 1);

  persons = hri_listener.getPersons();
  ASSERT_TRUE(persons.find("p1") != persons.end())
      << "the anonymous 'f1' person should have disappeared, since face 'f1' is now associated to a person";

  spinner.stop();
}

int main(int argc, char **argv)
{
  testing::InitGoogleTest(&argc, argv);
  ros::Time::init();  // needed for ros::Time::now()
  ros::init(argc, argv, "test_hri_person_manager");
  ros::NodeHandle nh;
  ROS_INFO("Starting HRI person manager tests");
  return RUN_ALL_TESTS();
}

