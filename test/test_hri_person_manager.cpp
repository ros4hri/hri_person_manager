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

#include "hri/base.h"
#include "person_matcher.h"

using namespace hri;
using namespace std;

// waiting time for the libhri callback to process their inputs
#define WAIT std::this_thread::sleep_for(std::chrono::milliseconds(5))

TEST(hri_person_matcher, BasicAssociationModel)
{
  auto model = PersonMatcher(0.4);

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
                     { "f1", face, "b2", body, 0.4 },
                     { "p1", person, "f1", face, 0.9 } };
  model.update(data);
  auto association = model.get_association("p1");
  EXPECT_EQ(association, (map<FeatureType, ID>{ { hri::face, "f1" }, { hri::body, "b1" } }));
  EXPECT_TRUE(association.find(hri::voice) == association.end());

  // Test *updating* an edge. The previous value should be removed
  data = { { "f1", face, "b1", body, 0.0 },
           { "f1", face, "b2", body, 0.4 },
           { "p1", person, "f1", face, 0.9 } };
  model.update(data);
  association = model.get_association("p1");
  EXPECT_EQ(association, (map<FeatureType, ID>{ { hri::face, "f1" }, { hri::body, "b2" } }));
  EXPECT_TRUE(association.find(hri::voice) == association.end());
}

TEST(hri_person_matcher, RemoveAddIds)
{
  auto model = PersonMatcher(0.4);


  Relations data = { { "f1", face, "b1", body, 0.7 },
                     { "f1", face, "b2", body, 0.4 },
                     { "p1", person, "f1", face, 0.9 } };
  model.update(data);
  auto association = model.get_association("p1");
  EXPECT_EQ(association, (map<FeatureType, ID>{ { hri::face, "f1" }, { hri::body, "b1" } }));
  EXPECT_TRUE(association.find(hri::voice) == association.end());

  model.erase("b1");
  association = model.get_association("p1");
  EXPECT_EQ(association, (map<FeatureType, ID>{ { hri::face, "f1" }, { hri::body, "b2" } }));
  EXPECT_TRUE(association.find(hri::voice) == association.end());

  model.update({ "f1", face, "b1", body, 0.7 });
  association = model.get_association("p1");
  EXPECT_EQ(association, (map<FeatureType, ID>{ { hri::face, "f1" }, { hri::body, "b1" } }));
  EXPECT_TRUE(association.find(hri::voice) == association.end());
}

int main(int argc, char **argv)
{
  testing::InitGoogleTest(&argc, argv);
  ros::Time::init();  // needed for ros::Time::now()
  ros::init(argc, argv, "test_hri_person_matcher");
  ros::NodeHandle nh;
  ROS_INFO("Starting HRI person matcher tests");
  return RUN_ALL_TESTS();
}

