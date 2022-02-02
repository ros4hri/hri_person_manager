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
#include <hri/hri.h>
#include <ros/ros.h>
#include <thread>
#include <chrono>
#include <memory>
#include "hri_msgs/IdsList.h"
#include "sensor_msgs/RegionOfInterest.h"

using namespace std;
using namespace ros;
using namespace hri;

// waiting time for the libhri callback to process their inputs
#define WAIT std::this_thread::sleep_for(std::chrono::milliseconds(5))

TEST(libhri, GetFaces)
{
  NodeHandle nh;

  ros::AsyncSpinner spinner(1);
  spinner.start();

  Publisher pub;

  {
    HRIListener hri_listener;

    pub = nh.advertise<hri_msgs::IdsList>("/humans/faces/tracked", 1);

    EXPECT_EQ(pub.getNumSubscribers(), 1U);

    EXPECT_EQ(hri_listener.getFaces().size(), 0);

    auto ids = hri_msgs::IdsList();

    ROS_INFO("[A]");
    ids.ids = { "A" };
    pub.publish(ids);
    WAIT;
    auto faces = hri_listener.getFaces();
    EXPECT_EQ(faces.size(), 1U);
    ASSERT_TRUE(faces.find("A") != faces.end());
    EXPECT_TRUE(faces["A"].lock()->id() == "A");

    ROS_INFO("[A]");
    pub.publish(ids);
    WAIT;
    EXPECT_EQ(hri_listener.getFaces().size(), 1U);

    ROS_INFO("[A,B]");
    ids.ids = { "A", "B" };
    pub.publish(ids);
    WAIT;
    faces = hri_listener.getFaces();
    EXPECT_EQ(faces.size(), 2U);
    EXPECT_TRUE(faces.find("A") != faces.end());
    EXPECT_TRUE(faces.find("B") != faces.end());

    ROS_INFO("[A,B]");
    pub.publish(ids);
    WAIT;
    EXPECT_EQ(hri_listener.getFaces().size(), 2U);

    ROS_INFO("[B]");
    ids.ids = { "B" };
    pub.publish(ids);
    WAIT;
    faces = hri_listener.getFaces();
    EXPECT_EQ(faces.size(), 1U);
    EXPECT_TRUE(faces.find("A") == faces.end());
    ASSERT_TRUE(faces.find("B") != faces.end());

    weak_ptr<Face> face_b = faces["B"];
    EXPECT_FALSE(face_b.expired());  // face B exists!

    ROS_INFO("[]");
    ids.ids = {};
    pub.publish(ids);
    WAIT;
    EXPECT_EQ(hri_listener.getFaces().size(), 0U);

    EXPECT_TRUE(face_b.expired());  // face B does not exist anymore!
  }

  EXPECT_EQ(pub.getNumSubscribers(), 0);
  spinner.stop();
}

TEST(libhri, GetFacesRoi)
{
  NodeHandle nh;

  ros::AsyncSpinner spinner(1);
  spinner.start();

  HRIListener hri_listener;

  auto pub = nh.advertise<hri_msgs::IdsList>("/humans/faces/tracked", 1);

  auto pub_r1 = nh.advertise<sensor_msgs::RegionOfInterest>("/humans/faces/A/roi", 1, true);  // /roi topic is latched
  auto pub_r2 = nh.advertise<sensor_msgs::RegionOfInterest>("/humans/faces/B/roi", 1, true);  // /roi topic is latched

  auto ids = hri_msgs::IdsList();

  ids.ids = { "A" };
  pub.publish(ids);
  WAIT;
  EXPECT_EQ(pub_r1.getNumSubscribers(), 1U)
      << "Face A should have subscribed to /humans/faces/A/roi";


  ids.ids = { "B" };
  pub.publish(ids);
  WAIT;
  EXPECT_EQ(pub_r1.getNumSubscribers(), 0U)
      << "Face A is deleted. No one should be subscribed to /humans/faces/A/roi anymore";
  EXPECT_EQ(pub_r2.getNumSubscribers(), 1U)
      << "Face B should have subscribed to /humans/faces/B/roi";


  auto faces = hri_listener.getFaces();
  auto face = faces["B"].lock();

  EXPECT_EQ(face->ns(), "/humans/faces/B");

  EXPECT_EQ(face->roi().width, 0);

  auto roi = sensor_msgs::RegionOfInterest();

  roi.width = 10;
  pub_r2.publish(roi);
  WAIT;
  EXPECT_EQ(face->roi().width, 10);

  roi.width = 20;
  pub_r2.publish(roi);
  WAIT;
  EXPECT_EQ(face->roi().width, 20);

  // RoI of face A published *before* face A is published in /faces/tracked,
  // but should still get its RoI, as /roi is latched.
  pub_r1.publish(roi);
  ids.ids = { "B", "A" };
  pub.publish(ids);
  WAIT;

  faces = hri_listener.getFaces();
  auto face_a = faces["A"].lock();
  auto face_b = faces["B"].lock();

  EXPECT_EQ(face_a->ns(), "/humans/faces/A");
  EXPECT_EQ(face_a->roi().width, 20);
  EXPECT_EQ(face_b->ns(), "/humans/faces/B");
  EXPECT_EQ(face_b->roi().width, 20);

  spinner.stop();
}

TEST(libhri, GetBodies)
{
  NodeHandle nh;

  ros::AsyncSpinner spinner(1);
  spinner.start();

  Publisher pub;

  {
    HRIListener hri_listener;

    pub = nh.advertise<hri_msgs::IdsList>("/humans/bodies/tracked", 1);

    EXPECT_EQ(pub.getNumSubscribers(), 1U);


    auto ids = hri_msgs::IdsList();

    ROS_INFO("[A]");
    ids.ids = { "A" };
    pub.publish(ids);
    WAIT;
    auto bodies = hri_listener.getBodies();
    EXPECT_EQ(bodies.size(), 1U);
    ASSERT_TRUE(bodies.find("A") != bodies.end());
    EXPECT_TRUE(bodies["A"].lock()->id() == "A");

    ROS_INFO("[A]");
    pub.publish(ids);
    WAIT;
    EXPECT_EQ(hri_listener.getBodies().size(), 1U);

    ROS_INFO("[A,B]");
    ids.ids = { "A", "B" };
    pub.publish(ids);
    WAIT;
    bodies = hri_listener.getBodies();
    EXPECT_EQ(bodies.size(), 2U);
    EXPECT_TRUE(bodies.find("A") != bodies.end());
    EXPECT_TRUE(bodies.find("B") != bodies.end());

    ROS_INFO("[A,B]");
    pub.publish(ids);
    WAIT;
    EXPECT_EQ(hri_listener.getBodies().size(), 2U);

    ROS_INFO("[B]");
    ids.ids = { "B" };
    pub.publish(ids);
    WAIT;
    bodies = hri_listener.getBodies();
    EXPECT_EQ(bodies.size(), 1U);
    EXPECT_TRUE(bodies.find("A") == bodies.end());
    ASSERT_TRUE(bodies.find("B") != bodies.end());

    weak_ptr<Body> body_b = bodies["B"];
    EXPECT_FALSE(body_b.expired());  // body B exists!

    ROS_INFO("[]");
    ids.ids = {};
    pub.publish(ids);
    WAIT;
    EXPECT_EQ(hri_listener.getBodies().size(), 0U);

    EXPECT_TRUE(body_b.expired());  // body B does not exist anymore!
  }

  EXPECT_EQ(pub.getNumSubscribers(), 0);
  spinner.stop();
}

int main(int argc, char **argv)
{
  testing::InitGoogleTest(&argc, argv);
  ros::Time::init();  // needed for ros::Time::now()
  ros::init(argc, argv, "test_hri");
  ros::NodeHandle nh;
  ROS_INFO("Starting HRI tests");
  return RUN_ALL_TESTS();
}

