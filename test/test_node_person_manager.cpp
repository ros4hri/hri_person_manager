// Copyright (c) 2024 PAL Robotics S.L. All rights reserved.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.


#include <chrono>
#include <future>
#include <memory>
#include <string>

#include "gtest/gtest.h"
#include "hri/hri.hpp"
#include "hri/types.hpp"
#include "hri_msgs/msg/ids_match.hpp"
#include "hri_msgs/msg/ids_list.hpp"
#include "lifecycle_msgs/srv/change_state.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rosgraph_msgs/msg/clock.hpp"
#include "std_srvs/srv/empty.hpp"

#include "hri_person_manager/managed_person.hpp"
#include "hri_person_manager/node_person_manager.hpp"

class NodePersonManagerTest : public ::testing::Test
{
protected:
  static void SetUpTestCase()
  {
    rclcpp::init(0, nullptr);
  }

  static void TearDownTestCase()
  {
    rclcpp::shutdown();
  }

  void SetUp() override
  {
    person_manager_node_ = std::make_shared<hri_person_manager::NodePersonManager>();
    person_manager_node_->set_parameter({"use_sim_time", true});
    person_manager_executor_ = rclcpp::executors::SingleThreadedExecutor::make_shared();
    person_manager_executor_->add_node(person_manager_node_->get_node_base_interface());

    tester_node_ = rclcpp::Node::make_shared("tester_node");
    tester_executor_ = rclcpp::executors::SingleThreadedExecutor::make_shared();
    tester_executor_->add_node(tester_node_);
    hri_listener_ = hri::HRIListener::create(tester_node_);

    time_ = tester_node_->get_clock()->now();
    clock_pub_ = tester_node_->create_publisher<rosgraph_msgs::msg::Clock>("/clock", 1);

    matches_pub_ = tester_node_->create_publisher<hri_msgs::msg::IdsMatch>(
      "/humans/candidate_matches", 10);
    faces_pub_ = tester_node_->create_publisher<hri_msgs::msg::IdsList>(
      "/humans/faces/tracked", 10);
    bodies_pub_ = tester_node_->create_publisher<hri_msgs::msg::IdsList>(
      "/humans/bodies/tracked", 10);
    voices_pub_ = tester_node_->create_publisher<hri_msgs::msg::IdsList>(
      "/humans/voices/tracked", 10);
    reset_srv_ = person_manager_node_->create_client<std_srvs::srv::Empty>(
      "/hri_person_manager/reset");

    auto client_change_state = tester_node_->create_client<lifecycle_msgs::srv::ChangeState>(
      "/hri_person_manager/change_state");
    auto request = std::make_shared<lifecycle_msgs::srv::ChangeState::Request>();

    request->transition.id = lifecycle_msgs::msg::Transition::TRANSITION_CONFIGURE;
    auto future_result = client_change_state->async_send_request(request);
    spin();
    EXPECT_TRUE(future_result.get()->success);

    request->transition.id = lifecycle_msgs::msg::Transition::TRANSITION_ACTIVATE;
    future_result = client_change_state->async_send_request(request);
    spin();
    EXPECT_TRUE(future_result.get()->success);
  }

  void TearDown() override
  {
    clock_pub_.reset();
    matches_pub_.reset();
    faces_pub_.reset();
    bodies_pub_.reset();
    voices_pub_.reset();

    hri_listener_.reset();
    person_manager_node_.reset();
    person_manager_executor_.reset();
    tester_node_.reset();
    tester_executor_.reset();
  }

  void spin(std::chrono::nanoseconds timeout = std::chrono::seconds(10))
  {
    person_manager_executor_->spin_all(timeout);

    time_ += rclcpp::Duration(std::chrono::milliseconds(200));
    rosgraph_msgs::msg::Clock clock_msg;
    clock_msg.clock = time_;
    clock_pub_->publish(clock_msg);
    person_manager_executor_->spin_all(timeout);

    tester_executor_->spin_all(timeout);
  }

  rclcpp::Node::SharedPtr tester_node_;
  std::shared_ptr<hri::HRIListener> hri_listener_;
  rclcpp::Publisher<hri_msgs::msg::IdsMatch>::SharedPtr matches_pub_;
  rclcpp::Publisher<hri_msgs::msg::IdsList>::SharedPtr faces_pub_;
  rclcpp::Publisher<hri_msgs::msg::IdsList>::SharedPtr bodies_pub_;
  rclcpp::Publisher<hri_msgs::msg::IdsList>::SharedPtr voices_pub_;
  rclcpp::Client<std_srvs::srv::Empty>::SharedPtr reset_srv_;
  rclcpp::Executor::SharedPtr person_manager_executor_;
  rclcpp::Executor::SharedPtr tester_executor_;

private:
  rclcpp::Time time_;
  rclcpp::Publisher<rosgraph_msgs::msg::Clock>::SharedPtr clock_pub_;
  std::shared_ptr<hri_person_manager::NodePersonManager> person_manager_node_;
};

TEST_F(NodePersonManagerTest, KnownPersons)
{
  hri_msgs::msg::IdsMatch match;
  hri_msgs::msg::IdsList ids;

  match.id1 = "f1";
  match.id1_type = hri_msgs::msg::IdsMatch::FACE;
  match.id2 = "p1";
  match.id2_type = hri_msgs::msg::IdsMatch::PERSON;
  match.confidence = 0.7;
  matches_pub_->publish(match);
  spin();
  auto persons = hri_listener_->getPersons();
  EXPECT_EQ(hri_listener_->getPersons().size(), 1U);
  ASSERT_TRUE(persons.count("p1"));
  auto p1 = persons["p1"];
  EXPECT_FALSE(p1->face())
    << "The face has not yet been published -> can not be associated to the person yet.";
  EXPECT_FALSE(p1->body());
  EXPECT_FALSE(p1->voice());

  ids.ids = {"f1"};
  faces_pub_->publish(ids);
  spin();
  ASSERT_TRUE(p1->face())
    << "The face has been published -> should now be associated to the person.";
  EXPECT_EQ(p1->face()->id(), "f1");

  ids.ids = {"f1", "f2"};
  faces_pub_->publish(ids);
  spin();
  EXPECT_EQ(p1->face()->id(), "f1");

  auto face_f1 = p1->face();
  match.id1 = "f2";
  match.id1_type = hri_msgs::msg::IdsMatch::FACE;
  match.id2 = "p1";
  match.id2_type = hri_msgs::msg::IdsMatch::PERSON;
  match.confidence = 0.8;
  matches_pub_->publish(match);
  spin();
  auto face_f2 = p1->face();
  EXPECT_TRUE(face_f1->valid()) << "Face 'f1' still exists";
  ASSERT_TRUE(face_f2) << "Face 'f2' should now be the mostly likely face of 'p1'";
  EXPECT_EQ(face_f2->id(), "f2");
}

TEST_F(NodePersonManagerTest, Reset)
{
  auto empty = std::make_shared<std_srvs::srv::Empty::Request>();
  hri_msgs::msg::IdsMatch match;
  hri_msgs::msg::IdsList ids;

  reset_srv_->async_send_request(empty);
  spin();

  ids.ids = {"f1"};
  faces_pub_->publish(ids);
  spin();
  match.id1 = "f2";
  match.id1_type = hri_msgs::msg::IdsMatch::FACE;
  match.id2 = "p2";
  match.id2_type = hri_msgs::msg::IdsMatch::PERSON;
  match.confidence = 0.7;
  matches_pub_->publish(match);
  spin();
  match.id1 = "f1";
  match.id1_type = hri_msgs::msg::IdsMatch::FACE;
  match.id2 = "p1";
  match.id2_type = hri_msgs::msg::IdsMatch::PERSON;
  match.confidence = 0.7;
  matches_pub_->publish(match);
  spin();
  auto persons = hri_listener_->getPersons();
  EXPECT_EQ(hri_listener_->getTrackedPersons().size(), 2U);
  EXPECT_EQ(persons.size(), 2U);
  ASSERT_TRUE(persons.count("p1"));
  EXPECT_TRUE(persons["p1"]);

  reset_srv_->async_send_request(empty);
  spin();
  EXPECT_EQ(hri_listener_->getPersons().size(), 0U);
  EXPECT_FALSE(persons["p1"]->valid()) << "The old pointer should now be invalid";

  matches_pub_->publish(match);
  spin();
  persons = hri_listener_->getPersons();
  EXPECT_EQ(persons.size(), 1U);
  ASSERT_TRUE(persons.count("p1"));
  EXPECT_TRUE(persons["p1"]);
}

TEST_F(NodePersonManagerTest, AnonymousPersons)
{
  auto empty = std::make_shared<std_srvs::srv::Empty::Request>();
  hri_msgs::msg::IdsMatch match;
  hri_msgs::msg::IdsList ids;

  EXPECT_EQ(hri_listener_->getPersons().size(), 0U) << "No one should be there yet";

  ids.ids = {"f1"};
  faces_pub_->publish(ids);
  spin();
  auto persons = hri_listener_->getPersons();
  ASSERT_EQ(persons.size(), 1U) << "Exactly one anonymous person should have been created";
  auto anon_id = persons.begin()->first;
  EXPECT_TRUE(anon_id.rfind(hri_person_manager::kAnonymous, 0U) == 0U)
    << "The anonymous person ID should start with " << hri_person_manager::kAnonymous
    << ". Got: " << anon_id;
  auto p1 = persons[anon_id];
  ASSERT_TRUE(p1);
  EXPECT_TRUE(p1->anonymous());
  ASSERT_TRUE(p1->face()) << "The anonymous person should be associated to its face";
  EXPECT_EQ(p1->face()->id(), "f1");

  ids.ids = {};
  faces_pub_->publish(ids);
  spin();
  EXPECT_EQ(hri_listener_->getPersons().size(), 0U)
    << "The anonymous person should have disappeared since its face is not detected anymore";

  // anon1 -> f1
  ids.ids = {"f1"};
  faces_pub_->publish(ids);
  spin();
  ASSERT_EQ(persons.size(), 1U);
  anon_id = persons.begin()->first;

  // p1 -> f1
  match.id1 = "f1";
  match.id1_type = hri_msgs::msg::IdsMatch::FACE;
  match.id2 = "p1";
  match.id2_type = hri_msgs::msg::IdsMatch::PERSON;
  match.confidence = 0.8;
  matches_pub_->publish(match);
  spin();
  persons = hri_listener_->getPersons();
  ASSERT_EQ(persons.size(), 1U);
  EXPECT_FALSE(persons.count(anon_id))
    << "The anonymous person associated to 'f1' should have disappeared";
  EXPECT_TRUE(persons.count("p1"))
    << "Face 'f1' should be associated to the non-anonymous person 'p1'";

  // p1; anon2 -> f1
  match.id1 = "f1";
  match.id1_type = hri_msgs::msg::IdsMatch::FACE;
  match.id2 = "p1";
  match.id2_type = hri_msgs::msg::IdsMatch::PERSON;
  match.confidence = 0.0;
  matches_pub_->publish(match);
  spin();
  EXPECT_EQ(hri_listener_->getPersons().size(), 2U)
    << "Face 'f1' is not associated anymore to an actual person; it should re-create an "
    << "anonymous person";

  // p1; anon2 -> f1; anon3 -> f2
  ids.ids = {"f1", "f2"};
  faces_pub_->publish(ids);
  spin();
  EXPECT_EQ(hri_listener_->getPersons().size(), 3U)
    << "Two anonymous persons are expected, one for each face 'f1' and 'f2'";

  // p1; anon2 -> f1; anon3 -> f2; anon4 -> b1
  ids.ids = {"b1"};
  bodies_pub_->publish(ids);
  spin();
  EXPECT_EQ(hri_listener_->getPersons().size(), 4U)
    << "A 3rd anonymous person should have been created for body 'b1'";

  // p1; anon2 -> f1; anon3 -> f2,b1
  match.id1 = "f2";
  match.id1_type = hri_msgs::msg::IdsMatch::FACE;
  match.id2 = "b1";
  match.id2_type = hri_msgs::msg::IdsMatch::BODY;
  match.confidence = 0.8;
  matches_pub_->publish(match);
  spin();
  EXPECT_EQ(hri_listener_->getPersons().size(), 3U)
    << "Now 'f2' and 'b1' are associated: one of the 2 anonymous persons should have disappeared";

  // p1; anon3 -> f2,b1
  ids.ids = {"f2"};
  faces_pub_->publish(ids);
  spin();
  EXPECT_EQ(hri_listener_->getPersons().size(), 2U)
    << "The face 'f1' disappears with its associated anonymous person";

  // p1; p2 -> f2,b1
  match.id1 = "b1";
  match.id1_type = hri_msgs::msg::IdsMatch::BODY;
  match.id2 = "p2";
  match.id2_type = hri_msgs::msg::IdsMatch::PERSON;
  match.confidence = 0.8;
  matches_pub_->publish(match);
  spin();
  persons = hri_listener_->getPersons();
  EXPECT_EQ(persons.size(), 2U)
    << "The anonymous person associated to 'f2' and 'b1' should be replaced by 'p2'";
  EXPECT_TRUE(persons.count("p1"));
  EXPECT_TRUE(persons.count("p2"));

  // reset
  reset_srv_->async_send_request(empty);
  spin();

  // anon1 -> f1
  ids.ids = {"f1"};
  faces_pub_->publish(ids);
  spin();
  persons = hri_listener_->getPersons();
  ASSERT_EQ(persons.size(), 1U);
  anon_id = persons.begin()->first;
  auto p_f1 = persons[anon_id];
  EXPECT_TRUE(p_f1->face());
  EXPECT_FALSE(p_f1->body());

  // anon2 -> b1
  ids.ids = {};
  faces_pub_->publish(ids);
  ids.ids = {"b1"};
  bodies_pub_->publish(ids);
  spin();
  persons = hri_listener_->getPersons();
  ASSERT_EQ(persons.size(), 1U);
  auto anon_id_2 = persons.begin()->first;
  EXPECT_NE(anon_id, anon_id_2) << "Two different anonymous persons should have been created";
  auto p_b1 = persons[anon_id_2];
  EXPECT_TRUE(p_b1->body());
  EXPECT_FALSE(p_b1->face());

  // reset
  reset_srv_->async_send_request(empty);
  spin();

  // anon1 -> f1
  ids.ids = {"f1"};
  faces_pub_->publish(ids);
  spin();
  persons = hri_listener_->getPersons();
  EXPECT_EQ(hri_listener_->getTrackedPersons().size(), 1U);
  ASSERT_EQ(persons.size(), 1U);
  anon_id = persons.begin()->first;
  EXPECT_TRUE(persons.count(anon_id));

  // anon1 -> f1; anon2 -> b1
  ids.ids = {"b1"};
  bodies_pub_->publish(ids);
  spin();
  EXPECT_EQ(hri_listener_->getPersons().size(), 2U);
  EXPECT_EQ(hri_listener_->getTrackedPersons().size(), 2U);

  // anon1 -> f1,b1
  match.id1 = "f1";
  match.id1_type = hri_msgs::msg::IdsMatch::FACE;
  match.id2 = "b1";
  match.id2_type = hri_msgs::msg::IdsMatch::BODY;
  match.confidence = 1.0;
  matches_pub_->publish(match);
  spin();
  persons = hri_listener_->getPersons();
  EXPECT_EQ(persons.size(), 1U) << "The two anonymous persons should have merged";
  anon_id = persons.begin()->first;
  auto p_anon = persons[anon_id];
  ASSERT_TRUE(p_anon);
  EXPECT_TRUE(p_anon->anonymous());
  ASSERT_TRUE(p_anon->face()) << "The anonymous person should be associated to the face";
  EXPECT_EQ(p_anon->face()->id(), "f1");
  ASSERT_TRUE(p_anon->body()) << "The anonymous person should be associated to the body";
  EXPECT_EQ(p_anon->body()->id(), "b1");

  // anon1 -> b1
  ids.ids = {};
  faces_pub_->publish(ids);
  spin();
  EXPECT_EQ(hri_listener_->getPersons().size(), 1U)
    << "The anonymous person should not have disappeared after its face is not detected anymore "
    << "since it is associated with the body";

  // anon1 -> b1; anon2 -> f1
  ids.ids = {"f1"};
  faces_pub_->publish(ids);
  spin();
  EXPECT_EQ(hri_listener_->getPersons().size(), 2U)
    << "The face 'f1' should be associated again with the body 'b1' anonymous person";

  // anon1 -> b1; p1 -> f1
  match.id1 = "f1";
  match.id1_type = hri_msgs::msg::IdsMatch::FACE;
  match.id2 = "p1";
  match.id2_type = hri_msgs::msg::IdsMatch::PERSON;
  matches_pub_->publish(match);
  spin();
  persons = hri_listener_->getPersons();
  EXPECT_EQ(persons.size(), 2U) << "The anonymous person should have been replaced by 'p1'";
  EXPECT_TRUE(persons.count(anon_id));
  EXPECT_TRUE(persons.count("p1"));

  // anon1 -> b1; p1 -> f1
  ids.ids = {"f1"};
  faces_pub_->publish(ids);
  spin();
  EXPECT_EQ(hri_listener_->getPersons().size(), 2U);

  // anon1 -> b1; p1 -> f1; anon2 -> f2
  ids.ids = {"f1", "f2"};
  faces_pub_->publish(ids);
  spin();
  EXPECT_EQ(hri_listener_->getPersons().size(), 3U);

  // anon1 -> b1; p1 -> f1; anon2 -> f2; anon3 -> b2
  ids.ids = {"b1", "b2"};
  bodies_pub_->publish(ids);
  spin();
  EXPECT_EQ(hri_listener_->getPersons().size(), 4U);

  // anon1 -> b1; p1 -> f1; anon2 -> f2,b2
  match.id1 = "f2";
  match.id1_type = hri_msgs::msg::IdsMatch::FACE;
  match.id2 = "b2";
  match.id2_type = hri_msgs::msg::IdsMatch::BODY;
  match.confidence = 0.8;
  matches_pub_->publish(match);
  spin();
  persons = hri_listener_->getPersons();
  EXPECT_EQ(persons.size(), 3U);
  EXPECT_TRUE(persons.count("p1"));
  EXPECT_FALSE(persons.count("p2"));

  // anon1 -> b1; p1 -> f1; p2 -> f2,b2
  match.id1 = "b2";
  match.id1_type = hri_msgs::msg::IdsMatch::BODY;
  match.id2 = "p2";
  match.id2_type = hri_msgs::msg::IdsMatch::PERSON;
  match.confidence = 0.8;
  matches_pub_->publish(match);
  spin();
  persons = hri_listener_->getPersons();
  EXPECT_EQ(persons.size(), 3U);
  EXPECT_TRUE(persons.count("p1"));
  EXPECT_TRUE(persons.count("p2"));
}

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
