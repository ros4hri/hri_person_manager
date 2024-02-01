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

#include <algorithm>
#include <cctype>
#include <fstream>
#include <iostream>
#include <map>
#include <regex>
#include <stdexcept>
#include <sstream>
#include <string>
#include <vector>

#include "boost/graph/connected_components.hpp"
#include "boost/graph/copy.hpp"
#include "boost/graph/named_function_params.hpp"
#include "boost/graph/properties.hpp"
#include "boost/graph/subgraph.hpp"
#include "boost/graph/graph_utility.hpp"
#include "boost/graph/vf2_sub_graph_iso.hpp"
#include "boost/range/iterator_range_core.hpp"
#include "hri/types.hpp"
#include "gtest/gtest.h"

#include "hri_person_manager/managed_person.hpp"
#include "hri_person_manager/person_matcher.hpp"

const char kInput[]{"INPUT"};
const char kGraph[]{"GRAPH"};
const char kOutput[]{"OUTPUT"};

// trim from start (in place)
static inline void ltrim(std::string & s)
{
  s.erase(
    s.begin(), std::find_if(s.begin(), s.end(), [](unsigned char ch) {return !std::isspace(ch);}));
}

// trim from end (in place)
static inline void rtrim(std::string & s)
{
  s.erase(
    std::find_if(s.rbegin(), s.rend(), [](unsigned char ch) {return !std::isspace(ch);}).base(),
    s.end());
}

// trim from both ends (in place)
static inline void trim(std::string & s)
{
  rtrim(s);
  ltrim(s);
}

std::vector<std::string> preparse(const std::vector<std::string> & lines)
{
  std::vector<std::string> res;

  for (const auto & l : lines) {
    std::string s(l);
    trim(s);
    if (
      s.size() > 0 &&
      !((s.rfind("%%", 0) == 0) || (s.rfind("```", 0) == 0) || (s.rfind("graph", 0) == 0)))
    {
      res.push_back(l);
    }
  }
  return res;
}

hri::FeatureType get_type(const std::string & name)
{
  switch (name[0]) {
    case 'p':
    case 'a':
      return hri::FeatureType::kPerson;
    case 'b':
      return hri::FeatureType::kBody;
    case 'v':
      return hri::FeatureType::kVoice;
    case 'f':
      return hri::FeatureType::kFace;
  }

  throw std::runtime_error(std::string("unknown node type: ") + name);
}

void input_parser(
  const std::vector<std::string> & raw_lines, hri_person_manager::PersonMatcher & pm)
{
  auto lines = preparse(raw_lines);

  std::regex edge_regex("(\\w+),(\\w+),([\\d.]+)");
  std::regex remove_node_regex("-(\\w+)");
  std::regex add_node_regex("(\\w+)");

  std::smatch result;

  for (const auto & line : lines) {
    if (std::regex_search(line, result, edge_regex) && result.size() > 1) {
      auto a = result.str(1);
      auto b = result.str(2);
      auto likelihood = stof(result.str(3));

      pm.update({{a, get_type(a), b, get_type(b), likelihood}});
    } else if (std::regex_search(line, result, remove_node_regex) && result.size() > 1) {
      auto a = result.str(1);
      pm.erase(a);
    } else if (std::regex_search(line, result, add_node_regex) && result.size() > 1) {
      auto a = result.str(1);
      pm.update({{a, get_type(a), a, get_type(a), 1.}});
    } else {
      throw std::runtime_error(std::string("malformed input: ") + line);
    }
  }
}

void mermaid_parser(const std::vector<std::string> & raw_lines, hri_person_manager::Graph & graph)
{
  auto lines = preparse(raw_lines);

  std::regex edge_regex("(\\w+) ---\\|([\\d.]+)\\| (\\w+)");
  std::regex add_node_regex("(\\w+)(.*)");

  std::smatch result;

  for (const auto & line : lines) {
    if (std::regex_search(line, result, edge_regex) && result.size() > 1) {
      auto a = result.str(1);
      auto b = result.str(3);
      auto likelihood = stof(result.str(2));

      auto node_a = hri_person_manager::get_node_by_name(a, graph);
      if (node_a == hri_person_manager::kInexistantNode) {
        node_a = boost::add_vertex(graph);
        graph[node_a].name = a;
        graph[node_a].type = get_type(a);
        if (a.rfind("anon", 0) == 0) {
          graph[node_a].anonymous = true;
        }
      }

      auto node_b = hri_person_manager::get_node_by_name(b, graph);
      if (node_b == hri_person_manager::kInexistantNode) {
        node_b = boost::add_vertex(graph);
        graph[node_b].name = b;
        graph[node_b].type = get_type(b);
        if (b.rfind("anon", 0) == 0) {
          graph[node_b].anonymous = true;
        }
      }

      auto e = boost::add_edge(node_a, node_b, graph).first;
      graph[e].likelihood = likelihood;
      graph[e].weight = hri_person_manager::likelihood2weight(likelihood);
      boost::put(boost::edge_weight_t(), graph, e, graph[e].weight);
      graph[e].log_likelihood = hri_person_manager::log_likelihood(likelihood);
    } else if (std::regex_search(line, result, add_node_regex) && result.size() > 1) {
      auto a = result.str(1);

      auto node_a = boost::add_vertex(graph);
      graph[node_a].name = a;
      graph[node_a].type = get_type(a);
      if (a.rfind("anon", 0) == 0) {
        graph[node_a].anonymous = true;
      }
    } else {
      throw std::runtime_error(std::string("malformed mermaid input: ") + line);
    }
  }
}

struct MyCallback
{
  template<typename CorrespondenceMap1To2, typename CorrespondenceMap2To1>
  bool operator()(CorrespondenceMap1To2 /*f*/, CorrespondenceMap2To1) const
  {
    return false;  // breaks after the first subgraph is found
  }
};

std::string print_graph(const hri_person_manager::Graph & graph)
{
  std::stringstream ret_stream;
  for (const auto & n : boost::make_iterator_range(boost::vertices(graph))) {
    ret_stream << "  - " << graph[n].name << std::endl;
  }
  for (const auto & e : boost::make_iterator_range(boost::edges(graph))) {
    hri_person_manager::Node a = boost::source(e, graph);
    hri_person_manager::Node b = boost::target(e, graph);

    ret_stream << "  " << graph[a].name << " -- " << graph[e].likelihood << " -- " << graph[b].name
               << std::endl;
  }
  return ret_stream.str();
}

std::string print_associations(const hri_person_manager::Graph & graph)
{
  std::stringstream ret_stream;
  hri_person_manager::Graph temp_graph(graph);

  std::vector<int> component(boost::num_vertices(temp_graph));  // component map
  auto nb_components = boost::connected_components(temp_graph, &component[0]);

  std::vector<hri_person_manager::Graph> connected_components;

  for (int i = 0; i < nb_components; i++) {
    auto subG = temp_graph.create_subgraph();
    for (size_t j = 0; j < boost::num_vertices(temp_graph); j++) {
      if (component[j] == i) {
        boost::add_vertex(j, subG);
      }
    }
    // size might be zero if the component only includes invalid (ie, removed) nodes
    if (boost::num_vertices(subG) > 0) {
      connected_components.push_back(subG);
    }
  }

  for (size_t i = 0; i < connected_components.size(); i++) {
    ret_stream << " Association " << i + 1 << ":" << std::endl
               << print_graph(connected_components[i]);
  }

  auto ret = ret_stream.str();
  return (ret.empty()) ? " none\n" : ret_stream.str();
}

void run_mermaid_test_step(
  const std::string & test_name, hri_person_manager::PersonMatcher & input,
  const hri_person_manager::Graph & graph, const hri_person_manager::Graph & output)
{
  std::cout << "[TEST STEP] " << test_name << std::endl << std::endl;

  auto input_g = input.getInternalGraph(true);
  auto vos = boost::copy_range<std::vector<hri_person_manager::Node>>(boost::vertices(input_g));

  bool graph_matches_internal_current_state = (
    (boost::num_vertices(input_g) == boost::num_vertices(graph)) &&
    // need to run the VF2 isomorphism *twice*: once to check that 'input' is
    // isomorphic to a subgraph of 'graph', once to check to opposite.
    // A->B
    vf2_subgraph_iso(
      input_g, graph, MyCallback(), vos, boost::edges_equivalent(
        [&input_g, &graph]
          (const hri_person_manager::Edge & e1, const hri_person_manager::Edge & e2)
        {return hri_person_manager::compare_floats(input_g[e1].likelihood, graph[e2].likelihood);}
      ).vertices_equivalent(
        [&input_g, &graph]
          (const hri_person_manager::Node & n1, const hri_person_manager::Node & n2)
        {
          return
          (input_g[n1].name == graph[n2].name) || (input_g[n1].anonymous && graph[n2].anonymous);
        })) &&
    // B->A
    vf2_subgraph_iso(
      graph, input_g, MyCallback(), vos, boost::edges_equivalent(
        [&graph, &input_g]
          (const hri_person_manager::Edge & e1, const hri_person_manager::Edge & e2)
        {return hri_person_manager::compare_floats(graph[e1].likelihood, input_g[e2].likelihood);}
      ).vertices_equivalent(
        [&graph, &input_g]
          (const hri_person_manager::Node & n1, const hri_person_manager::Node & n2)
        {
          return
          (graph[n1].name == input_g[n2].name) || (graph[n1].anonymous && input_g[n2].anonymous);
        })));
  EXPECT_TRUE(graph_matches_internal_current_state)
    << "The expected graph does not match the current internal state." << std::endl
    << "Current graph:" << std::endl
    << print_associations(input_g)
    << "Expected graph:" << std::endl
    << print_associations(graph);

  auto associations = input.getRawAssociations();

  hri_person_manager::Graph final_graph;
  for (const auto & g : associations) {
    boost::copy_graph(g.first, final_graph);
  }

  vos = boost::copy_range<std::vector<hri_person_manager::Node>>(boost::vertices(final_graph));

  bool output_matches_resulting_associations = (
    (boost::num_vertices(final_graph) == boost::num_vertices(output)) &&
    // same as above, needs to run the VF2 algorithm twice.
    // A->B
    vf2_subgraph_iso(
      final_graph, output, MyCallback(), vos, boost::edges_equivalent(
        [&final_graph, &output]
          (const hri_person_manager::Edge & e1, const hri_person_manager::Edge & e2)
        {
          return hri_person_manager::compare_floats(
            final_graph[e1].likelihood, output[e2].likelihood);
        }
      ).vertices_equivalent(
        [&final_graph, &output]
          (const hri_person_manager::Node & n1, const hri_person_manager::Node & n2)
        {
          return
          (final_graph[n1].name == output[n2].name) ||
          (final_graph[n1].anonymous && output[n2].anonymous);
        })) &&
    // B->A
    vf2_subgraph_iso(
      output, final_graph, MyCallback(), vos, boost::edges_equivalent(
        [&output, &final_graph]
          (const hri_person_manager::Edge & e1, const hri_person_manager::Edge & e2)
        {
          return hri_person_manager::compare_floats(
            output[e1].likelihood, final_graph[e2].likelihood);
        }
      ).vertices_equivalent(
        [&output, &final_graph]
          (const hri_person_manager::Node & n1, const hri_person_manager::Node & n2)
        {
          return
          (output[n1].name == final_graph[n2].name) ||
          (output[n1].anonymous && final_graph[n2].anonymous);
        })));
  EXPECT_TRUE(output_matches_resulting_associations)
    << "The expected output does not match the resulting associations." << std::endl
    << "Resulting output:" << std::endl
    << print_associations(final_graph)
    << "Expected output:" << std::endl
    << print_associations(output);
}

void run_mermaid_test_case(const std::string & file_name)
{
  std::cout << "Parsing file " << file_name << std::endl;
  std::ifstream input_file(file_name);
  std::string line;
  std::string in_section;
  std::vector<std::string> buffer;

  bool has_test = false;
  // false to use sequential anonmyous ID instead of random ones
  hri_person_manager::PersonMatcher input(0.5, false);
  hri_person_manager::Graph graph, output;
  std::string test_name;

  std::regex threshold_regex("THRESHOLD: ([\\d.]+)");
  std::smatch result;

  while (std::getline(input_file, line)) {
    if (std::regex_search(line, result, threshold_regex) && result.size() > 1) {
      auto threshold = stof(result.str(1));
      std::cout << "Setting likelihood threshold to " << threshold << std::endl;
      input.setThreshold(threshold);
      continue;
    }
    if (line.rfind("#", 0) == 0) {
      // parse buffer
      if (in_section == kInput) {
        input_parser(buffer, input);
      }
      if (in_section == kGraph) {
        graph = hri_person_manager::Graph();
        mermaid_parser(buffer, graph);
      }
      if (in_section == kOutput) {
        output = hri_person_manager::Graph();
        mermaid_parser(buffer, output);
      }

      in_section.clear();
      buffer.clear();
    }
    if (line.rfind("# ", 0) == 0) {
      if (has_test) {
        run_mermaid_test_step(test_name, input, graph, output);
        has_test = false;
      }
      test_name = line.substr(2);
      has_test = true;
      continue;
    }
    if (line.rfind("## INPUT", 0) == 0) {
      in_section = kInput;
      continue;
    }
    if (line.rfind("## OUTPUT", 0) == 0) {
      in_section = kOutput;
      continue;
    }
    if (line.rfind("## GRAPH", 0) == 0) {
      in_section = kGraph;
      continue;
    }

    buffer.push_back(line);
  }
}

TEST(PersonMatcherTest, LongSequence)
{
  run_mermaid_test_case("mermaid_tests_data/long_sequence.md");
}

TEST(PersonMatcherTest, NotSoTricky)
{
  run_mermaid_test_case("mermaid_tests_data/not_so_tricky.md");
}

TEST(PersonMatcherTest, RealWorld1)
{
  run_mermaid_test_case("mermaid_tests_data/real_world_1.md");
}

TEST(PersonMatcherTest, RealWorld2)
{
  run_mermaid_test_case("mermaid_tests_data/real_world_2.md");
}

TEST(PersonMatcherTest, BasicAssociationModel)
{
  {
    hri_person_manager::PersonMatcher model(0.4);

    EXPECT_THROW({model.getAssociation("p1");}, std::out_of_range);

    hri_person_manager::Relations data{
      {"f1", hri::FeatureType::kFace, "f1", hri::FeatureType::kFace, 1.},
      {"p1", hri::FeatureType::kPerson, "f1", hri::FeatureType::kFace, 1.0}};
    model.update(data);
    auto association = model.getAssociation("p1");
    EXPECT_EQ(association, (std::map<hri::FeatureType, hri::ID>{{hri::FeatureType::kFace, "f1"}}));
    EXPECT_TRUE(association.find(hri::FeatureType::kVoice) == association.end());
    EXPECT_TRUE(association.find(hri::FeatureType::kBody) == association.end());

    model.update({{"p1", hri::FeatureType::kPerson, "f1", hri::FeatureType::kFace, 0.9}});
    association = model.getAssociation("p1");
    EXPECT_EQ(association, (std::map<hri::FeatureType, hri::ID>{{hri::FeatureType::kFace, "f1"}}));
    EXPECT_TRUE(association.find(hri::FeatureType::kVoice) == association.end());
    EXPECT_TRUE(association.find(hri::FeatureType::kBody) == association.end());

    model.update({{"p1", hri::FeatureType::kPerson, "f1", hri::FeatureType::kFace, 0.4}});
    association = model.getAssociation("p1");
    EXPECT_EQ(association, (std::map<hri::FeatureType, hri::ID>{{hri::FeatureType::kFace, "f1"}}));
    EXPECT_TRUE(association.find(hri::FeatureType::kVoice) == association.end());
    EXPECT_TRUE(association.find(hri::FeatureType::kBody) == association.end());

    model.update({{"p1", hri::FeatureType::kPerson, "f1", hri::FeatureType::kFace, 0.35}});
    association = model.getAssociation("p1");
    EXPECT_TRUE(association.empty());

    model.update({{"p1", hri::FeatureType::kPerson, "f1", hri::FeatureType::kFace, 0.1}});
    association = model.getAssociation("p1");
    EXPECT_TRUE(association.empty());

    model.update({{"p1", hri::FeatureType::kPerson, "f1", hri::FeatureType::kFace, 0.0}});
    association = model.getAssociation("p1");
    EXPECT_TRUE(association.empty());

    model.update({{"p1", hri::FeatureType::kPerson, "f1", hri::FeatureType::kFace, 0.9}});
    association = model.getAssociation("p1");
    EXPECT_EQ(association, (std::map<hri::FeatureType, hri::ID>{{hri::FeatureType::kFace, "f1"}}));
    EXPECT_TRUE(association.find(hri::FeatureType::kVoice) == association.end());
    EXPECT_TRUE(association.find(hri::FeatureType::kBody) == association.end());
  }

  {
    hri_person_manager::PersonMatcher model(0.05);

    hri_person_manager::Relations data{
      {"f1", hri::FeatureType::kFace, "f1", hri::FeatureType::kFace, 1.},
      {"p1", hri::FeatureType::kPerson, "f1", hri::FeatureType::kFace, 0.1}};
    model.update(data);
    auto association = model.getAssociation("p1");
    EXPECT_EQ(association, (std::map<hri::FeatureType, hri::ID>{{hri::FeatureType::kFace, "f1"}}));
    EXPECT_TRUE(association.find(hri::FeatureType::kVoice) == association.end());
    EXPECT_TRUE(association.find(hri::FeatureType::kBody) == association.end());

    model.update({{"p1", hri::FeatureType::kPerson, "f1", hri::FeatureType::kFace, 0.01}});
    association = model.getAssociation("p1");
    EXPECT_TRUE(association.empty());

    model.update({{"p1", hri::FeatureType::kPerson, "f1", hri::FeatureType::kFace, 0.0}});
    association = model.getAssociation("p1");
    EXPECT_TRUE(association.empty());
  }
}

TEST(PersonMatcherTest, AssociationNetwork)
{
  auto model = hri_person_manager::PersonMatcher(0.4);

  // Test small transitive network, with p1 -> {f1, b1} more likely than p1 -> {f1, b2}
  hri_person_manager::Relations data{
    {"f1", hri::FeatureType::kFace, "f1", hri::FeatureType::kFace, 1.},
    {"b1", hri::FeatureType::kBody, "b1", hri::FeatureType::kBody, 1.},
    {"b2", hri::FeatureType::kBody, "b2", hri::FeatureType::kBody, 1.},
    {"f1", hri::FeatureType::kFace, "b1", hri::FeatureType::kBody, 0.7},
    {"f1", hri::FeatureType::kFace, "b2", hri::FeatureType::kBody, 0.6},
    {"p1", hri::FeatureType::kPerson, "f1", hri::FeatureType::kFace, 0.9}};
  model.update(data);
  auto association = model.getAssociation("p1");
  EXPECT_EQ(
    association, (std::map<hri::FeatureType, hri::ID>{{
      hri::FeatureType::kFace, "f1"}, {hri::FeatureType::kBody, "b1"}}));
  EXPECT_TRUE(association.find(hri::FeatureType::kVoice) == association.end());

  // Test *updating* an edge. The previous value should be replaced
  // A: a computed edge f1 -- 0.63 -- p1 was added -> p1 should still be
  // associated to f1 and b1
  data = {{"f1", hri::FeatureType::kFace, "b1", hri::FeatureType::kBody, 0.0}};
  model.update(data);
  association = model.getAssociation("p1");
  EXPECT_EQ(
    association, (std::map<hri::FeatureType, hri::ID>{{
      hri::FeatureType::kFace, "f1"}, {hri::FeatureType::kBody, "b1"}}));
  EXPECT_TRUE(association.find(hri::FeatureType::kVoice) == association.end());

  data = {{"f1", hri::FeatureType::kFace, "b2", hri::FeatureType::kBody, 0.64}};
  // B: now, the relation f1 <-> b2 is stronger than f1 <-> b1: p1 should be
  // associated to f1 and b2
  model.update(data);
  association = model.getAssociation("p1");
  EXPECT_EQ(
    association, (std::map<hri::FeatureType, hri::ID>{{
      hri::FeatureType::kFace, "f1"}, {hri::FeatureType::kBody, "b2"}}));
  EXPECT_TRUE(association.find(hri::FeatureType::kVoice) == association.end());

  // this time, the likelihood of p1 being associated to b2 is < threshold (0.9
  // * 0.4 < 0.4)
  // p1 should be associated back to b1
  data = {{"f1", hri::FeatureType::kFace, "b2", hri::FeatureType::kBody, 0.4}};
  model.update(data);
  association = model.getAssociation("p1");
  EXPECT_EQ(
    association, (std::map<hri::FeatureType, hri::ID>{{
      hri::FeatureType::kFace, "f1"}, {hri::FeatureType::kBody, "b1"}}));
  EXPECT_TRUE(association.find(hri::FeatureType::kVoice) == association.end());
}

TEST(PersonMatcherTest, ResetEraseIds)
{
  auto model = hri_person_manager::PersonMatcher(0.4);

  hri_person_manager::Relations data{
    {"f1", hri::FeatureType::kFace, "f1", hri::FeatureType::kFace, 1.},
    {"p1", hri::FeatureType::kPerson, "f1", hri::FeatureType::kFace, 0.9}};
  model.update(data);
  auto association = model.getAssociation("p1");
  EXPECT_EQ(association, (std::map<hri::FeatureType, hri::ID>{{hri::FeatureType::kFace, "f1"}}));

  model.erase("f1");
  association = model.getAssociation("p1");
  EXPECT_TRUE(association.empty());

  model.erase("p1");
  EXPECT_THROW({model.getAssociation("p1");}, std::out_of_range);

  data = {
    {"f1", hri::FeatureType::kFace, "f1", hri::FeatureType::kFace, 1.},
    {"b1", hri::FeatureType::kBody, "b1", hri::FeatureType::kBody, 1.},
    {"b2", hri::FeatureType::kBody, "b2", hri::FeatureType::kBody, 1.},
    {"p1", hri::FeatureType::kPerson, "f1", hri::FeatureType::kFace, 0.9},
    {"f1", hri::FeatureType::kFace, "b2", hri::FeatureType::kBody, 0.6},
    {"b1", hri::FeatureType::kBody, "f1", hri::FeatureType::kFace, 0.7}};
  model.update(data);
  association = model.getAssociation("p1");
  EXPECT_EQ(
    association, (std::map<hri::FeatureType, hri::ID>{{
      hri::FeatureType::kFace, "f1"}, {hri::FeatureType::kBody, "b1"}}));

  model.erase("b1");
  association = model.getAssociation("p1");
  EXPECT_EQ(
    association, (std::map<hri::FeatureType, hri::ID>{{
      hri::FeatureType::kFace, "f1"}, {hri::FeatureType::kBody, "b2"}}));

  data = {
    {"b1", hri::FeatureType::kBody, "b1", hri::FeatureType::kBody, 1.},
    {"f1", hri::FeatureType::kFace, "b1", hri::FeatureType::kBody, 0.7}};
  model.update(data);
  association = model.getAssociation("p1");
  EXPECT_EQ(
    association, (std::map<hri::FeatureType, hri::ID>{{
      hri::FeatureType::kFace, "f1"}, {hri::FeatureType::kBody, "b1"}}));

  model.reset();
  EXPECT_THROW({model.getAssociation("p1");}, std::out_of_range);

  data = {
    {"f1", hri::FeatureType::kFace, "f1", hri::FeatureType::kFace, 1.},
    {"b1", hri::FeatureType::kBody, "b1", hri::FeatureType::kBody, 1.},
    {"b2", hri::FeatureType::kBody, "b2", hri::FeatureType::kBody, 1.},
    {"p1", hri::FeatureType::kPerson, "f1", hri::FeatureType::kFace, 0.9},
    {"f1", hri::FeatureType::kFace, "b2", hri::FeatureType::kBody, 0.6},
    {"b1", hri::FeatureType::kBody, "f1", hri::FeatureType::kFace, 0.7}};
  model.update(data);
  association = model.getAssociation("p1");
  EXPECT_EQ(
    association, (std::map<hri::FeatureType, hri::ID>{{
      hri::FeatureType::kFace, "f1"}, {hri::FeatureType::kBody, "b1"}}));

  model.reset();
  EXPECT_THROW({model.getAssociation("p1");}, std::out_of_range);

  model.update(data);
  association = model.getAssociation("p1");
  EXPECT_EQ(
    association, (std::map<hri::FeatureType, hri::ID>{{
      hri::FeatureType::kFace, "f1"}, {hri::FeatureType::kBody, "b1"}}));
}

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
