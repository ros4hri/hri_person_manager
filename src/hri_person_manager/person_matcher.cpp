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


#include "hri_person_manager/person_matcher.hpp"

#include <algorithm>
#include <array>
#include <cassert>
#include <functional>
#include <iostream>
#include <iterator>
#include <limits>
#include <map>
#include <random>
#include <set>
#include <sstream>
#include <string>
#include <tuple>
#include <vector>

#include "boost/graph/connected_components.hpp"
#include "boost/graph/dijkstra_shortest_paths.hpp"
#include "boost/graph/graphviz.hpp"
#include "boost/graph/kruskal_min_spanning_tree.hpp"
#include "boost/graph/subgraph.hpp"
#include "boost/range/iterator_range_core.hpp"
#include "hri/types.hpp"

#include "hri_person_manager/managed_person.hpp"

namespace hri_person_manager
{

using IdNodeMap = std::map<hri::FeatureType, std::map<hri::ID, Node>>;

/** not super optimised, but not called very often either
 */
hri::ID generate_random_id(const int len = 5)
{
  static const std::array<std::string, 26> alphanum{
    {"a", "b", "c", "d", "e", "f", "g", "h", "i", "j", "k", "l", "m",
      "n", "o", "p", "q", "r", "s", "t", "u", "v", "w", "x", "y", "z"}
  };
  std::string tmp_s;
  tmp_s.reserve(len);

  std::random_device dev;
  std::mt19937 rng(dev());
  std::uniform_int_distribution<uint32_t> rnd_dist(0, alphanum.size() - 1);

  for (int i = 0; i < len; ++i) {
    tmp_s += alphanum[rnd_dist(rng)];
  }

  return tmp_s;
}

hri::ID generate_hash_id(hri::ID id, const int len = 5)
{
  static const std::array<std::string, 26> alphanum{
    {"a", "b", "c", "d", "e", "f", "g", "h", "i", "j", "k", "l", "m",
      "n", "o", "p", "q", "r", "s", "t", "u", "v", "w", "x", "y", "z"}
  };
  std::string tmp_s;
  tmp_s.reserve(len);

  auto hash = std::hash<hri::ID>{}(id);

  for (int i = 0; i < len; ++i) {
    tmp_s += alphanum[hash % 10];
    hash /= 10;
  }

  return tmp_s;
}

//////////// helpers for write_graphviz ////////////////
const std::map<unsigned int, std::string> kColorScheme{
  {0, "white"}, {1, "cadetblue1"}, {2, "cornsilk1"}, {3, "darkolivegreen2"}, {4, "darkorange1"},
  {5, "gold"}, {6, "gray66"}, {7, "lightgreen"}, {8, "lightslateblue"}, {9, "mediumorchid1"},
  {10, "turquoise1"}
};

class NodeLabeLWriter
{
public:
  explicit NodeLabeLWriter(Graph g)
  : g_(g)
  {}

  template<class NodeOrEdge>
  void operator()(std::ostream & out, const NodeOrEdge & v) const
  {
    auto name = g_[v].name;
    auto type = g_[v].type;

    std::string feat;
    switch (type) {
      case hri::FeatureType::kFace:
        feat = " [face]";
        break;
      case hri::FeatureType::kBody:
        feat = " [body]";
        break;
      case hri::FeatureType::kVoice:
        feat = " [voice]";
        break;
      case hri::FeatureType::kPerson:
      case hri::FeatureType::kTrackedPerson:
        feat = " [person]";
        break;
      case hri::FeatureType::kInvalid:
        feat = " [invalid]";
        break;
    }

    auto color = kColorScheme.at(g_[v].association_id % kColorScheme.size());

    if (g_[v].anonymous) {
      out << "[style=dashed shape=box color=" << color << " label=\"" << name << feat << "\"]";
    } else {
      out << "[style=filled shape=box color=" << color << " label=\"" << name << feat << "\"]";
    }
  }

private:
  Graph g_;
};

class WeightLabelWriter
{
public:
  explicit WeightLabelWriter(Graph g)
  : g_(g)
  {}

  template<class NodeOrEdge>
  void operator()(std::ostream & out, const NodeOrEdge & e) const
  {
    out << "[label=\"" << g_[e].likelihood << "\"]";
  }

private:
  Graph g_;
};
///////////////////////////////////////////////////////////////////////////

PersonMatcher::PersonMatcher(float likelihood_threshold, bool random_anonymous_name)
: random_anonymous_name_(random_anonymous_name)
{
  setThreshold(likelihood_threshold);
}

void PersonMatcher::pruneUnlikelyConnections()
{
  std::set<Edge> edges_to_prune;
  for (const auto & e : boost::make_iterator_range(boost::edges(g_))) {
    if (g_[e].likelihood < threshold_) {
      edges_to_prune.insert(e);
    }
  }

  for (const auto & e : edges_to_prune) {
    std::cout << "Removing edge from " << g_[source(e, g_)].name << " to " << g_[target(e, g_)].name
              << ": likelihood=" << g_[e].likelihood << " below threshold=" << threshold_
              << std::endl;
    boost::remove_edge(e, g_);
  }
}

std::vector<NodeSets> PersonMatcher::buildPartitions(Nodes nodes) const
{
  Node head = nodes.back();
  // note that the node descriptors passed to build_partition are global
  auto head_type = g_[head].type;

  nodes.pop_back();

  if (nodes.size() == 0) {
    return {{{{head}, head_type}}};
  }

  // 1. recursively get partitions for a subset of nodes
  auto partitions = buildPartitions(nodes);

  std::vector<NodeSets> updated_partitions;

  // 2. add 'head' node to existing partitions if valid
  for (const NodeSets & partition : partitions) {
    // 2.1 add 'head' to each subgraphs, one after the other
    for (size_t idx = 0; idx < partition.size(); idx++) {
      auto feature_mask = partition[idx].second;

      // a valid partition must not contain the same feature twice
      if (!static_cast<bool>(feature_mask & head_type)) {
        // copy the current partition
        NodeSets new_partition(partition);

        new_partition[idx].first.push_back(head);
        new_partition[idx].second = feature_mask | head_type;

        updated_partitions.push_back(new_partition);
      }
    }

    // 2.2 add a last partition made of [ {head}, other partitions... ]
    NodeSets new_partition(partition);
    new_partition.push_back({{head}, head_type});
    updated_partitions.push_back(new_partition);
  }

  return updated_partitions;
}

std::vector<Subgraphs> PersonMatcher::getPartitions(Graph & graph)
{
  Nodes nodes;
  for (const auto & n : boost::make_iterator_range(boost::vertices(graph))) {
    assert(boost::get(&NodeProps::valid, graph, n));
    nodes.push_back(graph.local_to_global(n));
  }

  auto raw_partitions = buildPartitions(nodes);

  std::vector<Subgraphs> viable_partitions;
  // component map to call boost connected_components
  std::vector<int> component(boost::num_vertices(graph));

  // -----------------------------------
  // TODO(SLE): optimize this, it is the slowest part of the code, due to the generation of many
  // subgraphs

  // filter the partitions to only keep the *connected* ones
  for (const auto & nodesets : raw_partitions) {
    Subgraphs partition;
    partition.reserve(nodes.size());

    bool connected = true;
    for (const auto & nodeset : nodesets) {
      auto subgraph = g_.create_subgraph(nodeset.first.begin(), nodeset.first.end());

      auto nb_components = boost::connected_components(
        subgraph, &component[0]);  // (note that we do not need the component map `component`)

      if (nb_components > 1) {
        connected = false;
        break;
      }
      partition.push_back({subgraph, nodeset.second});
    }

    if (connected) {
      viable_partitions.push_back(partition);
    }
  }
  // -----------------------------------

  return viable_partitions;
}

std::string PersonMatcher::setGetAnonymousId(std::vector<std::string> feature_ids)
{
  std::sort(feature_ids.begin(), feature_ids.end());

  std::string anon_id;

  // did we already assign an anonymous hri::ID to one of these features?
  for (std::string & id : feature_ids) {
    if (anonymous_ids_map_.count(id) > 0) {
      anon_id = anonymous_ids_map_.at(id);
      break;
    }
  }

  // found?
  // erase all the anonymous_id_map entry that contain that anonymous id, to
  // make sure a feature from *another* association does not end up re-using
  // the same id
  if (anon_id.size() != 0) {
    for (auto i = anonymous_ids_map_.begin(), last = anonymous_ids_map_.end(); i != last; ) {
      if ((i->second == anon_id)) {
        i = anonymous_ids_map_.erase(i);
      } else {
        ++i;
      }
    }
  } else {  // not found? create new random id
    if (random_anonymous_name_) {
      anon_id = kAnonymous + generate_hash_id(*feature_ids.begin());
    } else {
      anon_id = "anon" + std::to_string(incremental_anon_id_);
      incremental_anon_id_++;
    }
    // cout << "Generating new anon id " << anon_id << endl;
  }

  for (std::string & id : feature_ids) {
    anonymous_ids_map_[id] = anon_id;
  }

  return anon_id;
}

void PersonMatcher::addAnonymousPerson(Subgraph & association)
{
  auto & subgraph = association.first;
  assert(&subgraph.parent() == &g_);
  auto feature_mask = association.second;

  // return if there is already a person in this association
  if (static_cast<bool>(feature_mask & hri::FeatureType::kPerson)) {
    return;
  }

  std::vector<std::string> features_ids;
  for (const auto & n : boost::make_iterator_range(boost::vertices(subgraph))) {
    features_ids.push_back(boost::get(&NodeProps::name, subgraph, n));
  }

  // create a new global node for an anonymous person
  auto anon = boost::add_vertex(g_);

  g_[anon].name = setGetAnonymousId(features_ids);
  g_[anon].type = hri::FeatureType::kPerson;
  g_[anon].anonymous = true;

  // add the node to this association -- the vertex descriptor inside the
  // subgraph might be different than in the global graph -> sub_node != anon
  auto sub_node = boost::add_vertex(anon, subgraph);

  for (const auto & n : boost::make_iterator_range(boost::vertices(subgraph))) {
    if (n != sub_node) {
      // adding an edge to a subgraph also adds it to the parent
      auto e = boost::add_edge(sub_node, n, subgraph).first;
      boost::put(&EdgeProps::likelihood, subgraph, e, threshold_);
      boost::put(&EdgeProps::weight, subgraph, e, likelihood2weight(threshold_));
      boost::put(&EdgeProps::log_likelihood, subgraph, e, log_likelihood(threshold_));
      boost::put(&EdgeProps::computed, subgraph, e, true);
    }
  }
}

void PersonMatcher::clearAnonymousPersons()
{
  std::vector<Node> anonyms;

  for (const auto & n : boost::make_iterator_range(boost::vertices(g_))) {
    if (g_[n].anonymous == true) {
      anonyms.push_back(n);
    }
  }

  for (const auto & n : anonyms) {
    boost::clear_vertex(n, g_);
    g_[n].valid = false;
  }
}

void PersonMatcher::fullyConnectPersons(Subgraph & association)
{
  auto & graph = association.first;
  auto feature_mask = association.second;

  // return if there is no person in this association
  if (!static_cast<bool>(feature_mask & hri::FeatureType::kPerson)) {
    return;
  }

  // required to operate on a *copy* of graph, otherwise operations like
  // remove_edge do not work (probably because the edge might belong to other
  // subgraphs)
  auto graph_copy = cleanGraphCopy(graph);

  // 0. find the 'person' node
  Node person;
  for (const auto & n : boost::make_iterator_range(boost::vertices(graph_copy))) {
    if (boost::get(&NodeProps::type, graph_copy, n) == hri::FeatureType::kPerson) {
      person = n;
      break;
    }
  }

  // 1. remove computed edges
  std::vector<std::tuple<Node, Node, EdgeProps>> computed_edges;
  for (const auto & e : boost::make_iterator_range(boost::edges(graph_copy))) {
    if (boost::get(&EdgeProps::computed, graph_copy, e)) {
      computed_edges.push_back(
        std::make_tuple(boost::source(e, graph_copy), boost::target(e, graph_copy), graph_copy[e]));
    }
  }
  for (auto e : computed_edges) {
    Node s, t;
    EdgeProps ep;
    std::tie(s, t, ep) = e;
    boost::remove_edge(s, t, graph_copy);
  }

  // 2. compute shortest paths

  // vector for storing distance property
  std::vector<float> d(boost::num_vertices(graph_copy));

  boost::dijkstra_shortest_paths(
    graph_copy, person,
    boost::distance_map(&d[0]).weight_map(boost::get(&EdgeProps::log_likelihood, graph_copy)));

  // 3. create computed edges for features not directly connected to the person

  std::vector<std::tuple<Node, Node, EdgeProps>> edges_to_update;

  for (const auto & n : boost::make_iterator_range(boost::vertices(graph_copy))) {
    if (n == person) {
      continue;
    }

    const auto & e = boost::edge(n, person, graph_copy);

    // if already connected, continue
    if (e.second) {
      continue;
    }

    auto dist = d[n];

    // no path
    if (compare_floats(dist, std::numeric_limits<decltype(EdgeProps::log_likelihood)>::max())) {
      continue;
    }
    float likelihood = inv_log_likelihood(dist);

    if (likelihood < threshold_) {
      continue;
    }

    auto out = boost::add_edge(n, person, graph_copy);

    if (out.second) {
      Edge new_e = out.first;

      boost::put(&EdgeProps::likelihood, graph_copy, new_e, likelihood);
      boost::put(&EdgeProps::weight, graph_copy, new_e, likelihood2weight(likelihood));
      boost::put(&EdgeProps::log_likelihood, graph_copy, new_e, dist);
      boost::put(&EdgeProps::computed, graph_copy, new_e, true);

      edges_to_update.push_back({n, person, graph_copy[new_e]});
    } else {
      std::cout << "[EE] failed to add computed edge between "
                << boost::get(&NodeProps::name, graph_copy, person) << " and "
                << boost::get(&NodeProps::name, graph_copy, n) << ": edge already exist!"
                << std::endl;
      assert(false);
    }
  }

  // 4. bring back the previously removed edges, *if they have not been updated*
  for (const auto & e : computed_edges) {
    // parallel edges are not permitted -> add_edge will only insert the edge if not already present
    auto ok = boost::add_edge(std::get<0>(e), std::get<1>(e), graph_copy);
    // ok.second = true: the edge was added -> value not previously updated
    // ok.second = false: the edge was already there: value updated!
    if (ok.second) {
      graph_copy[ok.first] = std::get<2>(e);
      edges_to_update.push_back(e);
    }
  }

  // finally, perform the updates on the original graph (eg, not the copy)
  for (const auto & e : edges_to_update) {
    Node s, t;
    EdgeProps ep;
    std::tie(s, t, ep) = e;

    Node s_orig = get_node_by_name(boost::get(&NodeProps::name, graph_copy, s), graph);
    Node t_orig = get_node_by_name(boost::get(&NodeProps::name, graph_copy, t), graph);

    // this either create a new edge (out.second = true), or update the value
    // of an existing edge (out.second = false).  important: adding/updating an
    // edge to the subgraph also adds it to the parent graph
    auto out = boost::add_edge(s_orig, t_orig, graph);
    graph[out.first] = ep;
  }
}

float PersonMatcher::partitionAffinity(const Subgraphs & partition) const
{
  float affinity = 0.;

  for (const Subgraph & association : partition) {
    auto graph = cleanGraphCopy(association.first);

    // set the edge_weight property, as I've not been able to use the EdgeProps::weight
    // property directly in kruskal_minimum_spanning_tree
    for (const auto & e : boost::make_iterator_range(boost::edges(graph))) {
      boost::put(boost::edge_weight_t(), graph, e, boost::get(&EdgeProps::weight, graph, e));
    }

    std::vector<Edge> spanning_tree;

    boost::kruskal_minimum_spanning_tree(graph, std::back_inserter(spanning_tree));

    for (const auto & e : spanning_tree) {
      affinity += boost::get(&EdgeProps::likelihood, graph, e);
    }
  }

  return affinity;
}

Graph PersonMatcher::cleanGraphCopy(
  const Graph & graph, bool remove_anonymous, bool remove_computed_edges) const
{
  Graph res;

  for (const auto & n : boost::make_iterator_range(boost::vertices(graph))) {
    if (
      boost::get(&NodeProps::valid, graph, n) &&
      !(remove_anonymous && boost::get(&NodeProps::anonymous, graph, n)))
    {
      Node n_copy = boost::add_vertex(res);
      res[n_copy] = graph[n];
    }
  }

  for (const auto & e : boost::make_iterator_range(boost::edges(graph))) {
    if (remove_computed_edges && boost::get(&EdgeProps::computed, graph, e)) {
      continue;
    }

    Node s = get_node_by_name(boost::get(&NodeProps::name, graph, boost::source(e, graph)), res);
    Node t = get_node_by_name(boost::get(&NodeProps::name, graph, boost::target(e, graph)), res);

    if (s != kInexistantNode && t != kInexistantNode) {
      Edge e_copy = boost::add_edge(s, t, res).first;
      res[e_copy] = graph[e];
    }
  }

  return res;
}

Subgraphs PersonMatcher::computeAssociations()
{
  if (boost::num_vertices(g_) == 0) {
    return {};
  }

  clearAnonymousPersons();

  // !! mutates this->g_
  g_ = cleanGraphCopy(g_);

  // !! mutates this->g_
  pruneUnlikelyConnections();

  Subgraphs complete_partition;

  ////////// Get connected components
  std::vector<int> component(boost::num_vertices(g_));  // component map

  auto nb_components = boost::connected_components(active_graph_, &component[0]);

  // create one subgraph per connected component
  std::vector<Graph> connected_components;

  for (int i = 0; i < nb_components; i++) {
    auto subG = g_.create_subgraph();
    for (size_t j = 0; j < boost::num_vertices(g_); j++) {
      if (component[j] == i) {
        boost::add_vertex(j, subG);
      }
    }
    if (num_vertices(subG) > 0) {
      // size might be zero if the component only includes invalid (ie, removed) nodes
      connected_components.push_back(subG);
    }
  }

  for (auto subgraph : connected_components) {
    auto partitions = getPartitions(subgraph);

    // look for the partitions with the least component
    // -- the maximum possible number of partitions being the number of nodes
    // in the graph
    auto min_len = boost::num_vertices(g_);

    for (auto p : partitions) {
      if (p.size() < min_len) {
        min_len = p.size();
      }
    }

    // only keep the most compact partitions (ie, partitions with the minimal number of subgraph)
    std::vector<Subgraphs> compact_partitions;

    std::copy_if(
      partitions.begin(), partitions.end(), std::back_inserter(compact_partitions),
      [min_len](const Subgraphs & elem) {return elem.size() == min_len;});

    std::sort(
      compact_partitions.begin(), compact_partitions.end(),
      [this](const Subgraphs & e1, const Subgraphs & e2) {
        return partitionAffinity(e1) > partitionAffinity(e2);
      });

    complete_partition.insert(
      complete_partition.end(), compact_partitions[0].begin(), compact_partitions[0].end());
  }

  for (auto & subgraph : complete_partition) {
    addAnonymousPerson(subgraph);
    fullyConnectPersons(subgraph);
  }

  size_t idx = 0;

  ///////
  // assign their association id to each node -- only used to color the graphviz
  // output!
  for (const auto & association : complete_partition) {
    idx += 1;
    auto subgraph = association.first;
    for (const auto & n : boost::make_iterator_range(boost::vertices(subgraph))) {
      boost::put(&NodeProps::association_id, subgraph, n, idx);
    }
  }
  ///////

  return complete_partition;
}

void PersonMatcher::printPartition(const Subgraphs & partition) const
{
  size_t idx = 0;

  for (const auto & association : partition) {
    idx += 1;
    auto subgraph = association.first;

    std::cout << "  Association " << idx << std::endl;

    std::cout << "    Nodes: [";
    for (const auto & n : boost::make_iterator_range(boost::vertices(subgraph))) {
      std::cout << boost::get(&NodeProps::name, subgraph, n) << ", ";
    }
    std::cout << "]" << std::endl;

    for (const auto & e : boost::make_iterator_range(boost::edges(subgraph))) {
      std::cout << "    " << boost::get(&NodeProps::name, subgraph, source(e, subgraph))
                << " <-- " << boost::get(&EdgeProps::likelihood, subgraph, e)
                << " --> " << boost::get(&NodeProps::name, subgraph, target(e, subgraph))
                << std::endl;
    }

    std::cout << std::endl;
  }
}

void PersonMatcher::update(Relations relations, bool create_features_from_candidate_matches)
{
  hri::ID id1, id2;
  hri::FeatureType type1, type2;
  float likelihood;

  for (auto rel : relations) {
    std::tie(id1, type1, id2, type2, likelihood) = rel;

    Node v1 = get_node_by_name(id1, g_);
    Node v2 = get_node_by_name(id2, g_);

    if (likelihood <= 0.0) {
      if (v1 != kInexistantNode && v2 != kInexistantNode) {
        boost::remove_edge(v1, v2, g_);
      }
      continue;
    }

    if (
      !create_features_from_candidate_matches && (v1 == kInexistantNode || v2 == kInexistantNode))
    {
      if (
        (v1 == kInexistantNode && type1 == hri::FeatureType::kPerson && v2 != kInexistantNode) ||
        (v2 == kInexistantNode && type2 == hri::FeatureType::kPerson && v1 != kInexistantNode))
      {
        std::cout << "~features_from_matches=false, but we are trying to create a new person with "
                  << "an existing feature -> still create the person." << std::endl;
      } else {
        std::cout << "Not updating relation as one of the feature does not exist and "
                  << "~features_from_matches=false" << std::endl;
        continue;
      }
    }

    // if needed, create the vertices
    if (v1 == kInexistantNode) {
      v1 = boost::add_vertex(g_);
      g_[v1].name = id1;
      g_[v1].type = type1;
    }

    // id1 == id2 -> we a simply creating a new feature; no edge to add
    if (id1 == id2) {
      continue;
    }

    if (v2 == kInexistantNode) {
      v2 = boost::add_vertex(g_);
      g_[v2].name = id2;
      g_[v2].type = type2;
    }

    Edge e;
    bool ok;

    std::tie(e, ok) = boost::add_edge(v1, v2, g_);
    // (note: e points either to a new edge, or to the existing edge if already present)

    g_[e].likelihood = likelihood;
    g_[e].weight = likelihood2weight(likelihood);
    g_[e].log_likelihood = log_likelihood(likelihood);
    g_[e].computed = false;
  }
}

void PersonMatcher::erase(hri::ID id)
{
  Node v = get_node_by_name(id, g_);
  if (v == kInexistantNode) {
    std::cout << "Unknown hri::ID " << id << "! can not remove it" << std::endl;
    return;
  }

  if (anonymous_ids_map_.count(id)) {
    auto nb = anonymous_ids_map_.erase(id);
    if (nb != 1) {
      std::cout << "Removed " << nb << " elements for hri::ID " << id << " instead of exactly 1"
                << std::endl;
    }
  }
  boost::clear_vertex(v, g_);
  g_[v].valid = false;
}

void PersonMatcher::reset()
{
  g_ = Graph();
  anonymous_ids_map_.clear();
  incremental_anon_id_ = 1;
}

std::map<hri::ID, std::map<hri::FeatureType, hri::ID>> PersonMatcher::getAllAssociations()
{
  // TODO(SLE): we must ensure that g_ is *not mutated* until the end of the method.
  // Might be safer to make a copy of g_ at the start, and run
  // computeAssociations on that copy.
  // However this should be fine, as none of the callback based methods directly mutate
  // the graph (all mutations take place from 'publishPersons')

  auto associations = computeAssociations();

  std::map<hri::ID, std::map<hri::FeatureType, hri::ID>> res;

  for (const auto & association : associations) {
    Node person{kInexistantNode};

    std::map<hri::FeatureType, hri::ID> features;
    auto graph = association.first;

    for (const auto & n : boost::make_iterator_range(boost::vertices(graph))) {
      if (boost::get(&NodeProps::type, graph, n) == hri::FeatureType::kPerson) {
        person = n;
      } else {
        features[boost::get(&NodeProps::type, graph, n)] = boost::get(&NodeProps::name, graph, n);
      }
    }
    if (person != kInexistantNode) {
      res[boost::get(&NodeProps::name, graph, person)] = features;
    }
  }

  return res;
}

std::string PersonMatcher::getGraphviz() const
{
  std::stringstream ss;
  boost::write_graphviz(ss, g_, NodeLabeLWriter(g_), WeightLabelWriter(g_));

  return ss.str();
}

}  // namespace hri_person_manager
