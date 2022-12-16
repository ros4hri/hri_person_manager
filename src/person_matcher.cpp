// Copyright 2022 PAL Robotics S.L.
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

#include <boost/graph/filtered_graph.hpp>
#include <boost/graph/graph_utility.hpp>
#include <boost/graph/properties.hpp>
#include <boost/range/iterator_range_core.hpp>
#include <cmath>
#include <iterator>
#include <limits>
#include <sstream>
#include <stdexcept>
#include <tuple>
#include <array>
#include <algorithm>
#include <random>

#include "person_matcher.h"
#include "managed_person.h"

#include <boost/graph/detail/adjacency_list.hpp>
#include <boost/graph/graph_traits.hpp>
#include <boost/graph/dijkstra_shortest_paths.hpp>
#include <boost/graph/kruskal_min_spanning_tree.hpp>
#include <boost/graph/connected_components.hpp>
#include <boost/graph/graphviz.hpp>

#include "hri/base.h"


using namespace std;
using namespace hri;
using namespace boost;

// should normally be true -- false is useful for eg unit-testing with
// predicatable anonymous IDs
const bool RANDOM_ANONYMOUS_NAME = false;

typedef std::map<hri::FeatureType, std::map<hri::ID, Node>> IdNodeMap;

/** not super optimised, but not called very often either
 */
hri::ID generate_random_id(const int len = 5)
{
  static const std::array<string, 26> alphanum{
    { "a", "b", "c", "d", "e", "f", "g", "h", "i", "j", "k", "l", "m",
      "n", "o", "p", "q", "r", "s", "t", "u", "v", "w", "x", "y", "z" }
  };
  string tmp_s;
  tmp_s.reserve(len);

  std::random_device dev;
  std::mt19937 rng(dev());
  std::uniform_int_distribution<uint32_t> rnd_dist(0, alphanum.size() - 1);

  for (int i = 0; i < len; ++i)
  {
    tmp_s += alphanum[rnd_dist(rng)];
  }

  return tmp_s;
}

//////////// helpers for write_graphviz ////////////////
class node_label_writer
{
public:
  node_label_writer(Graph _g, std::map<hri::FeatureType, std::set<hri::ID>> _id_types)
    : g(_g), id_types(_id_types)
  {
  }
  template <class NodeOrEdge>
  void operator()(std::ostream& out, const NodeOrEdge& v) const
  {
    auto name = get(&NodeProps::name, g, v);

    hri::FeatureType type;

    for (auto _type : { face, body, voice, person })
    {
      if (id_types.at(_type).count(name))
      {
        type = _type;
        break;
      }
    }
    string feat;
    switch (type)
    {
      case face:
        feat = " [face]";
        break;
      case body:
        feat = " [body]";
        break;
      case voice:
        feat = " [voice]";
        break;
      case person:
      case tracked_person:
        feat = " [person]";
        break;
    }


    if (name.find(hri::ANONYMOUS) != std::string::npos)
    {
      out << "[style=dashed shape=box label=\"" << name << feat << "\"]";
    }
    else
    {
      out << "[shape=box label=\"" << name << feat << "\"]";
    }
  }

private:
  Graph g;
  std::map<hri::FeatureType, std::set<hri::ID>> id_types;
};

class weight_label_writer
{
public:
  weight_label_writer(Graph _g) : g(_g)
  {
  }
  template <class NodeOrEdge>
  void operator()(std::ostream& out, const NodeOrEdge& e) const
  {
    out << "[label=\"" << g[e].likelihood << "\"]";
  }

private:
  Graph g;
};
///////////////////////////////////////////////////////////////////////////

Node get_vertex(const Graph& g, const ID id)
{
  Graph::vertex_iterator v, vend;
  for (boost::tie(v, vend) = vertices(g); v != vend; ++v)
  {
    if (g[*v].valid && id == g[*v].name)
      return *v;
  }

  return INEXISTANT_VERTEX;
}

PersonMatcher::PersonMatcher(float likelihood_threshold)
{
  set_threshold(likelihood_threshold);

  for (auto type : { face, body, voice, person })
  {
    id_types[type] = set<ID>();
  }
}

void PersonMatcher::prune_unlikely_connections()
{
  set<Edge> edges_to_prune;
  for (const auto& e : boost::make_iterator_range(edges(g)))
  {
    if (g[e].likelihood < threshold)
    {
      edges_to_prune.insert(e);
    }
  }

  for (const auto& e : edges_to_prune)
  {
    cout << "Removing edge from " << g[source(e, g)].name << " to " << g[target(e, g)].name
         << ": likelihood=" << g[e].likelihood << " below threshold=" << threshold << endl;
    remove_edge(e, g);
  }
}


std::vector<NodeSets> PersonMatcher::build_partitions(Nodes nodes) const
{
  //  cout << "Input: [";
  //  for (const auto& n : nodes)
  //  {
  //    cout << n << ", ";
  //  }
  //  cout << "]" << endl;

  Node head = nodes.back();
  auto head_type =
      g[head].type;  // note that the node descriptors passed to build_partition are global

  nodes.pop_back();

  if (nodes.size() == 0)
  {
    return { { { { head }, head_type } } };
  }

  // 1. recursively get partitions for a subset of nodes
  auto partitions = build_partitions(nodes);

  std::vector<NodeSets> updated_partitions;

  // 2. add 'head' node to existing partitions if valid
  for (const NodeSets& partition : partitions)
  {
    // cout << "Input partition:" << endl;

    // 2.1 add 'head' to each subgraphs, one after the other
    for (size_t idx = 0; idx < partition.size(); idx++)
    {
      auto feature_mask = partition[idx].second;

      // a valid partition must not contain the same feature twice
      if (!(feature_mask & head_type))
      {
        // copy the current partition
        NodeSets new_partition(partition);

        new_partition[idx].first.push_back(head);
        new_partition[idx].second = feature_mask | head_type;

        updated_partitions.push_back(new_partition);
      }
    }


    // 2.2 add a last partition made of [ {head}, other partitions... ]
    NodeSets new_partition(partition);
    new_partition.push_back({ { head }, head_type });
    updated_partitions.push_back(new_partition);
  }

  //  cout << "Output:" << endl;
  //  for (const auto& partition : updated_partitions)
  //  {
  //    cout << "Partition:" << endl;
  //    for (const auto& nodes : partition)
  //    {
  //      cout << "[";
  //      for (const auto& n : nodes.first)
  //      {
  //        cout << n << ", ";
  //      }
  //      cout << "]" << endl;
  //    }
  //  }


  return updated_partitions;
}

std::vector<Subgraphs> PersonMatcher::get_partitions(Graph& graph)
{
  Nodes nodes;
  for (const auto& n : boost::make_iterator_range(vertices(graph)))
  {
    assert(graph[n].valid);
    nodes.push_back(graph.local_to_global(n));
  }

  // cout << "----------BUILD PARTITIONS-------------" << endl;
  auto raw_partitions = build_partitions(nodes);
  // cout << "---------------------------------------" << endl;
  cout << "Got " << raw_partitions.size() << " raw partitions" << endl;
  // for (const auto& p : raw_partitions)
  //{
  //  cout << "- " << p.size() << endl;
  //}

  vector<Subgraphs> viable_partitions;

  std::vector<int> component(num_vertices(graph));  // component map to call boost connected_components

  // filter the partitions to only keep the *connected* ones
  for (const auto& nodesets : raw_partitions)
  {
    Subgraphs partition;
    bool connected = true;
    for (const auto& nodeset : nodesets)
    {
      auto subgraph = g.create_subgraph(nodeset.first.begin(), nodeset.first.end());

      auto nb_components = connected_components(
          subgraph, &component[0]);  // (note that we do not need the component map `component`)

      if (nb_components > 1)
      {
        connected = false;
        break;
      }
      partition.push_back({ subgraph, nodeset.second });
    }

    if (connected)
    {
      viable_partitions.push_back(partition);
    }
  }


  cout << "Got " << viable_partitions.size() << " connected partitions" << endl;

  // for (const auto& p : viable_partitions)
  //{
  //  cout << "- " << p.size() << endl;
  //  // print_partition(p);
  //  // cout << endl;
  //}
  return viable_partitions;
}

void PersonMatcher::add_anonymous_person(Subgraph& association)
{
  auto& subgraph = association.first;
  assert(&subgraph.parent() == &g);
  auto feature_mask = association.second;

  // is there already a person in this association?
  if (feature_mask & FeatureType::person)
  {
    return;
  }

  // create a new global node for an anonymous person
  auto anon = add_vertex(g);

  if (RANDOM_ANONYMOUS_NAME)
  {
    g[anon].name = hri::ANONYMOUS + generate_random_id();
  }
  else
  {
    g[anon].name = "anon" + to_string(incremental_anon_id);
    g[anon].anonymous_id = incremental_anon_id;
    incremental_anon_id++;
  }

  g[anon].type = FeatureType::person;
  g[anon].anonymous = true;

  cout << "Adding anonymous person " << g[anon].name << endl;

  // add the node to this association -- the vertex descriptor inside the
  // subgraph might be different than in the global graph -> sub_node != anon
  auto sub_node = add_vertex(anon, subgraph);

  for (const auto& n : boost::make_iterator_range(vertices(subgraph)))
  {
    if (n != sub_node)
    {
      // adding an edge to a subgraph also adds it to the parent
      auto e = add_edge(sub_node, n, subgraph).first;
      subgraph[e].likelihood = threshold;
      subgraph[e].weight = likelihood2weight(threshold);
      subgraph[e].log_likelihood = log_likelihood(threshold);
      subgraph[e].computed = true;
    }
  }
}

void PersonMatcher::clear_anonymous_persons()
{
  vector<Node> anonyms;

  for (const auto& n : boost::make_iterator_range(vertices(g)))
  {
    if (g[n].anonymous == true)
    {
      anonyms.push_back(n);
    }
  }

  if (!RANDOM_ANONYMOUS_NAME)
  {
    // sort by anonymous_id, so that we decrease step-by-step for as long as possible the
    // incremental_anon_id value below
    sort(anonyms.begin(), anonyms.end(), [this](const Node& a, const Node& b) -> bool {
      return g[a].anonymous_id > g[b].anonymous_id;
    });
  }

  for (const auto& n : anonyms)
  {
    // if this anonymous person was the last added, make that ID reusable for
    // the next anonymous person
    if (!RANDOM_ANONYMOUS_NAME && g[n].anonymous_id == incremental_anon_id - 1)
    {
      incremental_anon_id--;
    }
    clear_vertex(n, g);
    g[n].valid = false;
    cout << "Clear anonymous person " << g[n].name << endl;
  }
}

void PersonMatcher::fully_connect_persons(Subgraph& association)
{
  auto& graph = association.first;
  auto feature_mask = association.second;

  // is there a person in this association?
  if (!(feature_mask & FeatureType::person))
  {
    return;
  }

  // required to operate on a *copy* of graph, otherwise operations like
  // remove_edge do not work (probably because the edge might belong to other
  // subgraphs)
  auto graph_copy = clean_graph_copy(graph);

  // 0. find the 'person' node
  Node person;
  for (const auto& n : boost::make_iterator_range(vertices(graph_copy)))
  {
    if (graph_copy[n].type == FeatureType::person)
    {
      person = n;
      break;
    }
  }

  Node person_orig = get_node_by_name(graph_copy[person].name, graph);

  // 1. remove computed edges
  vector<std::tuple<Node, Node, EdgeProps>> computed_edges;
  for (const auto& e : boost::make_iterator_range(edges(graph_copy)))
  {
    if (graph_copy[e].computed)
    {
      computed_edges.push_back(
          make_tuple(source(e, graph_copy), target(e, graph_copy), graph_copy[e]));
    }
  }
  for (auto e : computed_edges)
  {
    Node s, t;
    EdgeProps ep;
    std::tie(s, t, ep) = e;
    remove_edge(s, t, graph_copy);
  }

  // 2. compute shortest paths

  // vector for storing distance property
  vector<float> d(num_vertices(graph_copy));

  dijkstra_shortest_paths(
      graph_copy, person,
      distance_map(&d[0]).weight_map(get(&EdgeProps::log_likelihood, graph_copy)));

  // 3. create computed edges for features not directly connected to the person

  vector<std::tuple<Node, Node, EdgeProps>> edges_to_update;

  for (const auto& n : boost::make_iterator_range(vertices(graph_copy)))
  {
    if (n == person)
    {
      continue;
    }

    const auto& e = edge(n, person, graph_copy);

    // if already connected, continue
    if (e.second)
    {
      // cout << graph_copy[person].name << " and " << graph_copy[n].name
      //     << " already connected" << endl;
      continue;
    }

    auto dist = d[n];

    // no path
    if (dist == numeric_limits<decltype(EdgeProps::log_likelihood)>::max())
    {
      // cout << "No path between " << graph_copy[person].name << " and "
      //     << graph_copy[n].name << endl;
      continue;
    }
    float likelihood = inv_log_likelihood(dist);

    if (likelihood < threshold)
    {
      continue;
    }

    auto out = add_edge(n, person, graph_copy);

    if (out.second)
    {
      Edge new_e = out.first;

      // cout << "adding new edge between " << graph_copy[person].name << " and "
      //     << graph_copy[n].name << " with dist=" << dist << endl;
      graph_copy[new_e].likelihood = likelihood;
      graph_copy[new_e].weight = likelihood2weight(likelihood);
      graph_copy[new_e].log_likelihood = dist;
      graph_copy[new_e].computed = true;

      edges_to_update.push_back({ n, person, graph_copy[new_e] });
    }
    else
    {
      cout << "[EE] failed to add computed edge between " << graph_copy[person].name
           << " and " << graph_copy[n].name << ": edge already exist!" << endl;
      assert(false);
    }
  }

  // 4. bring back the previously removed edges, *if they have not been updated*
  for (const auto& e : computed_edges)
  {
    auto ok = add_edge(get<0>(e), get<1>(e),
                       graph_copy);  // parallel edges are not permitted -> add_edge will
                                     // only insert the edge if not already present
    // ok.second = true: the edge was added -> value not previously updated
    // ok.second = false: the edge was already there: value updated!
    if (ok.second)
    {
      graph_copy[ok.first] = get<2>(e);
      edges_to_update.push_back(e);
    }
  }

  // finally, perform the updates on the original graph (eg, not the copy)
  for (const auto& e : edges_to_update)
  {
    Node s, t;
    EdgeProps ep;
    std::tie(s, t, ep) = e;

    Node s_orig = get_node_by_name(graph_copy[s].name, graph);
    Node t_orig = get_node_by_name(graph_copy[t].name, graph);

    // this either create a new edge (out.second = true), or update the value
    // of an existing edge (out.second = false).  important: adding/updating an
    // edge to the subgraph also adds it to the parent graph
    auto out = add_edge(s_orig, t_orig, graph);
    graph[out.first] = ep;
  }
}

float PersonMatcher::partition_affinity(const Subgraphs& partition) const
{
  float affinity = 0.;

  for (const Subgraph& association : partition)
  {
    auto graph = clean_graph_copy(association.first);

    // set the edge_weight property, as I've not been able to use the EdgeProps::weight
    // property directly in kruskal_minimum_spanning_tree
    for (const auto& e : boost::make_iterator_range(edges(graph)))
    {
      put(edge_weight_t(), graph, e, graph[e].weight);
    }

    std::vector<Edge> spanning_tree;

    kruskal_minimum_spanning_tree(graph, std::back_inserter(spanning_tree));
    //, weight_map(get(&EdgeProps::weight, graph)));

    for (const auto& e : spanning_tree)
    {
      affinity += graph[e].likelihood;
    }
  }

  return affinity;
}

Graph PersonMatcher::clean_graph_copy(const Graph& graph, bool remove_anonymous) const
{
  Graph res;

  for (const auto& n : boost::make_iterator_range(vertices(graph)))
  {
    if (graph[n].valid && !(remove_anonymous && graph[n].anonymous))
    {
      Node n_copy = add_vertex(res);
      res[n_copy] = graph[n];
    }
    // else
    //{
    //  cout << "removing deleted node " << g[n].name << endl;
    //}
  }

  for (const auto& e : boost::make_iterator_range(edges(graph)))
  {
    Node s = get_node_by_name(graph[source(e, graph)].name, res);
    Node t = get_node_by_name(graph[target(e, graph)].name, res);

    if (s != INEXISTANT_VERTEX && t != INEXISTANT_VERTEX)
    {
      Edge e_copy = add_edge(s, t, res).first;
      res[e_copy] = graph[e];
    }
  }

  return res;
}

Subgraphs PersonMatcher::compute_associations()
{
  if (num_vertices(g) == 0)
    return {};

  clear_anonymous_persons();
  g = clean_graph_copy(g);

  // !! mutates this->g
  prune_unlikely_connections();

  Subgraphs complete_partition;

  ////////// Get connected components
  std::vector<int> component(num_vertices(g));  // component map



  auto nb_components = boost::connected_components(active_graph, &component[0]);

  // create one subgraph per connected component
  vector<Graph> connected_components;

  for (size_t i = 0; i < nb_components; i++)
  {
    auto subG = g.create_subgraph();
    for (size_t j = 0; j < num_vertices(g); j++)
    {
      if (component[j] == i)
      {
        add_vertex(j, subG);
      }
    }
    if (num_vertices(subG) > 0)
    {  // size might be zero if the component only includes invalid (ie, removed) nodes
      connected_components.push_back(subG);
    }
  }

  for (auto subgraph : connected_components)
  {
    // cout << "Processing connected component..." << endl;
    auto partitions = get_partitions(subgraph);


    // for (size_t i = 0; i < partitions.size(); i++)
    //{
    //  cout << "Partition " << (i + 1) << ":" << endl;
    //  print_partition(partitions[i]);
    //}

    // look for the partitions with the least component
    // -- the maximum possible number of partitions being the number of nodes
    // in the graph
    size_t min_len = num_vertices(g);

    for (auto p : partitions)
    {
      if (p.size() < min_len)
      {
        min_len = p.size();
      }
    }

    // only keep the most compact partitions (ie, partitions with the minimal number of subgraph)
    vector<Subgraphs> compact_partitions;

    std::copy_if(partitions.begin(), partitions.end(), std::back_inserter(compact_partitions),
                 [min_len](const Subgraphs& elem) { return elem.size() == min_len; });

    //    for (const auto& p : compact_partitions)
    //    {
    //      for (const auto& sg : p)
    //      {
    //        const auto& graph = sg.first;
    //        for (const auto& e : boost::make_iterator_range(edges(graph)))
    //        {
    //          cout << "Checking edge " << graph[source(e, graph)].name << " -- "
    //               << graph[e].likelihood << " (" << graph[e].weight << ") -- "
    //               << graph[target(e, graph)].name << endl;
    //          if (!compare_likelihoods(graph[e].likelihood,
    //                                   weight2likelihood(get(boost::edge_weight_t(), graph, e))))
    //          {
    //            EdgeProps ep = graph[e];
    //            cout << graph[e].likelihood
    //                 << " != " << weight2likelihood(get(boost::edge_weight_t(), graph, e)) << endl;
    //            assert(false);
    //          }
    //        }
    //      }
    //    }

    std::sort(compact_partitions.begin(), compact_partitions.end(),
              [this](const Subgraphs& e1, const Subgraphs& e2) {
                return partition_affinity(e1) > partition_affinity(e2);
              });

    cout << compact_partitions.size() << " 'minimum size' partitions" << endl;

    //    for (size_t i = 0; i < std::min(static_cast<int>(compact_partitions.size()), 3); i++)
    //    {
    //      cout << "Candidate partition " << i + 1 << endl;
    //
    //      cout << "- affinity: " << partition_affinity(compact_partitions[i]) << endl;
    //      print_partition(compact_partitions[i]);
    //    }
    //
    //    cout << "\n=> Best affinity of compact partitions:"
    //         << partition_affinity(compact_partitions[0]) << endl;

    complete_partition.insert(complete_partition.end(), compact_partitions[0].begin(),
                              compact_partitions[0].end());
  }

  // cout << "Nb of generated subgraphs before selection: " << g.num_children() << endl;
  // cout << "[Before anonymous persons + full connections]" << endl;
  // print_partition(complete_partition);

  for (auto& subgraph : complete_partition)
  {
    add_anonymous_person(subgraph);
    fully_connect_persons(subgraph);
  }

  // cout << "[After anonymous persons + full connections]" << endl;
  // print_partition(complete_partition);

  return complete_partition;
}

void PersonMatcher::print_partition(const Subgraphs& partition) const
{
  size_t idx = 0;

  for (const auto& association : partition)
  {
    idx += 1;
    auto subgraph = association.first;

    cout << "  Association " << idx << endl;

    cout << "    Nodes: [";
    for (const auto& n : boost::make_iterator_range(vertices(subgraph)))
    {
      cout << subgraph[n].name << ", ";
    }
    cout << "]" << endl;

    for (const auto& e : boost::make_iterator_range(edges(subgraph)))
    {
      cout << "    " << subgraph[source(e, subgraph)].name << " <-- " << subgraph[e].likelihood
           << " --> " << subgraph[target(e, subgraph)].name << endl;
    }

    cout << endl;
  }
}

void PersonMatcher::update(Relations relations)
{
  ID id1, id2;
  FeatureType type1, type2;
  float likelihood;

  for (auto rel : relations)
  {
    std::tie(id1, type1, id2, type2, likelihood) = rel;

    Node v1 = get_vertex(g, id1);

    if (v1 == INEXISTANT_VERTEX)
    {
      v1 = add_vertex(g);
      g[v1].name = id1;
      g[v1].type = type1;
      id_types[type1].insert(id1);
    }

    Node v2 = get_vertex(g, id2);

    if (v2 == INEXISTANT_VERTEX)
    {
      v2 = add_vertex(g);
      g[v2].name = id2;
      g[v2].type = type2;
      id_types[type2].insert(id2);
    }

    if (likelihood <= 0.0)
    {
      remove_edge(v1, v2, g);
      continue;
    }

    Edge e;
    bool ok;

    std::tie(e, ok) = add_edge(v1, v2, g);
    // (note: e points either to a new edge, or to the existing edge if already present)

    g[e].likelihood = likelihood;
    g[e].weight = likelihood2weight(likelihood);
    g[e].log_likelihood = log_likelihood(likelihood);
    g[e].computed = false;
  }
}

std::set<ID> PersonMatcher::erase(ID id)
{
  set<ID> removed_persons;

  Node v = get_vertex(g, id);
  if (v == INEXISTANT_VERTEX)
  {
    return removed_persons;
  }

  clear_vertex(v, g);
  g[v].valid = false;

  erase_id(id);

  if (id_types[person].count(id))
  {
    removed_persons.insert(id);
  }

  //  auto orphaned_persons = clear_orphaned_persons();

  //  removed_persons.insert(orphaned_persons.begin(), orphaned_persons.end());

  return removed_persons;
}

// std::set<ID> PersonMatcher::clear_orphaned_persons()
//{
//  // remove all orphan vertices and return removed persons
//  set<ID> removed_persons;
//
//  Graph::vertex_iterator v, vend;
//
//  while (true)
//  {
//    bool found_orphan = false;
//    for (boost::tie(v, vend) = vertices(g); v != vend; ++v)
//    {
//      if (out_degree(*v, g) == 0)
//      {
//        found_orphan = true;
//
//        ID id = get(&NodeProps::name, g, *v);
//        ROS_INFO_STREAM(" ---> clearing orphan " << id);
//
//        // is it a person node? if so, store it
//        if (id_types[person].count(id))
//        {
//          removed_persons.insert(id);
//          erase_id(id);
//
//          remove_vertex(*v, g);
//        }
//
//
//
//        // we break and restart the iteration over all the vertices of the graph
//        // because when a vertex is removed, the graph reindex all vertices -- if we
//        // were to first build a list of all vertices to delete, that list would be
//        // invalid after the first removed vertex.
//        break;
//      }
//    }
//
//    // continue until no more orphans
//    if (!found_orphan)
//    {
//      break;
//    }
//  }
//
//  return removed_persons;
//}
//
void PersonMatcher::reset()
{
  g = Graph();
}

map<ID, map<FeatureType, ID>> PersonMatcher::get_all_associations()
{
  // TODO: we must ensure that g is *not mutated* until the end of the method.
  // Might be safer to make a copy of g at the start, and run
  // compute_associations on that copy.
  auto associations = compute_associations();

  map<ID, map<FeatureType, ID>> res;

  for (const auto& association : associations)
  {
    Node person;

    map<FeatureType, ID> features;
    auto graph = association.first;

    for (const auto& n : boost::make_iterator_range(vertices(graph)))
    {
      if (graph[n].type == FeatureType::person)
      {
        person = n;
      }
      else
      {
        features[graph[n].type] = graph[n].name;
      }
    }
    res[graph[person].name] = features;
  }

  return res;
}

string PersonMatcher::get_graphviz() const
{
  stringstream ss;
  write_graphviz(ss, g, node_label_writer(g, id_types), weight_label_writer(g));

  return ss.str();
}

/** warning: if the same ID is used for 2 different features (eg a face and a
 * body), both will be deleted!
 */
void PersonMatcher::erase_id(ID id)
{
  for (auto type : { face, body, voice, person })
  {
    if (id_types[type].count(id))
    {
      id_types[type].erase(id);
    }
  }
}

