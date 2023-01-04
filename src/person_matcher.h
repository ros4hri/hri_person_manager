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

#ifndef HRI_PERSON_MATCHER_H
#define HRI_PERSON_MATCHER_H

#include <boost/graph/properties.hpp>
#include <boost/graph/subgraph.hpp>
#include <limits>
#include <map>
#include <vector>
#include <tuple>
#include <hri/base.h>

#include <boost/graph/adjacency_list.hpp>
#include <boost/pending/property.hpp>
#include <boost/graph/filtered_graph.hpp>

inline float likelihood2weight(float v)
{
  return 1. - v;
}

inline float weight2likelihood(float v)
{
  return 1. - v;
}

inline float log_likelihood(float v)
{
  return log(1. / v);
}


inline float inv_log_likelihood(float v)
{
  return 1. / exp(v);
}



struct NodeProps
{
  hri::ID name;
  hri::FeatureType type;
  bool anonymous = false;  // true for anonymous persons
  int anonymous_id;        // only used for tracking anon id for unit-testing
  bool valid = true;       // boost::subgraph does not support *removing* nodes -> we mark
                           // removed nodes as invalid to ignore them
};

struct EdgeProps
{
  float likelihood;
  float weight;
  float log_likelihood;
  // false if this edge comes from external 'candidate_matches', true if
  // the edge was added by the algorithm (eg, direct connection between a person and features)
  bool computed = false;
};

template <class T>
bool compare_likelihoods(T l1, T l2)
{
  return std::fabs(l1 - l2) <= std::numeric_limits<T>::epsilon();
}

// typedef boost::property<boost::edge_weight_t, float> EdgeWeightProperty;

// use boost::setS to forbid parallel edges
typedef boost::subgraph<boost::adjacency_list<
    boost::setS, boost::vecS, boost::undirectedS, NodeProps,
    boost::property<boost::edge_index_t, int, boost::property<boost::edge_weight_t, float, EdgeProps>>>>
    Graph;

// sub-graphs are Graph themselves, created via Graph.create_subgraph
// We attach an 'unsigned int' bitmask `feature_mask` to keep track of the features (face,
// body, voice, person...) already present in that subgraph.
typedef std::pair<Graph, unsigned int> Subgraph;
typedef std::vector<Subgraph> Subgraphs;


typedef boost::graph_traits<Graph>::vertex_descriptor Node;
const Node INEXISTANT_VERTEX(-1);

typedef std::vector<Node> Nodes;

// NodeSets are similar to Subgraph, but they only contain nodes, and are not
// explicitely tied to a graph. This is used by eg
// PersonMatcher::build_partitions to create partitions without mutating the
// internal graph (which would be the case if using subgraphs and
// create_subgraph)
typedef std::pair<Nodes, unsigned int> NodeSet;
typedef std::vector<NodeSet> NodeSets;


typedef boost::graph_traits<Graph>::edge_descriptor Edge;
typedef boost::graph_traits<Graph>::edge_iterator Edges;

typedef std::pair<hri::ID, hri::FeatureType> Feature;
typedef std::vector<std::tuple<hri::ID, hri::FeatureType, hri::ID, hri::FeatureType, float>> Relations;

/** small helper function to get a node by its name.
 *
 * Returns INEXISTANT_VERTEX if the name is not found.
 */
inline Node get_node_by_name(const std::string& name, const Graph& G)
{
  for (const auto& n : boost::make_iterator_range(vertices(G)))
  {
    if (G[n].valid && G[n].name == name)
    {
      return n;
    }
  }
  return INEXISTANT_VERTEX;
}

class PersonMatcher
{
public:
  PersonMatcher(float likelihood_threshold = 0.5, bool random_anonymous_name = true);

  /** sets the likelihood threshold to consider a feature (body, face, voice)
   * to belong to a person.
   */
  void set_threshold(float likelihood_threshold)
  {
    threshold = likelihood_threshold;
  }

  /** updates the probabilistic relations between one or several features.
   *
   * For instance:
   *
   * ```cpp
   * PersonMatcher model;
   *
   * model.update({ { "f1", face, "b1", body, 0.7 },
   *                { "f1", face, "b2", body, 0.6 },
   *                { "p1", person, "f1", face, 0.9 } });
   * ```
   *
   * Note: calling PersonMatcher::update has no side-effect beyond updating the
   * internal relation graph. No update of the most likely associations is
   * performed until PersonMatcher::get_all_associations is called.
   *
   * As such, there is not difference between calling PersonMatcher::update
   * once with several updates in a single vector, or calling
   * PersonMatcher::update several times with eg a single relation, as long as
   * get_all_associations is not called inbetween.
   */
  void update(Relations relations);

  /** deletes a given ID (be it a person, a face, a body or a voice) from
   * the probabilistic graph.
   */
  void erase(hri::ID id);

  /** clear the whole probabilistic graph.
   */
  void reset();


  /** returns a map with all the currently known persons, with their
   * most likely associations to faces/bodies/voices.
   */
  std::map<hri::ID, std::map<hri::FeatureType, hri::ID>> get_all_associations();

  /** returns the most likely associations for a given person.
   *
   * If the person does not exist, throw an out_of_range exception.
   *
   * Note: if you need to query more than one person, it is always more
   * efficient to use PersonMatcher::get_all_associations instead.
   */
  std::map<hri::FeatureType, hri::ID> get_association(hri::ID id)
  {
    return get_all_associations().at(id);
  }

  /** returns the current likelihood graph in dot format
   */
  std::string get_graphviz() const;

  /** !! for testing/debugging purposes: expose the raw subgraphs computed
   * by the algorithms.
   *
   * For production, always use PersonMatcher::get_all_associations instead.
   */
  Subgraphs get_raw_associations()
  {
    return compute_associations();
  }

  /** !! for testing/debugging purposes: returns a copy of the internal
   * probability graph, optionally removing anonymous persons.
   *
   * Note: contrary to PersonMatcher::clear_anonymous_persons, the index of the
   * next available anonymous ID is *not* updated while removing anonymous
   * nodes.
   *
   */
  Graph get_internal_graph(bool remove_anonymous = false) const
  {
    return clean_graph_copy(g, remove_anonymous);
  }


private:
  Graph g;

  /** creates a copy of the internal graph g with no subgraph and only valid nodes (ie,
   * non-removed nodes). Optionally, also remove anonymous nodes.
   */
  Graph clean_graph_copy(const Graph& graph, bool remove_anonymous = false) const;

  // used to create FilteredGraph with only valid nodes
  struct ValidNodePredicate
  {  // both edge and vertex
    bool operator()(Graph::edge_descriptor) const
    {
      return true;
    }  // accept all edges
    bool operator()(Graph::vertex_descriptor vd) const
    {
      return (*g)[vd].valid;
    }
    Graph* g;
  } valid_nodes_predicate{ &g };

  using ActiveGraph = boost::filtered_graph<Graph, ValidNodePredicate, ValidNodePredicate>;

  ActiveGraph active_graph{ g, valid_nodes_predicate, valid_nodes_predicate };


  /** 'Main' algorithm: compute the most likely associations between persons
   * and their body parts (face, voice, body...), and return one
   * boost::subgraph per association.
   *
   * If necessary, creates and add anonymous persons to associations lacking a
   * person.
   *
   * Note: PersonMatcher::compute_associations *mutates* the internal relations
   * graph. Specifically, after calling the method:
   * - all edges with a likelihood smaller than 'threshold' are removed
   * - new direct edges might be added between a person and a feature if these
   *   two belong to the same association and have an (indirect) likelihood
   *   above threshold.
   * - new anonymous persons might be added to the graph
   * - (potentially many) subgraphs corresponding to all possible associations
   *   (valid or not are created.
   *
   * See also:
   * - PersonMatcher::clean_graph_copy returns a 'clean' copy (ie, with all invalid nodes
   * removed and no subgraphs of the internal graph. PersonMatcher::compute_associations
   * calls it at the very start.
   * - PersonMatcher::clear_anonymous_persons to remove anonymous persons that
   * compute_associations might have created.
   */
  Subgraphs compute_associations();

  /** Deletes all relationships between nodes whose likelihood is below
   * 'threshold'.
   *
   * This method *does mutate* Graph g.
   */
  void prune_unlikely_connections();

  bool random_anonymous_name;

  /**
   * If the provided association does not include a person, create
   * an anonymous person.
   *
   * Note: this method *does mutate* Graph g if an anonymous person needs to be
   * added.
   */
  void add_anonymous_person(Subgraph& association);

  void clear_anonymous_persons();

  // Mapping between features and previously used anonymous person ids.
  // Used to reuse as much as possible the same anonymous IDs for the same
  // features
  std::map<std::string, std::string> anonymous_ids_map;

  /** this methods returns a anonymous person ID for an association lacking a person.
   *
   * It will:
   * - check (in alphabetical order) if any of the features of the association has already
   * been associated to an anonymous person in the past
   *   - if so:
   *        - deletes all entries in anonymous_ids_map with that id, to ensure no other
   *          association might end up reusing the same id
   *        - assigns the id to all features in this association
   *        - returns the id
   *   - if not:
   *        - generates a new ID, assign it to each features, and returns it.
   */
  std::string set_get_anonymous_id(std::vector<std::string>);

  int incremental_anon_id = 1;

  /** Add direct links between a person and its features, iff:
   *  - they are not already directly connected
   *  - the likelihood of the connection (computed as the product of
   *    likelihoods along the shortest path) is above the set likelihood
   *    `threshold`.
   *
   * If the provided association does not include a person, does nothing.
   *
   * Note: this method *does mutate* Graph g with added computed edges   *
   *
   * Algorithm:
   * ----------
   *
   * to compute the direct edges between the person and the features indirectly
   * connected to it:
   *  0. we find the person node
   *  1. we *temporarily remove* previously computed edges to the person,
   *  2. we compute the shortest path between the person and each of the features
   *  using the *log* of the likelihoods, so that the path lenght is the
   *  resulting probability of the association (product of the likelihoods)
   *  3. if the person and the feature are not directly connected, we create a new
   *  'computed' edge with the corresponding likelihood.
   *  4. we bring back the previously removed edges, if they have not been
   *  computed at step 3 (eg, for features that were disconnected from the rest
   *  of the graph when removing the edge)
   *
   */
  void fully_connect_persons(Subgraph& association);

  /** returns all possible (valid) partition of the provided graph.
   *
   * Mutates the provided boost::subgraph (and all its ancestors) to add all
   * the possible partitions.
   * It is highly recommended to regularly remove the subgraph (by eg calling
   * PersonMatcher::reset_subgraphs or PersonMatcher::clean_graph_copy),
   * otherwise the number of subgraph might explode.
   */
  std::vector<Subgraphs> get_partitions(Graph&);

  /** deletes all the subgraphs of a given graph
   */
  void reset_subgraphs(Graph& g)
  {
    // m_children is an undocumented boost property -- no officially documented way of
    // deleting a subgraph...
    for (auto i : g.m_children)
    {
      delete i;
    }
    g.m_children.clear();
  }

  /** Helper function recursively called by PersonMatcher::get_partitions
   */
  std::vector<NodeSets> build_partitions(Nodes) const;

  /** The graph partition 'affinity' is the sum of the likelihood of associations accross
   *  all the partition's subgraphs.
   *
   *  It is computed by:
   *
   *  - for each each subgraph:
   *      - assigning weights to each edges by taking the opposite of the likelihoods of associations
   *      - finding the minimum spanning tree
   *      - summing the weights of every edge in the subgraph
   *  - then, summing the weights over all subgraphs
   *  - taking the opposite of the result to get the final affinitiy
   */
  float partition_affinity(const Subgraphs& partition) const;

  void print_partition(const Subgraphs& partition) const;

  float threshold;
};

#endif  // HRI_PERSON_MATCHER_H
