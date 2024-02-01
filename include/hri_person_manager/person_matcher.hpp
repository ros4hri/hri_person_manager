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


#ifndef HRI_PERSON_MANAGER__PERSON_MATCHER_HPP_
#define HRI_PERSON_MANAGER__PERSON_MATCHER_HPP_

#include <cmath>
#include <limits>
#include <map>
#include <string>
#include <tuple>
#include <utility>
#include <vector>

#include "boost/graph/adjacency_list.hpp"
#include "boost/graph/filtered_graph.hpp"
#include "boost/graph/properties.hpp"
#include "boost/graph/subgraph.hpp"
#include "boost/pending/property.hpp"
#include "boost/range/iterator_range_core.hpp"
#include "hri/types.hpp"

namespace hri_person_manager
{

inline float likelihood2weight(float v) {return 1. - v;}
inline float weight2likelihood(float v) {return 1. - v;}
inline float log_likelihood(float v) {return log(1. / v);}
inline float inv_log_likelihood(float v) {return 1. / exp(v);}

struct NodeProps
{
  hri::ID name;
  hri::FeatureType type;
  bool anonymous = false;  // true for anonymous persons
  int anonymous_id;        // only used for tracking anon id for unit-testing
  bool valid = true;       // boost::subgraph does not support *removing* nodes -> we mark
                           // removed nodes as invalid to ignore them
  unsigned int association_id = 1;  // only used for the graphviz output
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

// use boost::setS to forbid parallel edges
using Graph =
  boost::subgraph<
  boost::adjacency_list<
    boost::setS, boost::vecS, boost::undirectedS, NodeProps, boost::property<
      boost::edge_index_t, int, boost::property<boost::edge_weight_t, float, EdgeProps>>>>;

// sub-graphs are Graph themselves, created via Graph.create_subgraph
// We attach an 'unsigned int' bitmask `feature_mask` to keep track of the features (face,
// body, voice, person...) already present in that subgraph.
using Subgraph = std::pair<Graph, hri::FeatureType>;
using Subgraphs = std::vector<Subgraph>;

using Node = Graph::vertex_descriptor;
using Nodes = std::vector<Node>;
const Node kInexistantNode(-1);

// NodeSet are similar to Subgraph, but they only contain nodes, and are not
// explicitely tied to a graph. This is used by eg
// PersonMatcher::buildPartitions to create partitions without mutating the
// internal graph (which would be the case if using subgraphs and
// create_subgraph)
using NodeSet = std::pair<Nodes, hri::FeatureType>;
using NodeSets = std::vector<NodeSet>;

using Edge = Graph::edge_descriptor;
using Edges = Graph::edge_iterator;

using Feature = std::pair<hri::ID, hri::FeatureType>;
using Relations =
  std::vector<std::tuple<hri::ID, hri::FeatureType, hri::ID, hri::FeatureType, float>>;

/** small helper function to get a node by its name.
 *
 * Returns kInexistantNode if the name is not found.
 */
inline Node get_node_by_name(const std::string & name, const Graph & graph)
{
  for (const auto & n : boost::make_iterator_range(boost::vertices(graph))) {
    if (boost::get(&NodeProps::valid, graph, n) && boost::get(&NodeProps::name, graph, n) == name) {
      return n;
    }
  }
  return kInexistantNode;
}

class PersonMatcher
{
public:
  explicit PersonMatcher(float likelihood_threshold = 0.5, bool random_anonymous_name = true);

  /** sets the likelihood threshold to consider a feature (body, face, voice) to belong to a person.
   */
  void setThreshold(float likelihood_threshold)
  {
    threshold_ = likelihood_threshold;
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
   * performed until PersonMatcher::getAllAssociations is called.
   *
   * As such, there is not difference between calling PersonMatcher::update
   * once with several updates in a single vector, or calling
   * PersonMatcher::update several times with eg a single relation, as long as
   * getAllAssociations is not called inbetween.
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
  std::map<hri::ID, std::map<hri::FeatureType, hri::ID>> getAllAssociations();

  /** returns the most likely associations for a given person.
   *
   * If the person does not exist, throw an out_of_range exception.
   *
   * Note: if you need to query more than one person, it is always more
   * efficient to use PersonMatcher::getAllAssociations instead.
   */
  std::map<hri::FeatureType, hri::ID> getAssociation(hri::ID id)
  {
    return getAllAssociations().at(id);
  }

  /** returns the current likelihood graph in dot format
   */
  std::string getGraphviz() const;

  /** !! for testing/debugging purposes: expose the raw subgraphs computed
   * by the algorithms.
   *
   * For production, always use PersonMatcher::getAllAssociations instead.
   */
  Subgraphs getRawAssociations()
  {
    return computeAssociations();
  }

  /** !! for testing/debugging purposes: returns a copy of the internal
   * probability graph, optionally removing anonymous persons.
   *
   * Note: contrary to PersonMatcher::clearAnonymousPersons, the index of the
   * next available anonymous ID is *not* updated while removing anonymous
   * nodes.
   *
   */
  Graph getInternalGraph(bool remove_anonymous = false, bool remove_computed_edges = false) const
  {
    return cleanGraphCopy(g_, remove_anonymous, remove_computed_edges);
  }

  /** !! for testing/bedugging purposes only
   *
   * Cleans the internal graph, by removing all anonymous persons and computed edges.
   */
  void cleanGraph()
  {
    g_ = cleanGraphCopy(g_, true, true);
  }

private:
  // used to create FilteredGraph with only valid nodes
  struct ValidNodePredicate
  {  // both edge and vertex
    bool operator()(Graph::edge_descriptor) const {return true;}  // accept all edges
    bool operator()(Graph::vertex_descriptor vd) const {return (*g)[vd].valid;}
    Graph * g;
  } valid_nodes_predicate_{ & g_};

  using ActiveGraph = boost::filtered_graph<Graph, ValidNodePredicate, ValidNodePredicate>;

  /** creates a copy of the internal graph g_ with no subgraph and only valid nodes (ie,
   * non-removed nodes). Optionally, also remove anonymous nodes.
   */
  Graph cleanGraphCopy(
    const Graph & graph, bool remove_anonymous = false, bool remove_computed_edges = false) const;

  /** 'Main' algorithm: compute the most likely associations between persons
   * and their body parts (face, voice, body...), and return one
   * boost::subgraph per association.
   *
   * If necessary, creates and add anonymous persons to associations lacking a
   * person.
   *
   * Note: PersonMatcher::computeAssociations *mutates* the internal relations
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
   * - PersonMatcher::cleanGraphCopy returns a 'clean' copy (ie, with all invalid nodes
   * removed and no subgraphs of the internal graph. PersonMatcher::computeAssociations
   * calls it at the very start.
   * - PersonMatcher::clearAnonymousPersons to remove anonymous persons that
   * computeAssociations might have created.
   */
  Subgraphs computeAssociations();

  /** Deletes all relationships between nodes whose likelihood is below
   * 'threshold'.
   *
   * This method *does mutate* Graph g_.
   */
  void pruneUnlikelyConnections();

  /**
   * If the provided association does not include a person, create
   * an anonymous person.
   *
   * Note: this method *does mutate* Graph g_ if an anonymous person needs to be
   * added.
   */
  void addAnonymousPerson(Subgraph & association);

  void clearAnonymousPersons();

  /** this methods returns a anonymous person ID for an association lacking a person.
   *
   * It will:
   * - check (in alphabetical order) if any of the features of the association has already
   * been associated to an anonymous person in the past
   *   - if so:
   *        - deletes all entries in anonymous_ids_map_ with that id, to ensure no other
   *          association might end up reusing the same id
   *        - assigns the id to all features in this association
   *        - returns the id
   *   - if not:
   *        - generates a new ID, assign it to each features, and returns it.
   */
  std::string setGetAnonymousId(std::vector<std::string>);

  /** Add direct links between a person and its features, iff:
   *  - they are not already directly connected
   *  - the likelihood of the connection (computed as the product of
   *    likelihoods along the shortest path) is above the set likelihood
   *    `threshold`.
   *
   * If the provided association does not include a person, does nothing.
   *
   * Note: this method *does mutate* Graph g_ with added computed edges   *
   *
   * Algorithm:
   * ----------
   *
   * to compute the direct edges between the person and the features indirectly
   * connected to it:
   *  0. we find the person node
   *  1. we *temporarily remove* previously computed edges to the person,
   *  2. we compute the shortest path between the person and each of the features
   *  using the *log* of the likelihoods, so that the path length is the
   *  resulting probability of the association (product of the likelihoods)
   *  3. if the person and the feature are not directly connected, we create a new
   *  'computed' edge with the corresponding likelihood.
   *  4. we bring back the previously removed edges, if they have not been
   *  computed at step 3 (eg, for features that were disconnected from the rest
   *  of the graph when removing the edge)
   *
   */
  void fullyConnectPersons(Subgraph & association);

  /** returns all possible (valid) partition of the provided graph.
   *
   * Mutates the provided boost::subgraph (and all its ancestors) to add all
   * the possible partitions.
   * It is highly recommended to regularly remove the subgraph (by eg calling
   * PersonMatcher::cleanGraphCopy), otherwise the number of subgraph might
   * explode.
   */
  std::vector<Subgraphs> getPartitions(Graph &);

  /** Helper function recursively called by PersonMatcher::getPartitions
   */
  std::vector<NodeSets> buildPartitions(Nodes) const;

  /** The graph partition 'affinity' is the sum of the likelihood of associations accross
   *  all the partition's subgraphs.
   *
   *  It is computed by:
   *
   *  - for each each subgraph:
   *      - assigning weights to each edges,  (w = 1 - likelihood) of association
   *      - finding the minimum spanning tree
   *      - summing likelihoods along each of the spanning tree edges
   *  - return the total sum
   */
  float partitionAffinity(const Subgraphs & partition) const;

  void printPartition(const Subgraphs & partition) const;

  Graph g_;
  ActiveGraph active_graph_{g_, valid_nodes_predicate_, valid_nodes_predicate_};
  bool random_anonymous_name_;
  // Mapping between features and previously used anonymous person ids.
  // Used to reuse as much as possible the same anonymous IDs for the same
  // features
  std::map<std::string, std::string> anonymous_ids_map_;
  int incremental_anon_id_ = 1;
  float threshold_;
};

}  // namespace hri_person_manager

#endif  // HRI_PERSON_MANAGER__PERSON_MATCHER_HPP_
