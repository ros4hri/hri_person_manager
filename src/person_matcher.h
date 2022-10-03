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

#include <map>
#include <vector>
#include <tuple>
#include <hri/base.h>

#include <boost/graph/adjacency_list.hpp>
#include <boost/pending/property.hpp>


struct VertexProps
{
  hri::ID name;
};
typedef boost::property<boost::edge_weight_t, float> EdgeWeightProperty;
// use boost::setS to forbid parallel edges
typedef boost::adjacency_list<boost::setS, boost::vecS, boost::undirectedS, VertexProps, EdgeWeightProperty> Graph;
typedef boost::graph_traits<Graph>::vertex_descriptor Vertex;
const Vertex INEXISTANT_VERTEX(-1);
typedef boost::graph_traits<Graph>::edge_descriptor Edge;

typedef std::pair<hri::ID, hri::FeatureType> Feature;
typedef std::vector<std::tuple<hri::ID, hri::FeatureType, hri::ID, hri::FeatureType, float>> Relations;

class PersonMatcher
{
public:
  PersonMatcher(float likelihood_threshold = 0.5);

  /** sets the likelihood threshold to consider a feature (body, face, voice)
   * to belong to a person.
   */
  void set_threshold(float likelihood_threshold);

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
   */
  void update(Relations relations);

  /** deletes a given ID (be it a person, a face, a body or a voice) from
   * the probabilistic graph.
   *
   * Removes as well orphan nodes that might result from removing the given ID.
   *
   * The set of all *person* IDs removed during the process is returned.
   */
  std::set<hri::ID> erase(hri::ID id);


  /** removes orphaned persons in the graph, and returns the list of
   * removed *persons*
   */
  std::set<hri::ID> clear_orphaned_persons();

  /** clear the whole probabilistic graph.
   */
  void reset();


  /** returns the most likely association of a person to
   * its face/body/voice.
   */
  std::map<hri::FeatureType, hri::ID> get_association(hri::ID) const;

  /** returns a pair with:
   *  -a map with all the currently known persons, with their
   * most likely associations to faces/bodies/voices.
   * - a set of all the remaining features that are not associated to a person
   */
  std::pair<std::map<hri::ID, std::map<hri::FeatureType, hri::ID>>, std::set<hri::ID>> get_all_associations() const;

  /** returns the current likelihood graph in dot format
   */
  std::string get_graphviz() const;

private:
  Graph g;

  void erase_id(hri::ID id);

  std::map<hri::FeatureType, std::set<hri::ID>> id_types;

  float threshold;
};

#endif  // HRI_PERSON_MATCHER_H
