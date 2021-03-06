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

#include <cmath>
#include <limits>
#include <tuple>

#include "person_matcher.h"


#include <boost/graph/detail/adjacency_list.hpp>
#include <boost/graph/graph_traits.hpp>
#include <boost/graph/dijkstra_shortest_paths.hpp>
#include "hri/base.h"


using namespace std;
using namespace hri;
using namespace boost;


PersonMatcher::PersonMatcher(float likelihood_threshold)
{
  threshold = log(1 / likelihood_threshold);

  id_vertex_map[person] = {};
  id_vertex_map[face] = {};
  id_vertex_map[body] = {};
  id_vertex_map[voice] = {};
}

void PersonMatcher::set_threshold(float likelihood_threshold)
{
  threshold = log(1 / likelihood_threshold);
}

void PersonMatcher::update(Relations relations)
{
  ID id1, id2;
  FeatureType type1, type2;
  float p;

  for (auto rel : relations)
  {
    std::tie(id1, type1, id2, type2, p) = rel;


    // cout << "==== " << id1 << " <-> " << id2 << " ====" << endl;
    // cout << "p=" << p << " -> w=" << weight << endl;

    auto& map1 = id_vertex_map[type1];
    auto& map2 = id_vertex_map[type2];

    Vertex v1, v2;

    if (map1.find(id1) == map1.end())
    {
      v1 = add_vertex(g);
      map1[id1] = v1;
    }
    else
    {
      v1 = map1[id1];
    }

    if (map2.find(id2) == map2.end())
    {
      v2 = add_vertex(g);
      map2[id2] = v2;
    }
    else
    {
      v2 = map2[id2];
    }

    if (p <= 0.0)
    {
      remove_edge(v1, v2, g);
      continue;
    }

    Edge edge;
    bool ok;


    float weight = log(1. / p);

    std::tie(edge, ok) = add_edge(v1, v2, weight, g);

    if (!ok)
    {
      boost::put(boost::edge_weight_t(), g, edge, weight);
    }
  }
}

void PersonMatcher::erase(ID id)
{
  for (auto type : { person, face, body, voice })
  {
    if (id_vertex_map[type].find(id) != id_vertex_map[type].end())
    {
      clear_vertex(id_vertex_map[type][id], g);
      remove_vertex(id_vertex_map[type][id], g);
      id_vertex_map[type].erase(id);
      return;
    }
  }
}

void PersonMatcher::reset()
{
  id_vertex_map[person] = {};
  id_vertex_map[face] = {};
  id_vertex_map[body] = {};
  id_vertex_map[voice] = {};

  g.clear();
}

map<FeatureType, ID> PersonMatcher::get_association(ID id) const
{
  //  boost::property_map<Graph, vertex_index_t>::type vertex_ids = get(vertex_index, g);
  //  boost::property_map<Graph, edge_weight_t>::type edge_probs = get(edge_weight, g);
  //
  //  cout << "vertices(g) = ";
  //
  //  for (auto vp : make_iterator_range(vertices(g)))
  //  {
  //    // cout << vertex_names[vp] << " ";
  //    cout << vp << " ";
  //  }
  //  cout << endl;
  //
  //  cout << "edges(g) = ";
  //
  //  for (auto ei : make_iterator_range(edges(g)))
  //  {
  //    cout << "(" << vertex_ids[source(ei, g)] << "," << vertex_ids[target(ei, g)]
  //         << ") => " << edge_probs[ei] << endl;
  //  }
  //  cout << endl;

  auto vertex = id_vertex_map.at(person).at(id);

  // vector for storing distance property
  vector<float> d(num_vertices(g));

  dijkstra_shortest_paths(g, vertex, distance_map(&d[0]));

  map<FeatureType, ID> res;

  map<FeatureType, std::pair<ID, float>> best_candidates{
    { face, { "", numeric_limits<float>::max() } },
    { body, { "", numeric_limits<float>::max() } },
    { voice, { "", numeric_limits<float>::max() } }
  };

  for (auto vp : make_iterator_range(vertices(g)))
  {
    for (auto type : { face, body, voice })
    {
      for (const auto& kv : id_vertex_map.at(type))
      {
        if (kv.second == vp)
        {
          if (d[vp] < best_candidates[type].second)
          {
            // cout << "best candidate: " << kv.first << " " << d[vp] << endl;
            best_candidates[type] = { kv.first, d[vp] };
            goto next_vertex;
          }
          else
            goto next_vertex;
        }
      }
    }
  next_vertex:;
  }

  if (best_candidates[face].second < threshold)
  {
    res[face] = best_candidates[face].first;
  }
  if (best_candidates[body].second < threshold)
  {
    res[body] = best_candidates[body].first;
  }
  if (best_candidates[voice].second < threshold)
  {
    res[voice] = best_candidates[voice].first;
  }

  return res;
}


map<ID, map<FeatureType, ID>> PersonMatcher::get_all_associations() const
{
  map<ID, map<FeatureType, ID>> res;

  for (const auto& kv : id_vertex_map.at(person))
  {
    res[kv.first] = get_association(kv.first);
  }

  return res;
}
