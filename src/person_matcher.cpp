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

#include <boost/graph/properties.hpp>
#include <cmath>
#include <limits>
#include <sstream>
#include <stdexcept>
#include <tuple>

#include "person_matcher.h"
#include "managed_person.h"

#include <boost/graph/detail/adjacency_list.hpp>
#include <boost/graph/graph_traits.hpp>
#include <boost/graph/dijkstra_shortest_paths.hpp>
#include "hri/base.h"
#include <boost/graph/graphviz.hpp>

using namespace std;
using namespace hri;
using namespace boost;

typedef std::map<hri::FeatureType, std::map<hri::ID, Vertex>> IdVertexMap;

//////////// helpers for write_graphviz ////////////////
class node_label_writer
{
public:
  node_label_writer(Graph _g, std::map<hri::FeatureType, std::set<hri::ID>> _id_types)
    : g(_g), id_types(_id_types)
  {
  }
  template <class VertexOrEdge>
  void operator()(std::ostream& out, const VertexOrEdge& v) const
  {
    auto name = get(&VertexProps::name, g, v);

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
  template <class VertexOrEdge>
  void operator()(std::ostream& out, const VertexOrEdge& e) const
  {
    auto w = get(boost::edge_weight_t(), g, e);
    out << "[label=\"" << 1 / exp(w) << "\"]";
  }

private:
  Graph g;
};
///////////////////////////////////////////////////////////////////////////

Vertex get_vertex(const Graph& g, const ID id)
{
  Graph::vertex_iterator v, vend;
  for (boost::tie(v, vend) = vertices(g); v != vend; ++v)
  {
    if (id == get(&VertexProps::name, g, *v))
    {
      return *v;
    }
  }

  return INEXISTANT_VERTEX;
}

PersonMatcher::PersonMatcher(float likelihood_threshold)
{
  threshold = log(1 / likelihood_threshold);

  for (auto type : { face, body, voice, person })
  {
    id_types[type] = set<ID>();
  }
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

    Vertex v1 = get_vertex(g, id1);
    Vertex v2 = get_vertex(g, id2);

    if (v1 == INEXISTANT_VERTEX)
    {
      v1 = add_vertex(g);
      g[v1].name = id1;
      id_types[type1].insert(id1);
    }

    if (v2 == INEXISTANT_VERTEX)
    {
      v2 = add_vertex(g);
      g[v2].name = id2;
      id_types[type2].insert(id2);
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

std::set<ID> PersonMatcher::erase(ID id)
{
  set<ID> removed_persons;

  Vertex v = get_vertex(g, id);
  if (v == INEXISTANT_VERTEX)
  {
    return removed_persons;
  }

  clear_vertex(v, g);
  remove_vertex(v, g);

  erase_id(id);

  if (id_types[person].count(id))
  {
    removed_persons.insert(id);
  }

  auto orphaned_persons = clear_orphaned_persons();

  removed_persons.insert(orphaned_persons.begin(), orphaned_persons.end());

  return removed_persons;
}

std::set<ID> PersonMatcher::clear_orphaned_persons()
{
  // remove all orphan vertices and return removed persons
  set<ID> removed_persons;

  Graph::vertex_iterator v, vend;

  while (true)
  {
    bool found_orphan = false;
    for (boost::tie(v, vend) = vertices(g); v != vend; ++v)
    {
      if (out_degree(*v, g) == 0)
      {
        found_orphan = true;

        ID id = get(&VertexProps::name, g, *v);
        ROS_INFO_STREAM(" ---> clearing orphan " << id);

        // is it a person node? if so, store it
        if (id_types[person].count(id))
        {
          removed_persons.insert(id);
          erase_id(id);

          remove_vertex(*v, g);
        }



        // we break and restart the iteration over all the vertices of the graph
        // because when a vertex is removed, the graph reindex all vertices -- if we were
        // to first build a list of all vertices to delete, that list would be invalid
        // after the first removed vertex.
        break;
      }
    }

    // continue until no more orphans
    if (!found_orphan)
    {
      break;
    }
  }

  return removed_persons;
}

void PersonMatcher::reset()
{
  g.clear();
}

map<FeatureType, ID> PersonMatcher::get_association(ID id) const
{
  auto vertex = get_vertex(g, id);
  if (vertex == INEXISTANT_VERTEX)
  {
    throw runtime_error("requesting associations for inexistant ID");
  }

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
      for (const auto& _id : id_types.at(type))
      {
        if (get_vertex(g, _id) == vp)
        {
          if (d[vp] < best_candidates[type].second)
          {
            // cout << "best candidate: " << kv.first << " " << d[vp] << endl;
            best_candidates[type] = { _id, d[vp] };
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


pair<map<ID, map<FeatureType, ID>>, set<Feature>> PersonMatcher::get_all_associations() const
{
  map<ID, map<FeatureType, ID>> res;

  for (const auto& id : id_types.at(person))
  {
    res[id] = get_association(id);
  }

  set<Feature> orphan_features;

  for (auto type : { face, body, voice })
  {
    for (const auto& id : id_types.at(type))
    {
      for (const auto& person_id : id_types.at(person))
      {
        bool used = false;
        if (res[person_id].count(type) && res[person_id][type] == id)
        {
          used = true;
        }
        if (!used)
        {
          orphan_features.insert(make_pair(id, type));
        }
      }
    }
  }

  return make_pair(res, orphan_features);
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

