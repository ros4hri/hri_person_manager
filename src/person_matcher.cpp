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


#include "person_matcher.h"


#include <boost/graph/graph_traits.hpp>
#include <boost/graph/dijkstra_shortest_paths.hpp>
#include "hri/base.h"


using namespace std;
using namespace hri;
using namespace boost;



PersonMatcher::PersonMatcher()
{
}


void PersonMatcher::update(Relations relations)
{
  add_edge(0, 1, 9, g);
  add_edge(0, 2, 7, g);
  add_edge(0, 3, 6, g);
  add_edge(2, 1, 9, g);
  add_edge(2, 3, 1, g);
  add_edge(4, 3, 5, g);

  boost::property_map<Graph, vertex_index_t>::type vertex_ids = get(vertex_index, g);
  boost::property_map<Graph, edge_weight_t>::type edge_probs = get(edge_weight, g);

  cout << "vertices(g) = ";

  for (auto vp : make_iterator_range(vertices(g)))
  {
    // cout << vertex_names[vp] << " ";
    cout << vp << " ";
  }
  cout << endl;

  cout << "edges(g) = ";

  for (auto ei : make_iterator_range(edges(g)))
  {
    cout << "(" << vertex_ids[source(ei, g)] << "," << vertex_ids[target(ei, g)]
         << ") => " << edge_probs[ei] << endl;
  }
  cout << endl;


  // vector for storing distance property
  vector<int> d(num_vertices(g));

  dijkstra_shortest_paths(g, *(vertices(g).first), distance_map(&d[0]));

  cout << "distances from start vertex:" << endl;
  for (auto vp : make_iterator_range(vertices(g)))
  {
    cout << "distance(" << vp << ") = " << d[vp] << endl;
  }
  cout << endl;
}

map<FeatureType, ID> PersonMatcher::get_association(ID)
{
  return {};
}
