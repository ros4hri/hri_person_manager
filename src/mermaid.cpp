#include <boost/graph/graph_utility.hpp>
#include <boost/graph/named_function_params.hpp>
#include <boost/graph/connected_components.hpp>
#include <boost/graph/copy.hpp>
#include <boost/graph/subgraph.hpp>
#include <fstream>
#include <stdexcept>
#include <string>
#include <vector>
#include <regex>

#include <hri/base.h>

#include <boost/graph/vf2_sub_graph_iso.hpp>

#include "person_matcher.h"

using namespace std;
using namespace hri;

const string INPUT = "INPUT";
const string GRAPH = "GRAPH";
const string OUTPUT = "OUTPUT";

// trim from start (in place)
static inline void ltrim(std::string &s)
{
  s.erase(s.begin(), std::find_if(s.begin(), s.end(),
                                  [](unsigned char ch) { return !std::isspace(ch); }));
}

// trim from end (in place)
static inline void rtrim(std::string &s)
{
  s.erase(std::find_if(s.rbegin(), s.rend(),
                       [](unsigned char ch) { return !std::isspace(ch); })
              .base(),
          s.end());
}

// trim from both ends (in place)
static inline void trim(std::string &s)
{
  rtrim(s);
  ltrim(s);
}

vector<string> preparse(const vector<string> &lines)
{
  vector<string> res;

  for (const auto &l : lines)
  {
    string s(l);
    trim(s);
    if (s.size() > 0 && not((s.rfind("\%\%", 0) == 0) || (s.rfind("```", 0) == 0) ||
                            (s.rfind("graph", 0) == 0)))
    {
      res.push_back(l);
    }
  }
  return res;
}

FeatureType get_type(const string &name)
{
  switch (name[0])
  {
    case 'p':
    case 'a':
      return FeatureType::person;
    case 'b':
      return FeatureType::body;
    case 'v':
      return FeatureType::voice;
    case 'f':
      return FeatureType::face;
  }

  throw runtime_error(string("unknown node type: ") + name);
}

void input_parser(const vector<string> &raw_lines, PersonMatcher &pm)
{
  auto lines = preparse(raw_lines);

  std::regex edge_regex("(\\w+),(\\w+),([\\d.]+)");
  std::regex remove_node_regex("-(\\w+)");
  std::regex add_node_regex("(\\w+)");

  std::smatch result;

  for (const auto &line : lines)
  {
    if (std::regex_search(line, result, edge_regex) && result.size() > 1)
    {
      auto a = result.str(1);
      auto b = result.str(2);
      auto likelihood = stof(result.str(3));
      cout << "add edge " << a << " -- " << likelihood << " -- " << b << endl;

      pm.update({ { a, get_type(a), b, get_type(b), likelihood } });
    }
    else if (std::regex_search(line, result, remove_node_regex) && result.size() > 1)
    {
      auto a = result.str(1);
      cout << "remove node " << a << endl;
      pm.erase(a);
    }
    else if (std::regex_search(line, result, add_node_regex) && result.size() > 1)
    {
      auto a = result.str(1);
      cout << "add node " << a << endl;
      pm.update({ { a, get_type(a), a, get_type(a), 0. } });
    }
    else
    {
      throw runtime_error(string("malformed input: ") + line);
    }
  }
}

void mermaid_parser(const vector<string> &raw_lines, Graph &G)
{
  auto lines = preparse(raw_lines);

  std::regex edge_regex("(\\w+) ---\\|([\\d.]+)\\| (\\w+)");
  std::regex add_node_regex("(\\w+)(.*)");

  std::smatch result;

  for (const auto &line : lines)
  {
    if (std::regex_search(line, result, edge_regex) && result.size() > 1)
    {
      auto a = result.str(1);
      auto b = result.str(3);
      auto likelihood = stof(result.str(2));

      cout << "add edge " << a << " -- " << likelihood << " -- " << b << endl;

      auto node_a = get_node_by_name(a, G);
      if (node_a == INEXISTANT_VERTEX)
      {
        node_a = add_vertex(G);
        G[node_a].name = a;
        G[node_a].type = get_type(a);
        if (a.rfind("anon", 0) == 0)
        {
          G[node_a].anonymous = true;
        }
      }

      auto node_b = get_node_by_name(b, G);
      if (node_b == INEXISTANT_VERTEX)
      {
        node_b = add_vertex(G);
        G[node_b].name = b;
        G[node_b].type = get_type(b);
        if (b.rfind("anon", 0) == 0)
        {
          G[node_b].anonymous = true;
        }
      }

      auto e = add_edge(node_a, node_b, G).first;
      G[e].likelihood = likelihood;
      G[e].weight = likelihood2weight(likelihood);
      put(boost::edge_weight_t(), G, e, G[e].weight);
      G[e].log_likelihood = log_likelihood(likelihood);
    }
    else if (std::regex_search(line, result, add_node_regex) && result.size() > 1)
    {
      auto a = result.str(1);
      cout << "add node " << a << endl;

      auto node_a = add_vertex(G);
      G[node_a].name = a;
      G[node_a].type = get_type(a);
      if (a.rfind("anon", 0) == 0)
      {
        G[node_a].anonymous = true;
      }
    }
    else
    {
      throw runtime_error(string("malformed mermaid input: ") + line);
    }
  }
}


struct my_callback
{
  template <typename CorrespondenceMap1To2, typename CorrespondenceMap2To1>
  bool operator()(CorrespondenceMap1To2 /*f*/, CorrespondenceMap2To1) const
  {
    return false;  // breaks after the first subgraph is found
  }
};

void print_graph(const Graph &G)
{
  for (const auto &n : boost::make_iterator_range(vertices(G)))
  {
    cout << "  - " << G[n].name << endl;
  }
  for (const auto &e : boost::make_iterator_range(edges(G)))
  {
    Node a = source(e, G);
    Node b = target(e, G);

    cout << "  " << G[a].name << " -- " << G[e].likelihood << " -- " << G[b].name << endl;
  }
}

void print_associations(const Graph &_G)
{
  Graph G(_G);

  std::vector<int> component(num_vertices(G));  // component map
  auto nb_components = boost::connected_components(G, &component[0]);

  vector<Graph> connected_components;

  for (size_t i = 0; i < nb_components; i++)
  {
    auto subG = G.create_subgraph();
    for (size_t j = 0; j < num_vertices(G); j++)
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

  for (size_t i = 0; i < connected_components.size(); i++)
  {
    cout << " Association " << i + 1 << ":" << endl;
    print_graph(connected_components[i]);
    cout << endl;
  }
}


void run_test(const string &test_name, PersonMatcher &input, const Graph &graph, const Graph &output)
{
  cout << "\n------------------------------------------------" << endl;
  cout << "[TEST] " << test_name << endl << endl;

  auto input_g = input.get_internal_graph(true);

  auto vos = boost::copy_range<vector<Node> >(vertices(input_g));

  if (num_vertices(input_g) == num_vertices(graph) &&
      // need to run the VF2 isomorphism *twice*: once to check that 'input' is
      // isomorphic to a subgraph of 'graph', once to check to opposite.
      vf2_subgraph_iso(
          input_g, graph, my_callback(), vos,
          boost::edges_equivalent([&input_g, &graph](const Edge &e1, const Edge &e2) {
            return compare_likelihoods(input_g[e1].likelihood, graph[e2].likelihood);
          }).vertices_equivalent([&input_g, &graph](const Node &n1, const Node &n2) {
            return input_g[n1].name == graph[n2].name;
          })) &&

      // cout << "B->A" << endl;
      vf2_subgraph_iso(
          graph, input_g, my_callback(), vos,
          boost::edges_equivalent([&graph, &input_g](const Edge &e1, const Edge &e2) {
            return compare_likelihoods(input_g[e2].likelihood, graph[e1].likelihood);
          }).vertices_equivalent([&graph, &input_g](const Node &n1, const Node &n2) {
            return input_g[n2].name == graph[n1].name;
          }))

  )
  {
    cout << "☑️ Input graph correct." << endl << "Running algorithm...\n" << endl;

    auto associations = input.get_raw_associations();

    Graph final_graph;
    for (const auto &G : associations)
    {
      boost::copy_graph(G.first, final_graph);
    }

    vos = boost::copy_range<vector<Node> >(vertices(final_graph));

    if (num_vertices(final_graph) == num_vertices(output) &&
        // same as above, needs to run the VF2 algorithm twice.
        // cout << "A->B" << endl;
        vf2_subgraph_iso(
            final_graph, output, my_callback(), vos,
            boost::edges_equivalent([&final_graph, &output](const Edge &e1, const Edge &e2) {
              // cout << "Comparing " << final_graph[e1].likelihood << " to "
              //     << output[e2].likelihood << endl;
              // cout << compare_likelihoods(final_graph[e1].likelihood, output[e2].likelihood) << endl;
              return compare_likelihoods(final_graph[e1].likelihood, output[e2].likelihood);
            }).vertices_equivalent([&final_graph, &output](const Node &n1, const Node &n2) {
              // cout << "Comparing " << final_graph[n1].name << " to " << output[n2].name
              // << endl; cout << (final_graph[n1].name == output[n2].name) << endl;
              return final_graph[n1].name == output[n2].name;
            })) &&

        // cout << "B->A" << endl;
        vf2_subgraph_iso(
            output, final_graph, my_callback(), vos,
            boost::edges_equivalent([&output, &final_graph](const Edge &e1, const Edge &e2) {
              // cout << "Comparing " << final_graph[e2].likelihood << " to "
              //     << output[e1].likelihood << endl;
              // cout << compare_likelihoods(final_graph[e2].likelihood, output[e1].likelihood) << endl;
              return compare_likelihoods(final_graph[e2].likelihood, output[e1].likelihood);
            }).vertices_equivalent([&output, &final_graph](const Node &n1, const Node &n2) {
              // cout << "Comparing " << final_graph[n2].name << " to " << output[n1].name
              // << endl; cout << (final_graph[n2].name == output[n1].name) << endl;
              return final_graph[n2].name == output[n1].name;
            })))

    {
      cout << "✅ TEST SUCCESSFUL" << endl;
    }
    else
    {
      cout << "🟥 TEST FAILED" << endl;

      cout << "\nComputed associations:" << endl;
      print_associations(final_graph);
      cout << "\nExpected associations:" << endl;
      print_associations(output);

      exit(1);
    }
  }
  else
  {
    cout << "🟥 input not parsed to expected graph:" << endl;

    cout << "\nComputed graph (" << boost::num_vertices(input_g) << " vertices):" << endl;
    // cout << input.get_graphviz() << endl;
    print_graph(input_g);
    cout << "\nExpected graph (" << boost::num_vertices(graph) << " vertices):" << endl;
    print_graph(graph);

    exit(1);
  }
}

int main(int argc, char **argv)
{
  ifstream input_file(argv[1]);
  string line;

  string in_section;
  vector<string> buffer;


  bool has_test = false;
  PersonMatcher input(0.5, false);  // false to use sequential anonmyous ID instead of random ones
  Graph graph, output;
  string test_name;

  std::regex threshold_regex("THRESHOLD: ([\\d.]+)");
  std::smatch result;

  while (std::getline(input_file, line))
  {
    if (std::regex_search(line, result, threshold_regex) && result.size() > 1)
    {
      auto threshold = stof(result.str(1));
      cout << "Setting likelihood threshold to " << threshold << endl;
      input.set_threshold(threshold);
      continue;
    }
    if (line.rfind("RESET GRAPH", 0) == 0)
    {
      if (has_test)
      {
        // parse buffer
        if (in_section == INPUT)
        {
          cout << "...processing input..." << endl;
          input_parser(buffer, input);
        }
        if (in_section == GRAPH)
        {
          cout << "...adding graph..." << endl;
          graph = Graph();
          mermaid_parser(buffer, graph);
        }
        if (in_section == OUTPUT)
        {
          cout << "...adding output..." << endl;
          output = Graph();
          mermaid_parser(buffer, output);
        }

        in_section.clear();
        buffer.clear();

        // run test
        run_test(test_name, input, graph, output);
        has_test = false;
      }

      cout << "\n#####################################################" << endl;
      cout << "#####################################################" << endl;
      cout << "🌟 RESET GRAPH\n\n" << endl;
      input.reset();
      graph = Graph();
      output = Graph();
      continue;
    }
    if (line.rfind("#", 0) == 0)
    {
      // parse buffer
      if (in_section == INPUT)
      {
        cout << "...processing input..." << endl;
        input_parser(buffer, input);
      }
      if (in_section == GRAPH)
      {
        cout << "...adding graph..." << endl;
        graph = Graph();
        mermaid_parser(buffer, graph);
      }
      if (in_section == OUTPUT)
      {
        cout << "...adding output..." << endl;
        output = Graph();
        mermaid_parser(buffer, output);
      }

      in_section.clear();
      buffer.clear();
    }
    if (line.rfind("# ", 0) == 0)
    {
      if (has_test)
      {
        run_test(test_name, input, graph, output);
        has_test = false;
      }
      test_name = line.substr(2);
      cout << "\n####################################" << endl;
      cout << "\n[TEST PARSING] " << test_name << "..." << endl;
      has_test = true;
      continue;
    }
    if (line.rfind("## INPUT", 0) == 0)
    {
      in_section = INPUT;
      continue;
    }
    if (line.rfind("## OUTPUT", 0) == 0)
    {
      in_section = OUTPUT;
      continue;
    }
    if (line.rfind("## GRAPH", 0) == 0)
    {
      in_section = GRAPH;
      continue;
    }


    buffer.push_back(line);
  }
}

