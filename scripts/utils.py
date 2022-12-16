import re
import networkx as nx

from impl import (
    likelihood2weights,
    log_transform_weights,
    weights2likelihood,
    algorithm,
)
from impl import PERSON, FACE, VOICE, BODY
import matplotlib.pyplot as plt


def preparse(lines):
    lines = [l.strip() for l in lines.split("\n") if l]
    lines = [
        l
        for l in lines
        if (
            not l.startswith("%%")
            and not l.startswith("```")
            and not l.startswith("graph")
        )
    ]

    return lines


def get_connected_subgraphs(G):

    return [G.subgraph(n) for n in G.connected_components()]


def get_type(name):
    if name.startswith("p") or name.startswith("a"):
        return PERSON
    if name.startswith("b"):
        return BODY
    if name.startswith("v"):
        return VOICE
    if name.startswith("f"):
        return FACE

    raise RuntimeError(
        "the node names must start with (p)erson (or (a)nonymous), (v)oice, (b)ody or (f)ace. Got %s"
        % name
    )


def input_parser(lines, G):

    lines = preparse(lines)

    for l in lines:

        res = re.match(r"(\w+),(\w+),([\d\.]+)", l)
        if res:
            a, b, w = res.groups()
            w = float(w)

            print("add edge: %s -- %s -- %s" % (a, w, b))
            G.add_node(a, type=get_type(a))
            G.add_node(b, type=get_type(b))
            G.add_edge(
                a,
                b,
                weight=likelihood2weights(w),
                likelihood=w,
                log_weight=log_transform_weights(w),
                computed=False,
            )
        else:
            res = re.match(r"-(\w+)", l)
            if res:
                a = res.groups()[0]
                if a is None:
                    print("error! malformed line: [%s]" % l)
                else:
                    print("remove node: %s" % a)
                    G.remove_node(a)
            else:
                res = re.match(r"(\w+)", l)
                if res:
                    a = res.groups()[0]
                    if a is None:
                        print("error! malformed line: [%s]" % l)
                    else:
                        print("add node: %s" % a)
                        G.add_node(a, type=get_type(a))
    return G


def mermaid_parser(g):

    G = nx.Graph()

    lines = preparse(g)

    for l in lines:

        res = re.match(r"(\w+) ---\|([\d\.]+)\| (\w+)", l)
        if res:
            a, w, b = res.groups()
            w = float(w)

            G.add_node(a, type=get_type(a))
            G.add_node(b, type=get_type(b))
            print("add edge: %s -- %s -- %s" % (a, w, b))
            G.add_edge(
                a,
                b,
                weight=likelihood2weights(w),
                likelihood=w,
                log_weight=log_transform_weights(w),
                computed=False,
            )
        else:
            res = re.match(r"(\w+)(.*)", l)
            if res:
                a, b = res.groups()
                if a is None or b != "":
                    print("error! malformed line: [%s]" % l)
                else:
                    print("add node: %s" % a)
                    G.add_node(a, type=get_type(a))

    return G


def plot(G, title="Network", show=True):

    fig, ax = plt.subplots(figsize=(12, 12))

    pos = nx.spring_layout(G, weight="likelihood", seed=7)

    # nodes
    nx.draw_networkx_nodes(G, pos, nodelist=G.nodes)
    nx.draw_networkx_labels(G, pos)
    nx.draw_networkx_edges(G, pos)

    # edge weight labels
    edge_labels = dict(
        [
            (e, weights2likelihood(w))
            for e, w in nx.get_edge_attributes(G, "weight").items()
        ]
    )

    nx.draw_networkx_edge_labels(G, pos, edge_labels)

    # Resize figure for label readibility
    ax.margins(0.1, 0.05)
    fig.tight_layout()
    plt.axis("off")
    plt.title(title)

    if show:
        plt.show()


def edge_match(e1, e2):
    return True if e1["likelihood"] == e2["likelihood"] else False


def run_test(G, test, last=False):

    print("\n------------------------------------------------")
    print("[TEST] %s\n" % test["name"])

    if nx.is_isomorphic(G, test["graph"], edge_match=edge_match):
        print("Input graph correct.")
        print("Running algorithm...\n")

        G, graph = algorithm(G)
        anon_nodes = [n for n in graph.nodes if n.startswith("anon")]

        expected_anon_nodes = [n for n in test["output"].nodes if n.startswith("anon")]
        if anon_nodes != expected_anon_nodes:
            print("Relabelling anonymous nodes...")
            nx.relabel.relabel_nodes(
                graph, dict(zip(anon_nodes, expected_anon_nodes)), copy=False
            )

        if nx.is_isomorphic(graph, test["output"], edge_match=edge_match):
            print("TEST SUCCESSFUL")
        else:
            print("TEST FAILED")

            plot(test["graph"], title="Source", show=False)

            print("\nExpected:")
            print(nx.nx_pydot.to_pydot(test["output"]).to_string())
            plot(test["output"], title="Expected", show=False)

            print("\nGot:")
            print(nx.nx_pydot.to_pydot(graph).to_string())
            plot(graph, title="Computed (algorithm output)")

            if not last:
                input("\nPress enter to move to next test")

    else:
        print("[EE] input not parsed to expected graph:")

        print("\nExpected:")
        print(nx.nx_pydot.to_pydot(test["graph"]).to_string())

        print("\nGot:")
        print(nx.nx_pydot.to_pydot(G).to_string())

        if not last:
            input("\nPress enter to move to next test")

    return G


def test_file_parser(f):

    INPUT = "INPUT"
    GRAPH = "GRAPH"
    OUTPUT = "OUTPUT"

    G = nx.Graph()

    test = {}
    in_section = ""
    buffer = ""

    def end_of_section():
        nonlocal in_section, buffer

        if in_section == INPUT:
            print("...processing input...")
            input_parser(buffer, G)

        if in_section == GRAPH:
            print("...adding graph...")
            test["graph"] = mermaid_parser(buffer)

        if in_section == OUTPUT:
            print("...adding output...")
            test["output"] = mermaid_parser(buffer)

        in_section = ""
        buffer = ""

    for l in f.readlines():

        if l.startswith("RESET GRAPH"):
            if test:
                end_of_section()

                G = run_test(G, test)

                test = {}

            print("\n####################################")
            print(" Resetting the graph\n\n")
            G = nx.Graph()
            continue

        if l.startswith("#"):
            end_of_section()

        if l.startswith("# "):
            if test:
                G = run_test(G, test)
                test = {}

            test = {"name": l.split("#")[1].strip()}
            print("\n\nParsing test %s..." % test["name"])
            continue

        if l.startswith("## INPUT"):
            in_section = INPUT
            continue
        if l.startswith("## GRAPH"):
            in_section = GRAPH
            continue
        if l.startswith("## OUTPUT"):
            in_section = OUTPUT
            continue

        buffer += l + "\n"

    # run the last test as well
    end_of_section()
    if test:
        run_test(G, test, last=True)


if __name__ == "__main__":
    import sys

    with open(sys.argv[1], "r") as f:
        test_file_parser(f)
