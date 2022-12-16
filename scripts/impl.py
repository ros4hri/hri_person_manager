#! python3
import matplotlib.pyplot as plt
import math
import networkx as nx
import itertools
from collections import Counter
import random

PERSON = 0b0001
FACE = 0b0010
VOICE = 0b0100
BODY = 0b1000


LIKELIHOOD_THRESHOLD = 0.4

NO_FEATURE = "_"


def next_anon_id():
    return "anon_" + "".join(random.choices("abcdfghijklmnopqrstuvwxyz", k=5))


def fully_connect_persons(G, mask, likelihood_threshold):
    G = nx.Graph(G)  # unfreeze the graph
    if not (mask & PERSON):  # no person in this subgraph. Add anonymous one

        id = next_anon_id()

        for n in list(G.nodes):
            G.add_edge(
                n,
                id,
                weight=likelihood2weights(likelihood_threshold),
                likelihood=likelihood_threshold,
                log_weight=log_transform_weights(likelihood_threshold),
            )

        G.nodes[id].update({"type": PERSON})
        return G, []

    else:

        person = None

        # get the person node
        for n in G.nodes:
            if G.nodes[n]["type"] == PERSON:
                person = n
                break

        edges_to_update = []

        for n in G.nodes:
            if n == person:
                continue

            if (n, person) in G.edges and not G.edges[(n, person)]["computed"]:
                # already connected
                continue

            # if existing computed edge, check if we need to update the weight
            if (n, person) in G.edges:
                tmpG = nx.Graph(G)
                tmpG.remove_edge(n, person)
                if not nx.is_connected(tmpG):
                    continue
                G.remove_edge(n, person)

            length = nx.dijkstra_path_length(G, n, person, weight="log_weight")
            likelihood = back_log_transform_weights(length)

            if likelihood > likelihood_threshold:

                edges_to_update.append((n, person, likelihood))

                G.add_edge(
                    n,
                    person,
                    weight=likelihood2weights(likelihood),
                    likelihood=likelihood,
                    log_weight=log_transform_weights(likelihood),
                    computed=True,
                )

        return G, edges_to_update


def prune_unlikely_connections(G, likelihood_threshold):
    resG = nx.Graph(G)
    for u, v, l in list(G.edges.data("likelihood")):
        if l < likelihood_threshold:
            print(
                "Pruning edge %s <-> %s due to likelihood l=%s below threshold"
                % (u, v, l)
            )
            resG.remove_edge(u, v)
    return resG


def build_partitions(nodes, nb_features=4):

    # print("get_partitions input nodes: %s" % nodes)
    if len(nodes) == 1:
        n, t = nodes[0]
        # print("---> return partitions: " + str([[([n], t)]]))
        return [[([n], t)]]

    head, rest = nodes[0], nodes[1:]

    head_node, head_type = head

    partitions = build_partitions(rest, nb_features)

    updated_partitions = []

    for p in partitions:

        for i, subgraph in enumerate(p):
            g, mask = subgraph
            if (
                len(g) < nb_features
            ):  # sub-graph does not contain more nodes than features (otherwise, at least one feature is duplicated)
                if not (
                    mask & head_type
                ):  # sub-graph does not already contains this type of node
                    new_partition = p[:]
                    new_partition[i] = (g + [head_node], mask | head_type)
                    # print("Add partition " + str(new_partition))
                    updated_partitions.append(new_partition)

        # print("Add partition " + str(p + [([head_node], head_type)]))
        updated_partitions.append(p + [([head_node], head_type)])

    # print("---> return partitions: " + str(updated_partitions))
    return updated_partitions


def get_partitions(G, nb_features=4):

    nodes = [(n, G.nodes[n]["type"]) for n in G.nodes]
    raw_partitions = build_partitions(nodes, nb_features)

    print("Got %d raw partitions" % len(raw_partitions))
    # for p in raw_partitions:
    #    print("- %s" % len(p))

    viable_partitions = []
    for p in raw_partitions:
        viable = True
        for g in p:
            if not nx.is_connected(G.subgraph(g[0])):
                viable = False
                break
        if viable:
            viable_partitions.append(p)

    print(
        "Got %d viable partitions (made of connected graphs)" % len(viable_partitions)
    )
    # for p in viable_partitions:
    #    print("- %s" % len(p))

    return viable_partitions


def partition_affinity(G, partition):
    """The graph partition 'affinity' is the sum of the likelihood of associations accross all the partition's subgraphs.

    It is computed by:

    - for each each subgraph:
        - finding the 'maximum likelihood' spanning tree, ie the minimum
          spanning tree using as edge weight w=1-likelihood
        - summing the likelihoods of every edge in the minimum spanning tree
    - then, summing it over all subgraphs
    """

    affinity = 0

    for g in partition:
        subG = nx.minimum_spanning_tree(G.subgraph(g[0]))

        affinity += sum([subG.edges[e]["likelihood"] for e in subG.edges])

    return affinity


def algorithm(G, likelihood_threshold=LIKELIHOOD_THRESHOLD):

    G = prune_unlikely_connections(G, likelihood_threshold)

    full_partition = []

    for subG_nodes in nx.connected_components(G):

        subG = G.subgraph(subG_nodes)

        # print("Input edges: %s" % subG.edges)

        # get all viable partitions of the graph
        partitions = get_partitions(subG)

        # only keep the most compact partitions (ie, partitions with the minimal number of subgraph)
        min_len = min([len(p) for p in partitions])
        partitions = filter(lambda x: len(x) == min_len, partitions)
        partitions = sorted(
            partitions, key=lambda x: partition_affinity(subG, x), reverse=True
        )

        # for p in partitions:
        #    print(p)
        #    print(partition_affinity(subG, p))

        full_partition += partitions[0]

    if full_partition:

        resG = nx.Graph()

        for nodes in full_partition:
            subG = G.subgraph(nodes[0])
            subG, edges_to_update = fully_connect_persons(
                subG, mask=nodes[1], likelihood_threshold=likelihood_threshold
            )
            resG.update(subG)

            for s, t, l in edges_to_update:
                if (s, t) in G.edges:
                    G.remove_edge(s, t)

                G.add_edge(
                    s,
                    t,
                    likelihood=l,
                    weight=likelihood2weights(l),
                    log_weight=log_transform_weights(l),
                    computed=True,
                )

        return G, resG
    else:
        return G, nx.Graph()


def likelihood2weights(v):
    return round(1 - v, 3)


def weights2likelihood(v):
    return likelihood2weights(v)


def log_transform_weights(w):
    if w <= 0:
        return 1000
    return math.log(1 / w)


def back_log_transform_weights(w):
    if w > 900:
        return 0
    return round(1 / math.exp(w), 3)


def get_color(type):
    if type == PERSON:
        return "blue"
    if type == VOICE:
        return "orange"
    if type == FACE:
        return "green"
    if type == BODY:
        return "purple"


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


if __name__ == "__main__":

    G = nx.Graph()

    for a, b, l in [
        #        ("body3", "person2", 0.7),
        #        ("person2", "face1", 0.9),
        #        ("face1", "body2", 0.8),
        #        ("face1", "person1", 0.2),
        #        ("face1", "body1", 0.1),
        #        #        ("voice1", "body2", 0.5),
        #        #        ("voice3", "body3", 0.5),
        #        ("person1", "face2", 0.7),
        #        ("person1", "body2", 0.81),
        #        ("body1", "voice2", 0.9),
        #        ("face2", "body1", 0.6),
        ("f1", "p1", 0.8),
        ("f1", "p2", 0.7),
        ("f1", "p3", 0.5),
        ("f2", "p4", 0.8),
        ("f2", "p2", 0.65),
        ("b1", "f1", 0.8),
        ("b1", "f2", 0.75),
        ("b2", "f2", 0.7),
        ("b2", "f1", 0.65),
        ("v1", "f1", 0.6),
        ("v1", "f2", 0.58),
        ("v3", "f1", 0.2),
        ("v3", "f2", 0.21),
    ]:
        G.add_edge(
            a,
            b,
            weight=likelihood2weights(l),
            likelihood=l,
            log_weight=log_transform_weights(l),
            computed=False,
        )

    for n in G.nodes:
        if n.startswith("p"):
            G.nodes[n].update({"type": PERSON})

        if n.startswith("b"):
            G.nodes[n].update({"type": BODY})

        if n.startswith("v"):
            G.nodes[n].update({"type": VOICE})

        if n.startswith("f"):
            G.nodes[n].update({"type": FACE})

    ######################

    _, associations = algorithm(G)
    plot(associations)
