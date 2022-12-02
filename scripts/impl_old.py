#! python3
import matplotlib.pyplot as plt
import math
import networkx as nx
import itertools
from collections import Counter
import random

PERSON = "person"
BODY = "body"
FACE = "face"
VOICE = "voice"

LIKELIHOOD_THRESHOLD = 0.4

NO_FEATURE = "_"


def next_anon_id():
    return "anon_" + "".join(random.choices("abcdfghijklmnopqrstuvwxyz", k=5))


def cost_path(p, G):

    if not p:
        return 0.0

    n1, n2 = p[-1]

    return cost_path(p[:-1], G) * (1 + G[n1][n2]["weight"])


def add_anonymous_persons(G, likelihood_threshold):
    for n in list(G.nodes):
        if G.nodes[n]["type"] != PERSON:
            id = next_anon_id()
            G.add_edge(
                n,
                id,
                weight=transform_weights(likelihood_threshold),
                # weight=transform_weights(0),
                likelihood=likelihood_threshold,
                # likelihood=0,
            )
            G.nodes[id].update({"type": PERSON})


def find_permissible_paths(n1, n2, G):
    if G.nodes[n1]["type"] == G.nodes[n2]["type"]:
        return []

    paths = nx.all_simple_edge_paths(G, source=n1, target=n2)
    # paths = nx.all_shortest_paths(G, source=n1, target=n2, weight="weight")

    permissible_paths = []
    for p in paths:
        if len(p) == 1:
            permissible_paths.append((p, cost_path(p, G)))

        node_types = set()
        permissible = True
        for e in p:
            u, v = e
            if (
                G.nodes[u]["type"] in node_types  # a type we've already seen? forbidden
                or G.nodes[u]["type"]
                == G.nodes[n2]["type"]  # the type of the destination? forbidden
            ):
                permissible = False
                break

            node_types.add(G.nodes[u]["type"])

        if permissible:
            permissible_paths.append((p, cost_path(p, G)))

    return permissible_paths


def best_permissible_path(n1, n2, G):
    paths = find_permissible_paths(n1, n2, G)
    if not paths:
        return None
    if len(paths) == 1:
        return paths[0]

    return sorted(paths, key=lambda x: x[1])[0]


def path_to_edge_path(p):
    prev = p[0]
    edge_path = []
    for n in p[1:]:
        edge_path.append((prev, n))
    return edge_path


def edge_path_to_path(p):
    path = []

    for u, _ in p:
        path.append(u)

    path.append(p[-1][1])
    return path


def possible_person_associations(p, G):

    # get all known faces, bodies, voices
    faces = [k for k, attr in G.nodes.items() if attr["type"] == FACE]
    bodies = [k for k, attr in G.nodes.items() if attr["type"] == BODY]
    voices = [k for k, attr in G.nodes.items() if attr["type"] == VOICE]

    permissible_features = {
        "face": {NO_FEATURE: ([], 0)},
        "body": {NO_FEATURE: ([], 0)},
        "voice": {NO_FEATURE: ([], 0)},
    }

    # for each face, body, voice, check whether there exists a path
    # to the person, and if so, returns the best one
    for f in faces:
        path = best_permissible_path(p, f, G)
        if path:
            permissible_features["face"][f] = path

    for f in bodies:
        path = best_permissible_path(p, f, G)
        if path:
            permissible_features["body"][f] = path

    for f in voices:
        path = best_permissible_path(p, f, G)
        if path:
            permissible_features["voice"][f] = path

    # for t, v in permissible_features.items():
    #    print(t)
    #    for n, p in v.items():
    #        print("%s -> %s" % (n, p))

    combinations = list(
        itertools.product(
            permissible_features["face"].items(),
            permissible_features["body"].items(),
            permissible_features["voice"].items(),
        )
    )

    combinations_likelihood = []

    for combination in combinations:

        # TODO: early pruning of invalid feature sets (eg _,_,_) fails because
        # then, maximise_total_likelihood can not find valid solutions (as one
        # anonymous person is created per feature -> impossible to find a
        # partition for all persons)

        # if [f[0] for f in combination] == [NO_FEATURE, NO_FEATURE, NO_FEATURE]:
        #    continue

        likelihood = 0.0
        nodes_in_combination_graph = {}

        valid_combination = True

        for item in combination:
            feature, path_cost = item
            path, cost = path_cost

            if feature != NO_FEATURE:

                # ensure that all the nodes required to reach that feature are
                # present in the combination (otherwise, we could create
                # 'disconnected' association)
                for u, v in path:
                    t = G.nodes[v]["type"]
                    t_idx = list(permissible_features.keys()).index(t)
                    if combination[t_idx][0] != v:
                        valid_combination = False
                        break

                for node in edge_path_to_path(path):
                    nodes_in_combination_graph.setdefault(
                        G.nodes[node]["type"], set()
                    ).add(node)

                likelihood += backtransform_weights(cost)

        # the graph spanned by the selected features in the combination can not contain more than one node of each type
        for _, nodes in nodes_in_combination_graph.items():
            if len(nodes) > 1:
                valid_combination = False

        if valid_combination:
            combinations_likelihood.append(([i[0] for i in combination], likelihood))

    return sorted(combinations_likelihood, key=lambda x: x[1], reverse=True)


def maximise_total_likelihood(candidate_combinations):
    full_combinations = itertools.product(*(candidate_combinations.values()))

    full_combinations_with_likelihood = []

    for candidate in full_combinations:

        valid = True

        # if the same feature is used more than once across persons, this is
        # not a viable overall set of association
        used_features = []
        for c in candidate:
            used_features += c[0]
        counted = Counter(used_features)
        for f, c in counted.items():
            if f != NO_FEATURE and c > 1:
                valid = False
                break

        if not valid:
            continue

        combos = {p: candidate[i] for i, p in enumerate(candidate_combinations.keys())}
        total_likelihood = sum([p[1] for p in candidate])

        full_combinations_with_likelihood.append((combos, total_likelihood))

    return sorted(full_combinations_with_likelihood, key=lambda x: x[1], reverse=True)


def prune_unlikely_connections(G, likelihood_threshold):
    for u, v, l in list(G.edges.data("likelihood")):
        if l < likelihood_threshold:
            print(
                "Pruning edge %s <-> %s due to likelihood l=%s below threshold"
                % (u, v, l)
            )
            G.remove_edge(u, v)


def algorithm(G, likelihood_threshold=LIKELIHOOD_THRESHOLD):

    print("############ STEP 1 ################")
    print("##                                ##")
    print("##  Prune unlikely connections    ##")
    print("##                                ##")
    print("####################################")
    prune_unlikely_connections(G, likelihood_threshold)

    print("############ STEP 2 ################")
    print("##                                ##")
    print("##  Add default anonymous persons ##")
    print("##  to all features               ##")
    print("##                                ##")
    print("####################################")
    add_anonymous_persons(G, likelihood_threshold)

    print("############ STEP 3 ################")
    print("##                                ##")
    print("##  Most likely associations      ##")
    print("##                                ##")
    print("####################################")
    persons = [k for k in G.nodes if G.nodes[k]["type"] == PERSON]
    for p in persons:
        print("\nPossible associations for person %s:" % p)
        combinations_likelihood = possible_person_associations(p, G)

        for i in combinations_likelihood:
            print(i)

    print("\n\nAssociations combinations\n")
    candidate_combinations = {p: possible_person_associations(p, G) for p in persons}
    best = maximise_total_likelihood(candidate_combinations)

    print(str(len(best)) + " viable combinations of associations\n\n")

    print("\nMost likely associations:")
    for kv in best[0][0].items():
        print("  - Person %s: %s" % kv)

    groups = [[p] + g[0] for p, g in best[0][0].items()]

    # remove dummy nodes
    groups = [[x for x in g if x != NO_FEATURE] for g in groups]

    # remove groups which contain only one nodes, as that node would be an anonymous person (or a person not associated to any feature)
    nodes_to_remove = [x for g in groups for x in g if len(g) == 1]
    for n in nodes_to_remove:
        if G.nodes[n]["type"] != PERSON:
            print("ERROR! Feature %s not associated to any person!" % n)
        G.remove_node(n)

    groups = [g for g in groups if len(g) > 1]

    subgraphs = []
    for g in groups:
        subG = G.subgraph(g)
        subgraphs.append(subG)

    for i, g in enumerate(groups):
        print("Group %s: %s" % (i, g))
        for n in g:
            G.nodes[n].update({"group": i})

    print("############ STEP 4 ################")
    print("##                                ##")
    print("##      Creation of missing       ##")
    print("##      direct connections        ##")
    print("##                                ##")
    print("####################################")

    print("\n\nTODO!!\n\n")

    resG = nx.Graph(subgraphs[0])
    for subG in subgraphs[1:]:
        resG.update(subG)
    return resG


#
#    print("############ STEP 3 ################")
#    print("##                                ##")
#    print("##      Anonymous persons         ##")
#    print("##                                ##")
#    print("####################################")
#
#    print("\n\nTODO!!\n\n")
#


def transform_weights(w):
    if w <= 0:
        return 1000
    return math.log(1 / w)


def backtransform_weights(w):
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


if __name__ == "__main__":

    G = nx.Graph()

    nodes = {
        "body3": {"type": "body"},
        "body2": {"type": "body"},
        "person2": {"type": "person"},
        "face1": {"type": "face"},
        "person1": {"type": "person"},
        "body1": {"type": "body"},
        "face2": {"type": "face"},
        "voice2": {"type": "voice"},
        "voice1": {"type": "voice"},
        "voice3": {"type": "voice"},
    }

    nodes_color = [get_color(nodes[n]["type"]) for n in nodes.keys()]

    for a, b, w in [
        ("body3", "person2", 0.7),
        ("person2", "face1", 0.9),
        ("face1", "body2", 0.8),
        ("face1", "person1", 0.2),
        ("face1", "body1", 0.1),
        ("voice1", "body2", 0.5),
        ("voice3", "body3", 0.5),
        ("person1", "face2", 0.7),
        ("person1", "body2", 0.81),
        ("body1", "voice2", 0.9),
        ("face2", "body1", 0.6),
    ]:
        G.add_edge(a, b, weight=transform_weights(w), likelihood=w, computed=False)

    for n, attr in nodes.items():
        G.nodes[n].update(attr)

    ###########

    algorithm(G)

    ######################################################################
    ######################################################################
    ######################################################################
    ######################################################################

    fig, ax = plt.subplots(figsize=(12, 12))

    pos = nx.spring_layout(G, weight="likelihood", seed=7)

    # nodes
    nx.draw_networkx_nodes(
        G, pos, nodelist=G.nodes, node_color=[G.nodes[n]["group"] for n in G.nodes]
    )
    nx.draw_networkx_labels(G, pos)
    nx.draw_networkx_edges(G, pos)

    # edge weight labels
    edge_labels = dict(
        [
            (e, backtransform_weights(w))
            for e, w in nx.get_edge_attributes(G, "weight").items()
        ]
    )

    nx.draw_networkx_edge_labels(G, pos, edge_labels)

    # Resize figure for label readibility
    ax.margins(0.1, 0.05)
    fig.tight_layout()
    plt.axis("off")
    plt.show()
