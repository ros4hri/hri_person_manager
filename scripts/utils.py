import re
import networkx as nx

test = """
``` mermaid
graph LR

body3
face1 ---|0.8| body2
face1 ---|0.2| person1
%%face1 ---|0.1| body1
person1 ---|0.7| face2
person1 ---|0.81| body2
body1 ---|0.9| voice2
face2 ---|0.6| body1
```
"""


def transform_weights(w):
    return w


def mermaid_parser(g):

    G = nx.Graph()

    lines = [l.strip() for l in g.split("\n") if l]
    lines = [
        l
        for l in lines
        if (
            not l.startswith("%%")
            and not l.startswith("```")
            and not l.startswith("graph")
        )
    ]

    for l in lines:

        res = re.match(r"(\w+) ---\|([\d\.]+)\| (\w+)", l)
        if res:
            a, w, b = res.groups()
            w = float(w)

            print("add edge: %s -- %s -- %s" % (a, w, b))
            G.add_edge(a, b, weight=transform_weights(w), likelihood=w, computed=False)
        else:
            res = re.match(r"(\w+)(.*)", l)
            if res:
                a, b = res.groups()
                if a is None or b != "":
                    print("error! malformed line: [%s]" % l)
                else:
                    print("add node: %s" % a)
                    G.add_node(a)

    return G


def test_file_parser(f):

    tests = []

    test = {}
    input_mermaid = ""
    for l in f.readlines():

        if l.startswith("# "):
            if test:
                print("...adding output...")
                test["output"] = mermaid_parser(input_mermaid)
                tests.append(test)
            input_mermaid = ""
            test = {"name": l.split("#")[1].strip()}
            print("\n\nAdd test %s" % test["name"])
            continue

        if l.startswith("## Results:"):
            print("...adding input...")
            test["input"] = mermaid_parser(input_mermaid)
            input_mermaid = ""
            continue

        input_mermaid += l + "\n"

    if test:
        print("...adding output...")
        test["output"] = mermaid_parser(input_mermaid)
        tests.append(test)
    return tests


if __name__ == "__main__":
    import sys

    with open(sys.argv[1], "r") as f:
        tests = test_file_parser(f)

    print(tests)
