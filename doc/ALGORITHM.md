# Person matching algorithm

## Definitions and requirements

**DEFINITION 1**: a *feature* is a face, body, or voice. Accordingly, the *feature
type* is one of *face*, *body* or *voice*.

*Note: these 3 features happen to be ones currently specified in the ROS4HRI
specification. Additional features could be added -- nothing in the algorithm is
specific to these three ones.*

**DEFINITION 2**: a *node* is either a feature or a person, and accordingly *node
types* are one of face, body, voice or person.

**DEFINITION 3**: *nodes* (eg either persons or features) might be *connected* to
each other if a likelihood of association has been published between them (via
the `/humans/candidate_matches` topic). Together they span a graph called the
*person features graph*.

**DEFINITION 4**: A *permissible path* between two nodes is a path that does not
contain more than one node of each type (including the start and end nodes). The
likelihood of that path is computed as the product of likelihoods along the
path.

**DEFINITION 5**: A node can be *associated* to  another one, as long as:
- it is not already associated to a node of the same type
- there exists a permissible path between the two nodes
- the likelihood of the resulting path is above the *likelihood
  threshold* (this threshold can be configured via the ROS parameter
  `/humans/match_threshold`).

**DEFINITION 6**: an *association* is a connected set of nodes, such that each
node in the set is *associated* to the other nodes, as defined in **DEFINITION
5**.

**DEFINITION 7**: the association *affinity* of a person is the _sum_ of
the likehoods on the maximum spanning tree of an association.

**REQUIREMENT 1: maximum likelihood**: associations between persons and their
features must be chosen so that they maximize the total affinity of all
associations.

**REQUIREMENT 2: existence of persons**: at anytime, every features must be
associated to exactly one person (anonymous or not).

**REQUIREMENT 3: anonymous persons**: as a consequence of **REQUIREMENT 2**,
features that are not associated to a person must be associated to temporary
'anonymous' persons. This should still respect the requirements of **DEFINITION
5**: if two features are associated (eg a face and a body), they must be
associated to the same anonymous person.

**REQUIREMENT 4: stability**: associations with indirect paths (eg `A`
associated to `C` because `A` connected to `B` and `B` connected to `C`) should
be maintained in case of the path is broken (eg `B` disappears) *if and only if*
the likelihood of the association between the two nodes is superior to the
likelihood threshold when the path is broken. In that case, a new *direct*
association must be created and and the likelihood of that association will
remain constant until the association is removed (because one of the two nodes
disappeared).

## Working example

The following diagram represents a possible likelihood graph, built over time be
receiving 'matches' between features and persons:

``` mermaid
graph LR

body3:::body ---|0.7| person2
person2:::person ---|0.9| face1
face1:::face ---|0.8| body2:::body
face1 ---|0.2| person1
face1 ---|0.1| body1:::body
person1:::person ---|0.7| face2
person1 ---|0.81| body2
body1 ---|0.9| voice2:::voice
face2:::face ---|0.6| body1
voice3:::voice ---|0.5| body3
voice1:::voice ---|0.5| body2

classDef person fill:#d74
classDef voice fill:#47d
classDef body fill:#5b2
classDef face fill:#d47
```

The purpose of the algorithm is to produce this set of 3 associations (we assume
here a likelihood threshold of 0.4):

``` mermaid
graph LR

body3:::body ---|0.7| person2
person2:::person ---|0.9| face1:::face
voice3:::voice ---|0.5| body3

body2:::body ---|0.81| person1
voice1:::voice ---|0.5| body2
person1:::person ---|0.7| face2:::face
person1 ---|0.405| voice1

anonymous_person1:::person ---|0.4| body1
body1:::body ---|0.9| voice2
voice2:::voice ---|0.4| anonymous_person1

classDef person fill:#d74
classDef voice fill:#47d
classDef body fill:#5b2
classDef face fill:#d47
```

Note that one anonymous person had to be created for the features `body1` and
`voice2` that would otherwise be dangling.

Note also that the connection between `person1` and `voice1` did not orginally
exist: it is a *computed* edge, added to ensure the relation `person1 <-->
voice1` is maintained if `body2` disappears.

## Algorithm

### Main algorithm *computeAssociations*

Given a probablisitic relation graph,

1. prune all edges whose likelihood is below `threshold`
1. remove all existing anonymous persons
1. for each resulting connected component of the graph:
   1. generate the list of all possible partitions of the graph, such that:
      * each partition contains at most one node of each type
      * the partition is connected
   1. filter partitions to only keep to fully connected ones
   1. find the minimum partition size (ie, the minimum possible number of associations)
   1. select all the partitions of that size
   1. amongst these partitions, select the one of highest affinity
   1. add missing persons as *anonymous persons*
   1. create direct connections between the person and its features within each
      association (see *fullyConnectPersons* algorithm below)
1. return the union of these best partitions over all connected components 


### Algorithm *fullyConnectPersons*

Add direct links between a person and its features, iff:
- they are not already directly connected
- the likelihood of the connection (computed as the product of
  likelihoods along the shortest path) is above the set likelihood
   `threshold`.

### Algorithm

1. find the person node of the association
1. *temporarily remove* previously computed edges to the person,
1. compute the shortest path between the person and each of the features using
   the *log* of the likelihoods, so that the path length is the resulting
   probability of the association (product of the likelihoods)
1. if the person and the feature are not directly connected, create a new
   *computed* edge with the corresponding likelihood.
1. re-add the previously removed edges, if they have not been re-computed at
   step 4. This ensure that features that were disconnected from the rest of the
   graph when removing the edge (because they only belonged to the association
   due to a previously computed edge) are brought back.
