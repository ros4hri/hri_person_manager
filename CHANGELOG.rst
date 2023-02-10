^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package hri_person_manager
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Forthcoming
-----------
* generate stable anonymous person ID, based on feature's name
* fix wrong early return when assigning random IDs
* Contributors: Séverin Lemaignan

1.0.3 (2023-02-10)
------------------
* add ~features_from_matches parameter to control how features a created
  `~features_from_matches` (`bool`, default: `true`): if set to true, features
  appearing in a `candidate_matches` message are assumed to be currently
  tracked, and are added to the features-person graph. If set to false, only
  features published on `/humans/*/tracked` are considered tracked, and
  `candidate_matches` for untracked features will be ignored.
* fix 829ca7843 accidentally prevented new features to be created
* Revert "ensure removing nodes takes place *after* possible other updates"
  This reverts commit 41499982edbea97c6ed977eab05f0e2be86fd762.
  Changing the order of updates to the model would break invariants.
  Better not to do that.
* Contributors: Séverin Lemaignan

1.0.2 (2023-02-03)
------------------
* ensure removing nodes takes place *after* possible other updates
  It might happen that in the same hri_person_manager update cycle, a feature is removed and a update between that feature and another one is received. If the updated arrives just after the deletion, the feature will be immediately re-created. We now perform all the deletions *last* in an update cycle.
  In some rare situation where the same feature (eg a face) is deleted and actually purposefully re-created with one hri_person_manager update cycle (100ms by default), this change will lead to undesired behaviours (the feature will not be re-created)
* do not re-create nodes when the candidate_match likelihood is 0
* ensure the 'show_graph' script can be executed
* Contributors: Séverin Lemaignan

1.0.1 (2023-01-05)
------------------
* documentation of the new algorithm
* fix shadowing issues
* Contributors: Séverin Lemaignan

1.0.0 (2023-01-04)
------------------

Major rewrite of the core algorithm.

General algorithm description:

Consider a connected graph containing red, blue, yellow and green nodes,
connected by weighted edges. Write a python script that (1) generates
all possible graph partitions where each subgraph is connected and
contains at most one node of each color, (2) select the partitions with
the least subgraphs, (3) amongst these, select the partition that
minimize the total sum of weights in each subgraph's minimum spanning
tree.

* color the graphviz output per association
* minor
* fix/update the unit tests
* publish the timestamps for the list of known/tracked persons
* minor UX to the show-graph script
* clean up + minor optimisation/bugfix in the ROS node
* add mermaid-based tests to the unit-test suite
* add logic to re-use anonymous person ID when same features are visible
* C++ impl complete, based on boost::graph
  all mermaid tests pass with C++ version
  Note to myself: I *hate* boost::graph.
* Python impl complete, all mermaid-based tests pass
* add small utility to parse mermaid syntax in python graphs
* WIP on my algorithm, Python impl
* Contributors: Séverin Lemaignan

0.3.0 (2022-10-14)
------------------
* track /h/{f|b|v}/tracked to automatically create/remove anonymous persons
* use Boost graph bundled properties to store vertex ids
  Until now, ID where stored in a separate map. This would cause major issues when
  a node was removed as Boost would re-assign vertex ids that would not match anymore
  the ID that we had stored.
* publish dot graph of humans on /humans/graph
* more tests -- tests do not pass for a strange reason, need investigating
* add accessor to set/get a person's alias
* anonymous_persons can be removed by publishing a candidate match with confidence=0
* [minor] improved console logging behaviour
* Contributors: Séverin Lemaignan

0.2.4 (2022-07-12)
------------------
* missing 'break' in a switch leading to mis-handling anonymous persons
* Contributors: Séverin Lemaignan

0.2.3 (2022-06-01)
------------------
* update to new IdsMatch.msg
* Contributors: Séverin Lemaignan

0.2.2 (2022-06-01)
------------------
* attempting to fix unit-test compilation error on ferrum
* Contributors: Séverin Lemaignan

0.2.1 (2022-05-27)
------------------
* fix variable shadowing/initialisation order
* add missing dep on std_srvs
* Contributors: Séverin Lemaignan

0.2.0 (2022-05-27)
------------------
* increase candidate_matches subscriber queue to ensure no message missed
* large refactor, moving 'managed persons' to their own class
  While here:
  - added a /hri_person_manager/reset service to clear all existing
  associations;
  - updated unit-tests to latest libhri (0.5.0)
* make reference tf frame a parameter /humans/reference_frame
* Publish the person's tf frame + /location_confidence
  While here, encapsulate code in a PersonManager class
* support anonymous persons, ie persons that we are aware of because a face,
  body, voice has been detected, but that is not yet identified.
* publish separately /humans/persons/tracked (actively tracked) and /humans/persons/known
* Contributors: Séverin Lemaignan

0.1.0 (2022-03-06)
------------------
* ROS wrapper + test of ROS node
* complete implementation of PersonMatcher algo. Tests pass.
* Contributors: Séverin Lemaignan
