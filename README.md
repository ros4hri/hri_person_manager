hri_person_manager
==================

`hri_person_manager` is a ROS node, part of the ROS4HRI framework. It 
aggregates information about detected faces, bodies and voices into consistent
*persons*, and exposes these detected persons with their links to their 
matching face, body, voice.

The aggregation relies on additional ROS nodes to provide candidate 'matches'
between persons and/or body parts. For instance a face recogniser, that would
publish candidate matches between faces and unique persons.

`hri_person_manager` is part of the ROS4HRI framework, and follows the 
[ROS REP-155](https://www.ros.org/reps/rep-0155.html).

Installation
------------

### Dependencies

- Boost Graph library
- `libhri` and `hri_msgs`

### Installation

Compile & install the node like any standard ROS node (eg, use `catkin
build`).

It can be installed manually, by running the following set
of commands:

```
$ cd ros_ws/src
$ git clone https://github.com/ros4hri/hri_person_manager.git
$ cd hri_person_manager
$ mkdir build && cd build
$ cmake -DCMAKE_BUILD_TYPE=Release -DCMAKE_INSTALL_PREFIX=/opt/ros/noetic .. && make && make install
```

(of course, replace the `INSTALL_PREFIX` by your desired prefix)

Algorithm
---------

This node subscribes to the `/humans/candidate_matches` topic (of type
[hri_msgs/IdsMatch.msg](https://github.com/ros4hri/hri_msgs/blob/master/msg/IdsMatch.msg)
and creates a probablisitic graph with each detected features (face, body,
voice) or persons. Features are considered to be linked if the likelihood of
their connection is above the `humans/match_threshold` parameter.

It then looks for a partition of that graph such that:
- each subgraph is connected
- each subgraph has at most one feature of each type
- the overall likelihood of associations is maximised

See [doc/ALGORITHM.md](doc/ALGORITHM.md) for details.

Parameters
----------

- `/humans/match_threshold` (`float`, default: 0.5): the minimum level of
  likelihood to consider a face/body/voice to belong to a given person.

- `/humans/reference_frame` (`string`, default: `map`): persons' TF frames are
  published with respect to `reference_frame`. Typically, faces/bodies/voices
  frames are published wrt to their respective sensors frame.
  `hri_person_manager` instead publishes TF frames of humans in `reference_frame`.
  `reference_frame` is usually a 'static' frame (eg `map`), so that if the
  person moves out of view of the robot (and therefore, its position can not be
  updated anymore), it 'stays' where it was last seen.

- `/humans/robot_reference_frame` (`string`, default: `base_link`): used to
  compute the distance between a person and the robot. Ideally, a frame attached
  to the robot, at ~people head height.

- `/humans/proxemics/{personal,social,public}_distance` (`float`, default: 1.2m
  for `personal_distance`, 3.6m for `social_distance`, 20m for
  `public_distance`): defines the range of the 3 main proxemics zone: the
  *personal space* goes from 0m to `personal_distance`, the *social space* from
  `personal_distance` to `social_distance`, and the *public space* from
  `social_distance` to `public_distance`. Persons detected beyond the
  `public_space` distance are not published. Set `public_distance` to 0m to
  ignore the maximum public space distance and always publish all the people.

- `~features_from_matches` (`bool`, default: `true`): if set to true, features
  appearing in a `candidate_matches` message are assumed to be currently
  tracked, and are added to the features-person graph. If set to false, only
  features published on `/humans/*/tracked` are considered tracked, and
  `candidate_matches` for untracked features will be ignored.

Published topics
----------------

- `/humans/persons/tracked` (type: `hri_msgs/IdList`): list of the persons
  currently tracked (ie seen or heard)
- `/humans/persons/known` (type: `hri_msgs/IdList`): list of the persons
  that were or are tracked at some point
- `/humans/persons/in_{personal,social,public}_space` (type: `hri_msgs/IdList`):
  list of persons currently tracked and in the corresponding proxemics space (cf
  parameters above for the distnace ranges of each zone)

- `/humans/persons/<person id>/...`: refer to [the REP-155](https://www.ros.org/reps/rep-0155.html#persons) for the list of topics and their semantics


### Proxemics

In addition to the standard REP-155 topics, `hri_person_manager` also publishes
proxemics:

- `/humans/persons/in_{personal|social|public}_space` (type:
  `hri_msgs/IdsList`): list of the persons currently in each proxemics zone
  (based on the ranges defined in the parameter section above)
- `/humans/persons/<person id>/proxemic_space` (type: `std_msgs/String`): one of
  `unknown`, `personal`, `social`, `public` (based on the ranges defined in the
  parameter section above)

### Debugging/introspection

- `/humans/graph` (type: `std_msgs/String`): a graph of the current
  probablisitic 'feature associations' network, in GraphViz format. Useful for
  debugging and can be plotted using the GraphViz tool.


