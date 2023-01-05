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
ROS REP-155.

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

