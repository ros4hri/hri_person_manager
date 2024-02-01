# hri_person_manager

A [ROS4HRI](https://wiki.ros.org/hri)-compatible person manager node.
It aggregates information about detected faces, bodies and voices into consistent *persons*,
and exposes these detected persons with their links to their matching face, body, voice.

The aggregation relies on additional ROS nodes to provide candidate 'matches' between persons and/or body parts.
For instance a face recogniser, that would publish candidate matches between faces and unique persons.

## Algorithm

This node collects the candidate matches between features (face, body, voice) or between feature and person.
Then it creates a probablisitic graph from the ones with likelihood higher then a threshold.

Finally it looks for a partition of that graph such that:
- each subgraph is connected
- each subgraph has at most one feature of each type
- the overall likelihood of associations is maximised

The resulting most likely graph is published.

See [doc/ALGORITHM.md](doc/ALGORITHM.md) for details.

## ROS API

### Parameters

All parameters are loaded in the lifecycle `configuration` transition.

- `match_threshold` (default: `0.5`):
  Minimum level of likelihood to consider a face/body/voice to belong to a given person.
- `reference_frame` (default: `map`):
  Persons' TF frames are published with respect to `reference_frame`.
  Typically, faces/bodies/voices  frames are published wrt to their respective sensors frame.
  `hri_person_manager` instead publishes TF frames of humans in `reference_frame`.
  `reference_frame` is usually a 'static' frame (eg `map`), so that if the person moves out of view of the robot
  (and therefore, its position can not be updated anymore), it 'stays' where it was last seen.

### Topics

This package follows the ROS4HRI conventions ([REP-155](https://www.ros.org/reps/rep-0155.html)).
If the topic message type is not indicated, the ROS4HRI convention is implied.

#### Subscribed

- `/humans/candidate_matches`

#### Published

- `/humans/persons/known`
- `/humans/persons/tracked`
- `/humans/graph` ([std_msgs/String](https://github.com/ros2/common_interfaces/blob/humble/std_msgs/msg/String.msg)):
  Most likely graph exoressed in [DOT Language](https://graphviz.org/doc/info/lang.html).
