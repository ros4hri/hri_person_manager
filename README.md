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

- `match_threshold` (`double > 0`, default: `0.5`):
  Minimum level of likelihood to consider a face/body/voice to belong to a given person.
- `reference_frame` (`string`, default: `map`):
  Persons' TF frames are published with respect to `reference_frame`.
  Typically, faces/bodies/voices  frames are published wrt to their respective sensors frame.
  `hri_person_manager` instead publishes TF frames of humans in `reference_frame`.
  `reference_frame` is usually a 'static' frame (eg `map`), so that if the person moves out of view of the robot
  (and therefore, its position can not be updated anymore), it 'stays' where it was last seen.
- `robot_reference_frame` (`string`, default: `base_link`):
  Used to compute the distance between a person and the robot.
  Ideally, a frame attached to the robot, at ~people head height.
- `{personal|social|public}_distance` (`double`, default: {`1.2`|`3.6`|`20.`}m):
  Define the range of the 3 main proxemics zone:
  - *personal space* goes from 0m to `personal_distance`;
  - *social space* from `personal_distance` to `social_distance`;
  - *public space* from `social_distance` to `public_distance`.
  Persons detected beyond the `public_space` distance are not published.
  Set `public_distance` to 0m to ignore the maximum public space distance and always publish all the people.

### Topics

This package follows the ROS4HRI conventions ([REP-155](https://www.ros.org/reps/rep-0155.html)).
If the topic message type is not indicated, the ROS4HRI convention is implied.

#### Subscribed

- `/humans/candidate_matches`

#### Published

- `/humans/persons/known`
- `/humans/persons/tracked`
- `/humans/persons/in_{personal|social|public}_space` ([hri_msgs/IdList](https://github.com/ros4hri/hri_msgs/blob/humble-devel/msg/IdsList.msg)):
  List of persons currently tracked and in the corresponding proxemics space
  (cf. parameters above for the distance ranges of each zone).
- `/humans/persons/<person_id>/proxemic_space` ([std_msgs/String](https://github.com/ros2/common_interfaces/blob/humble/std_msgs/msg/String.msg)):
  One of `unknown`, `personal`, `social`, `public`
  (based on the ranges defined in the parameter section above).
- `/humans/graph` ([std_msgs/String](https://github.com/ros2/common_interfaces/blob/humble/std_msgs/msg/String.msg)):
  A graph of the current probabilistic 'feature associations' network, in [DOT Language](https://graphviz.org/doc/info/lang.html).
  It can be used for debugging/visualization purposes.

## Visualization

The current graph can be exported on PDF using:
`ros2 run hri_person_manager show_humans_graph <filename>`
