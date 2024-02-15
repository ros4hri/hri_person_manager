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

### Launch

```
ros2 launch hri_person_manager person_manager.launch.py
```

The `person_manager.launch.py` launch file accepts as arguments and configures all the [parameters](#parameters).
It also automatically transitions the node to the active state.

## Visualization

The current graph can be exported on PDF using:
`ros2 run hri_person_manager show_humans_graph <filename>`

## Example

For an example of usage, execute in *different* terminals the following commands:
- USB camera:
  1. `apt install ros-humble-usb-cam`
  2. `ros2 run usb_cam usb_cam_node_exe --ros-args -p camera_info_url:="file:///`pwd`/sample_camera_calibration.yml`

  You **must** provide a calibration file for your camera,
  otherwise `hri_face_detect` will not be able to publish the TF frames of the faces
  (note that the name of the camera in the calibration file should be `default_cam`,
  as it will assume this name for the camera TF frame).
  Without calibration, `hri_person_manager` will still partially work,
  but e.g. proximal zones will not be published, and the person TF frames will not be available.

- HRI face detect:
  1. Either
    - if you are on a PAL robot `apt install ros-humble-hri-face-detect`
    - otherwise build and install from [source](https://github.com/ros4hri/hri_face_detect).
  2. `ros2 launch hri_face_detect face_detect.launch.py`
- HRI fullbody:
  1. Either
    - if you are on a PAL robot `apt install ros-humble-hri-fullbody`
    - otherwise build and install from [source](https://github.com/ros4hri/hri_face_fullbody).
  2. `ros2 launch hri_fachri_fullbody hri_fullbody.launch.py`
- HRI person manager:
  1. Either
    - if you are on a PAL robot `apt install ros-humble-hri-person-manager`
    - otherwise build and install from [source](https://github.com/ros4hri/hri_person_manager).
  2. `ros2 launch hri_person_manager person_manager.launch.py`
- Static transform for `map`:
  1. `ros2 run tf2_ros static_transform_publisher --frame-id map --child-frame-id default_cam`
- Check the output topics (`/humans/persons/tracked`, `/humans/persons/in_{personal,social,public}_space`):
  1. `ros2 topic echo <topic_name>`

  For instance, in the following example you should see your own person ID:

  ```
  ros2 topic echo /humans/persons/in_personal_space
  ```

- Export the person association graph
  1. `ros2 run hri_person_manager show_humans_graph`
  2. `evince /tmp/graph.pdf`

