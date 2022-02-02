hri_person_manager
==================

`hri_person_manager` is a ROS node, part of the ROS4HRI framework. It 
aggregates information about detected faces, bodies and voices into consistent
*persons*, and exposes these detected persons with their links to their 
matching face, body, voice.

The aggregation relies on additional *matching* nodes to be available and 
running, for instance a face recogniser to uniquely associate a face to a
person.

`hri_person_manager` is part of the ROS4HRI framework, and follows the 
ROS REP-155.

