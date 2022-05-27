^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package hri_person_manager
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

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
