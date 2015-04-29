^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package carl_moveit
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.0.15 (2015-04-29)
-------------------
* Merge branch 'develop' of github.com:WPI-RAIL/carl_moveit into develop
* Added feedback to arm actions, fixed a bug with pickup for objects in the map frame
* Contributors: David Kent

0.0.14 (2015-04-27)
-------------------
* Merge branch 'develop' of github.com:WPI-RAIL/carl_moveit into develop
* minor fixes
* Contributors: David Kent

0.0.13 (2015-04-22)
-------------------
* changelog reverted
* Merge branch 'develop' of github.com:WPI-RAIL/carl_moveit into develop
* changelog updated
* Added threshold for attaching scene objects to the robot end-effector
* Debug statement cleanup
* Approach angle fix
* Connected pickup action with scene object attaching
* Added fingers as allowable touch links for scene object attachment
* Merge branch 'develop' of github.com:WPI-RAIL/carl_moveit into develop
* Support for scene collision objects from segmented/recognized object data
* Contributors: David Kent, Russell Toris

0.0.12 (2015-04-14)
-------------------
* Scene object adjustment
* bug fix
* moveit planning scene collision object test
* Contributors: David Kent

0.0.11 (2015-04-03)
-------------------
* Removed some unnecessary output
* Clear octomap after the jaco arm homes using the kinova api
* Enabled clear_octomap service
* Contributors: David Kent

0.0.10 (2015-03-31)
-------------------
* Minor action renaming
* Updated method documentation
* Added grasp_joint to the arm planning group, bugfixes on pickup trajectory execution
* Added pickup action, changed end effector link to use jaco_link_eef instead of jaco_link_hand
* Contributors: David Kent

0.0.9 (2015-03-27)
------------------
* Merge branch 'develop' of github.com:WPI-RAIL/carl_moveit into develop
* Changed the ready arm action to a general arm actions server which can currently either READY or RETRACT the arm
* Contributors: David Kent

0.0.8 (2015-03-24)
------------------
* Updated IK and MoveToPose to use stamped poses instead of base_footprint frame poses
* Removed run dependency on moveit visualization
* Update CMakeLists.txt
* Update .travis.yml
* lift server start
* Planning-based hand lift
* Output message when arm is already retracted
* retract bug fix
* Added check if arm is already retracted before performing a home->retract action
* Cancel retract if home fails
* Better failure detection on joint pose goal planning and moving
* Merge branch 'develop' of github.com:WPI-RAIL/carl_moveit into develop
* Error code debugging on failed moveToPose
* Contributors: David Kent, Russell Toris

0.0.7 (2015-02-17)
------------------
* Documentation
* Common action preemtion during retract
* Merge branch 'develop' of github.com:WPI-RAIL/carl_moveit into develop
* Common action preempting
* Update .travis.yml
* Contributors: David Kent, Russell Toris

0.0.6 (2015-02-06)
------------------
* Updated launch file; changed preempted ready action behavior
* Common motion planning actions, such as readying and retracting the JACO arm
* Contributors: David Kent

0.0.5 (2015-01-21)
------------------
* Merge branch 'develop' of github.com:WPI-RAIL/carl_moveit into develop
* Tuned jump threshold for Cartesian movement
* Update .travis.yml
* Update .travis.yml
* Update .travis.yml
* Update .travis.yml
* Contributors: David Kent, Russell Toris

0.0.4 (2015-01-16)
------------------
* Launched carl_moveit_wrapper with full launch file
* Update .travis.yml
* Added Cartesian path planning, experimental Cartesian control using a Jacobian pseudoinverse, and some general cleanup
* Contributors: David Kent

0.0.3 (2014-12-02)
------------------
* Merge pull request #1 from WPI-RAIL/master
  Merge
* Merge branch 'master' of github.com:WPI-RAIL/carl_moveit
* minor changes with collisions
* Contributors: David Kent, Russell Toris

0.0.2 (2014-11-03)
------------------
* missing dep added back
* Contributors: Russell Toris

0.0.1 (2014-10-31)
------------------
* mongo fix
* travis test
* cleanup for release
* updates for pick and place
* initial commit
* Contributors: Russell Toris, dekent
