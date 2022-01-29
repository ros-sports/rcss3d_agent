^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package rcss3d_agent
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.0.4 (2022-01-29)
------------------
* adapt to new changes in cpplint about using a leading "./" in include statements
* Contributors: ijnek

0.0.3 (2022-01-28)
------------------
* follow guidelines for ament library creation
* move all non public hpp files into src directory
* separate rcss3d_agent_node into separate package
* separate JointPos and JointVel for perceptor and effector
* add subscriptions for joints, beam and say
* large refactor to purify rcss3d_agent to be a layer between rcssserver3d and ros2
* update package description
* copied files across from https://gitlab.com/ijnek/ros2_rcss3d/-/tree/kenji-rolling
* add sexpresso submodule
* Contributors: ijnek
