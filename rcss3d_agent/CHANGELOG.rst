^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package rcss3d_agent
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.0.6 (2022-02-04)
------------------
* add missing include for std::optional
* add effector msg to allow synchronize effector to be used
* Contributors: ijnek, Scott K Logan

0.0.5 (2022-02-03)
------------------
* add score left and score right to message received from simulator
* don't rely on the order of items in sexp. Use their path. This prevents buggy code when the sexp is updated on the simulator end in the future.
* comunicate that the node has to be restarted
* make error msg easier to understand
* make sure program can exit when connection to simulator is broken
* add a model parameter so differnet types of robots can be loaded
* don't allow sending empty say messages
* add team name to hear msg
* fix bug in gamestate parsing
* Contributors: ijnek

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
