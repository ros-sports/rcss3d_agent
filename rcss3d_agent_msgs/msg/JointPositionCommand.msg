# This is a message that holds data to describe a command for a set of joints.
#
# A command consists of:
# * the goal position of the joint (rad)
# * the PID parameters of the built in controller (if one parameter is given, all must be given)
#   N.B. only the Proportional gain is used in the joint position controller at the moment
#
# Each joint is uniquely identified by its name
#
# This message consists of a multiple arrays, one for each part of the joint state.
# The goal is to make each of the fields optional. When e.g. your joints don't
# have a built in PID controller, or you do not want to update their parameters,
# you can leave their arrays empty.
#
# All arrays in this message should have the same size, or be empty.
# This is the only way to uniquely associate the joint name with the correct
# states.

string[] name
float32[] p_gain
float32[] i_gain
float32[] d_gain
float32[] position
