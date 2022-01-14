# This is a message that holds data to describe a command for a set of joints.
#
# A command consists of:
# * the goal angular speed of the joint (rad/s)
#
# Each joint is uniquely identified by its name
#
# Both arrays in this message should have the same size, or be empty.
# This is the only way to uniquely associate the joint name with the
# correct states.

string[] name
float32[] speed
