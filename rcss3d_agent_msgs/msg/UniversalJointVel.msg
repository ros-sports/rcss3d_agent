# Effector for all axis with a two degrees of freedom. The first parameter is the name of the axis.
# The second and third parameters contain the change in angle of the two joints. The order of the
# joints is the same as in the name.

string name  # The name of the corresponding hinge joint
float32 ax1  # The angular velocity of the first axis in radians per second
float32 ax2  # The angular velocity of the second axis in radians per second