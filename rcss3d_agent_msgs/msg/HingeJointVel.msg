# Effector for all axis with a single degree of freedom.
# name - name of the axis.
# ax - a speed value, passed in radians per second.
# Setting a speed value on a hinge means that the speed will be maintained until a new value is provided. 
# Even if the hinge meets its extremity, it will bounce around at the extremity until a new
# speed value is requested.

string name  # The name of the corresponding hinge joint
float32 ax  # The angular velocity in radians per second