# Information about the change in orientation of a body. The message contains the name of the body
# to which the gyro perceptor belongs and three rotation angles. These rotation angles describe the
# change rates in orientation of the body during the last cycle. In other words the current angular
# velocities along the three axes of freedom of the corresponding body in degrees per second.
# To keep track of the orientation of the body, the information to each gyro rate perceptor is sent
# every cycle.

string name  # The name of the body part the gyro rate perceptor is attached to
float32 x  # The current angular velocities along the x axis in degrees per second
float32 y  # The current angular velocities along the y axis in degrees per second
float32 z  # The current angular velocities along the z axis in degrees per second