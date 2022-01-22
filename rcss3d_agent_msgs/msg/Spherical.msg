# Spherical msg used for soccer vision perceptors. All distances and angles are given
# relative to the position and orientation of the camera.
#
# Details: https://gitlab.com/robocup-sim/SimSpark/-/wikis/Perceptors#vision-perceptors
#
# Angles are stored in degrees.

# The distance to the seen object, in metres.
float32 r

# The horizontal angle (φ) to the visible object relative to the view direction in degrees
# and an accuracy of two digits. Positive angles are left of the vertical midline of the
# agent's vision, negative angles right.
float32 theta

# The vertical angle (θ) to the visible object relative to the view direction in degrees
# and an accuracy of two digits. Positive angles mean the seen object is above the
# horizontal midline of agent's vision, negative means below.
float32 phi