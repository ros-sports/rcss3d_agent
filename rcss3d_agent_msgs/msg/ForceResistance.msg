# The force that acts on a body. [px, py, pz] describes the point of origin relative to the body
# itself and [fx, fy, fz] describes the resulting force on this point. The two vectors are just
# an approximation about the real applied force. The point of origin is calculated as weighted
# average of all contact points to which the force is applied, while the force vector represents
# the total force applied to all of these contact points. The information to a force resistance
# perceptor is just sent in case of a present collision of the corresponding body with another
# simulation object. If there is no force applied, the message of this perceptor is omitted.

string name  # The name of the body, to which the force resistance perceptor belongs
float32 px  # The local x coordinate of the origin of the applied force in meters
float32 py  # The local y coordinate of the origin of the applied force in meters
float32 pz  # The local z coordinate of the origin of the applied force in meters
float32 fx  # The x component of the force vector in newton (kg m/s2)
float32 fy  # The y component of the force vector in newton (kg m/s2)
float32 fz  # The z component of the force vector in newton (kg m/s2)