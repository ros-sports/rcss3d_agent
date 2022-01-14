# Due to how robot observations are sent in SimSpark,
# this data message uses the "head" limb of the robot.

std_msgs/Header header
rcss3d_agent_msgs/Spherical head
string team  # (Optional) Team Name, empty string indicated "unknown"
int32 id  # (Optional) Player Id, 0 indicates "unknown"