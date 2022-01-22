# An observed player is described through the name of it's team, the player number and one or more
# position vectors to different body parts.

string team  # The player number of the seen player
int32 id  # The player number of the seen player
Spherical[<=1] head  # Observation of head
Spherical[<=1] rlowerarm  # Observation of right lower arm
Spherical[<=1] llowerarm  # Observation of left lower arm
Spherical[<=1] rfoot  # Observation of right foot
Spherical[<=1] lfoot  # Observation of left foot