# Perceptors are the senses of an agent, allowing awareness of the agent's model state and the
# environment. The server sends perceptor messages to agents, via the network protocol, for every
# cycle of the simulation. Details of each message type are explained in each msg file.

GyroRate[] gyro_rates
HingeJointPos[] hinge_joints
UniversalJointPos[] universal_joints
ForceResistance[] force_resistances
Accelerometer[] accelerometers
Vision[<=1] vision
GameState game_state
AgentState[<=1] agent_state
Hear[] hears