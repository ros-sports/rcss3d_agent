# Agent processes are not allowed to communicate with each other directly, but agents may exchange
# messages via the simulation server. For this purpose agents are equipped with the so-called hear
# perceptor, which serves as an aural sensor and receives messages shouted by other players.
#
# See restrictions: https://gitlab.com/robocup-sim/SimSpark/-/wikis/Perceptors#restrictions

string team  # The team of the player who sent the message
float32 time  # The simulation time at which the given message was heard in seconds.
bool self  # Whether the player is hearing their own words.
float32[<=1] direction  # Direction of heard message, if the message wasn't the player's own words.
                        # Field has zero size if the "self" field is true.
string message  # Up to 20 characters, which may be taken from the ASCII printing character subset
                # [0x20, 0x7E] except the white space character and the normal brackets ( and )