# Contains several information about the actual state of the soccer game environment. The actual
# play time and play mode are transmitted in each cycle. Play time starts from zero at kickoff and
# is given as a floating point number in seconds, to two decimal places.

float32 time  # The current play time in seconds
string playmode  # The current play mode of the soccer game (eg. BeforeKickOff)
int32 score_left  # The score of the left team
int32 score_right  # The score of the right team