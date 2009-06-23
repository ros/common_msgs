# This message defines the expected format for Controller Statuss messages
# Embed this in the feedback state message of highlevel controllers
byte UNDEFINED=0
byte SUCCESS=1
byte ABORTED=2
byte PREEMPTED=3
byte ACTIVE=4

# Status of the controller = {UNDEFINED, SUCCESS, ABORTED, PREEMPTED, ACTIVE}
byte value

#Comment for debug
string comment