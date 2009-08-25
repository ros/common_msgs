GoalID goal_id
uint8 status
uint8 PENDING         = 0
uint8 ACTIVE          = 1
uint8 PREEMPTED       = 2
uint8 SUCCEEDED       = 3
uint8 ABORTED         = 4
uint8 REJECTED        = 5
uint8 PREEMPTING      = 6
uint8 RECALLING       = 7
uint8 RECALLED        = 8

# An action client can determine that a goal is LOST. This should not be sent over the wire.
uint8 LOST            = 9

#Allow for the user to associate a string with GoalStatus for debugging
string text  
