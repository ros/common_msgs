GoalID goal_id
uint8 status
uint8 PENDING         = 0   # The goal has yet to be processed by the action server
uint8 ACTIVE          = 1   # The goal is currently being processed by the action server
uint8 PREEMPTED       = 2   # The goal was preempted by either another goal, or a preempt
                            #    message being sent to the action server (Terminal State)
uint8 SUCCEEDED       = 3   # The goal was achieved successfully by the action server
uint8 ABORTED         = 4   # The goal was aborted by the action server (Terminal State)
uint8 REJECTED        = 5   # The goal was rejected by the action server without being processed (Terminal State)
                            #    and without a request from the action client to cancel (Terminal State)
uint8 PREEMPTING      = 6   # Processing of the goal was canceled by either another goal,
                            #    or a cancel request sent to the action server
uint8 RECALLING       = 7   # The goal has not been processed and a cancel request has been received from
                            #    the action client, but the action server has not confirmed the goal is canceled
uint8 RECALLED        = 8   # The goal was canceled by either another goal, or a cancel request before the
                            #    action server began processing the goal. (Terminal State)
uint8 LOST            = 9   # An action client can determine that a goal is LOST. This should not be
                            #    sent over the wire by an action server

#Allow for the user to associate a string with GoalStatus for debugging
string text  
