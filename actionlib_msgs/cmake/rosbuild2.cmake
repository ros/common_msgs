
macro(rosbuild_actions OUTPUT_VAR)

  set(${OUTPUT_VAR} "")

  set(_autogen "")
  set(_autogen_msg_list "")
  foreach(_action ${ARGN})
    # message(STATUS "Generating Messages for Action" ${_action})
    #construct the path to the .action file
    set(_input ${PROJECT_SOURCE_DIR}/${_action})

    get_filename_component(_actionfile_only ${_action} NAME)
      
    string(REPLACE ".action" "" _action_bare ${_actionfile_only})

    set(genaction_exe ${actionlib_msgs_SOURCE_DIR}/genaction-rosbuild2.py)

    #We have to do this because message generation assumes filenames without full paths
    set(_base_output_action ${_action_bare}Action.msg)
    set(_base_output_goal ${_action_bare}Goal.msg)
    set(_base_output_action_goal ${_action_bare}ActionGoal.msg)
    set(_base_output_result ${_action_bare}Result.msg)
    set(_base_output_action_result ${_action_bare}ActionResult.msg)
    set(_base_output_feedback ${_action_bare}Feedback.msg)
    set(_base_output_action_feedback ${_action_bare}ActionFeedback.msg)
   
    set(_output_dir ${CMAKE_CURRENT_BINARY_DIR}/msg)
    set(_output_action ${_output_dir}/${_base_output_action})
    set(_output_goal ${_output_dir}/${_base_output_goal})
    set(_output_action_goal ${_output_dir}/${_base_output_action_goal})
    set(_output_result ${_output_dir}/${_base_output_result})
    set(_output_action_result ${_output_dir}/${_base_output_action_result})
    set(_output_feedback ${_output_dir}/${_base_output_feedback})
    set(_output_action_feedback ${_output_dir}/${_base_output_action_feedback})

    add_custom_command(
      OUTPUT ${_output_action} ${_output_goal} ${_output_action_goal} 
      ${_output_result} ${_output_action_result} ${_output_feedback} 
      ${_output_action_feedback} 
      COMMAND ${ROSBUILD_SUBSHELL} ${genaction_exe} 
      ${_input}
      -o ${_output_dir}
      DEPENDS ${_input} ${genaction_exe} ${ROS_MANIFEST_LIST}
      COMMENT "Generating messages from action ${_input}"
    )
    list(APPEND _autogen 
      ${_output_action} ${_output_goal} ${_output_action_goal} ${_output_result} 
      ${_output_action_result} ${_output_feedback} ${_output_action_feedback}
      )
    list(APPEND _autogen_msg_list 
      ${_base_output_action} ${_base_output_goal} ${_base_output_action_goal} ${_base_output_result} 
      ${_base_output_action_result} ${_base_output_feedback} ${_base_output_action_feedback})
    list(APPEND ${OUTPUT_VAR} "${_output_action};${_output_goal};${_output_action_goal};${_output_result};${_output_action_result};${_output_feedback};${_output_action_feedback}")

    # message(STATUS ${_autogen_msg_list})

  endforeach(_action)

endmacro()
