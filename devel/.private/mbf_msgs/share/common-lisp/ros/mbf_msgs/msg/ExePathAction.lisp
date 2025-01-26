; Auto-generated. Do not edit!


(cl:in-package mbf_msgs-msg)


;//! \htmlinclude ExePathAction.msg.html

(cl:defclass <ExePathAction> (roslisp-msg-protocol:ros-message)
  ((action_goal
    :reader action_goal
    :initarg :action_goal
    :type mbf_msgs-msg:ExePathActionGoal
    :initform (cl:make-instance 'mbf_msgs-msg:ExePathActionGoal))
   (action_result
    :reader action_result
    :initarg :action_result
    :type mbf_msgs-msg:ExePathActionResult
    :initform (cl:make-instance 'mbf_msgs-msg:ExePathActionResult))
   (action_feedback
    :reader action_feedback
    :initarg :action_feedback
    :type mbf_msgs-msg:ExePathActionFeedback
    :initform (cl:make-instance 'mbf_msgs-msg:ExePathActionFeedback)))
)

(cl:defclass ExePathAction (<ExePathAction>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <ExePathAction>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'ExePathAction)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name mbf_msgs-msg:<ExePathAction> is deprecated: use mbf_msgs-msg:ExePathAction instead.")))

(cl:ensure-generic-function 'action_goal-val :lambda-list '(m))
(cl:defmethod action_goal-val ((m <ExePathAction>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader mbf_msgs-msg:action_goal-val is deprecated.  Use mbf_msgs-msg:action_goal instead.")
  (action_goal m))

(cl:ensure-generic-function 'action_result-val :lambda-list '(m))
(cl:defmethod action_result-val ((m <ExePathAction>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader mbf_msgs-msg:action_result-val is deprecated.  Use mbf_msgs-msg:action_result instead.")
  (action_result m))

(cl:ensure-generic-function 'action_feedback-val :lambda-list '(m))
(cl:defmethod action_feedback-val ((m <ExePathAction>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader mbf_msgs-msg:action_feedback-val is deprecated.  Use mbf_msgs-msg:action_feedback instead.")
  (action_feedback m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <ExePathAction>) ostream)
  "Serializes a message object of type '<ExePathAction>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'action_goal) ostream)
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'action_result) ostream)
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'action_feedback) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <ExePathAction>) istream)
  "Deserializes a message object of type '<ExePathAction>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'action_goal) istream)
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'action_result) istream)
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'action_feedback) istream)
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<ExePathAction>)))
  "Returns string type for a message object of type '<ExePathAction>"
  "mbf_msgs/ExePathAction")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'ExePathAction)))
  "Returns string type for a message object of type 'ExePathAction"
  "mbf_msgs/ExePathAction")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<ExePathAction>)))
  "Returns md5sum for a message object of type '<ExePathAction>"
  "612b0e536656f46f77caecc9772e4bcd")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'ExePathAction)))
  "Returns md5sum for a message object of type 'ExePathAction"
  "612b0e536656f46f77caecc9772e4bcd")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<ExePathAction>)))
  "Returns full string definition for message of type '<ExePathAction>"
  (cl:format cl:nil "# ====== DO NOT MODIFY! AUTOGENERATED FROM AN ACTION DEFINITION ======~%~%ExePathActionGoal action_goal~%ExePathActionResult action_result~%ExePathActionFeedback action_feedback~%~%================================================================================~%MSG: mbf_msgs/ExePathActionGoal~%# ====== DO NOT MODIFY! AUTOGENERATED FROM AN ACTION DEFINITION ======~%~%Header header~%actionlib_msgs/GoalID goal_id~%ExePathGoal goal~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%string frame_id~%~%================================================================================~%MSG: actionlib_msgs/GoalID~%# The stamp should store the time at which this goal was requested.~%# It is used by an action server when it tries to preempt all~%# goals that were requested before a certain time~%time stamp~%~%# The id provides a way to associate feedback and~%# result message with specific goal requests. The id~%# specified must be unique.~%string id~%~%~%================================================================================~%MSG: mbf_msgs/ExePathGoal~%# ====== DO NOT MODIFY! AUTOGENERATED FROM AN ACTION DEFINITION ======~%# Follow the given path until completion or failure~%~%nav_msgs/Path path~%~%# Controller to use; defaults to the first one specified on \"controllers\" parameter~%string controller~%~%# use different slots for concurrency~%uint8 concurrency_slot~%~%# define goal tolerance for the action~%bool tolerance_from_action~%float32 dist_tolerance~%float32 angle_tolerance~%~%================================================================================~%MSG: nav_msgs/Path~%#An array of poses that represents a Path for a robot to follow~%Header header~%geometry_msgs/PoseStamped[] poses~%~%================================================================================~%MSG: geometry_msgs/PoseStamped~%# A Pose with reference coordinate frame and timestamp~%Header header~%Pose pose~%~%================================================================================~%MSG: geometry_msgs/Pose~%# A representation of pose in free space, composed of position and orientation. ~%Point position~%Quaternion orientation~%~%================================================================================~%MSG: geometry_msgs/Point~%# This contains the position of a point in free space~%float64 x~%float64 y~%float64 z~%~%================================================================================~%MSG: geometry_msgs/Quaternion~%# This represents an orientation in free space in quaternion form.~%~%float64 x~%float64 y~%float64 z~%float64 w~%~%================================================================================~%MSG: mbf_msgs/ExePathActionResult~%# ====== DO NOT MODIFY! AUTOGENERATED FROM AN ACTION DEFINITION ======~%~%Header header~%actionlib_msgs/GoalStatus status~%ExePathResult result~%~%================================================================================~%MSG: actionlib_msgs/GoalStatus~%GoalID goal_id~%uint8 status~%uint8 PENDING         = 0   # The goal has yet to be processed by the action server~%uint8 ACTIVE          = 1   # The goal is currently being processed by the action server~%uint8 PREEMPTED       = 2   # The goal received a cancel request after it started executing~%                            #   and has since completed its execution (Terminal State)~%uint8 SUCCEEDED       = 3   # The goal was achieved successfully by the action server (Terminal State)~%uint8 ABORTED         = 4   # The goal was aborted during execution by the action server due~%                            #    to some failure (Terminal State)~%uint8 REJECTED        = 5   # The goal was rejected by the action server without being processed,~%                            #    because the goal was unattainable or invalid (Terminal State)~%uint8 PREEMPTING      = 6   # The goal received a cancel request after it started executing~%                            #    and has not yet completed execution~%uint8 RECALLING       = 7   # The goal received a cancel request before it started executing,~%                            #    but the action server has not yet confirmed that the goal is canceled~%uint8 RECALLED        = 8   # The goal received a cancel request before it started executing~%                            #    and was successfully cancelled (Terminal State)~%uint8 LOST            = 9   # An action client can determine that a goal is LOST. This should not be~%                            #    sent over the wire by an action server~%~%#Allow for the user to associate a string with GoalStatus for debugging~%string text~%~%~%================================================================================~%MSG: mbf_msgs/ExePathResult~%# ====== DO NOT MODIFY! AUTOGENERATED FROM AN ACTION DEFINITION ======~%~%# Predefined success codes:~%uint8 SUCCESS           = 0~%# 1..9 are reserved as plugin specific non-error results~%~%# Predefined error codes:~%uint8 FAILURE           = 100  # Unspecified failure, only used for old, non-mfb_core based plugins~%uint8 CANCELED          = 101~%uint8 NO_VALID_CMD      = 102~%uint8 PAT_EXCEEDED      = 103~%uint8 COLLISION         = 104~%uint8 OSCILLATION       = 105~%uint8 ROBOT_STUCK       = 106  # The robot is not obeying the commanded velocity~%uint8 MISSED_GOAL       = 107  # The robot has overshot the goal and cannot reach it anymore~%uint8 MISSED_PATH       = 108  # The robot has diverged from the path and cannot rejoin it~%uint8 BLOCKED_GOAL      = 109  # There's an obstacle at the goal~%uint8 BLOCKED_PATH      = 110  # There's an obstacle on the path~%uint8 INVALID_PATH      = 111~%uint8 TF_ERROR          = 112~%uint8 NOT_INITIALIZED   = 113~%uint8 INVALID_PLUGIN    = 114~%uint8 INTERNAL_ERROR    = 115~%uint8 OUT_OF_MAP        = 116  # The start and / or the goal are outside the map~%uint8 MAP_ERROR         = 117  # The map is not available or not running properly~%uint8 STOPPED           = 118  # The controller execution has been stopped rigorously~%~%uint8 ERROR_RANGE_START = 100~%uint8 ERROR_RANGE_END   = 149~%~%# 121..149 are reserved as plugin specific errors:~%uint8 PLUGIN_ERROR_RANGE_START = 121~%uint8 PLUGIN_ERROR_RANGE_END   = 149~%~%uint32 outcome~%string message~%~%geometry_msgs/PoseStamped  final_pose~%float32 dist_to_goal~%float32 angle_to_goal~%~%~%================================================================================~%MSG: mbf_msgs/ExePathActionFeedback~%# ====== DO NOT MODIFY! AUTOGENERATED FROM AN ACTION DEFINITION ======~%~%Header header~%actionlib_msgs/GoalStatus status~%ExePathFeedback feedback~%~%================================================================================~%MSG: mbf_msgs/ExePathFeedback~%# ====== DO NOT MODIFY! AUTOGENERATED FROM AN ACTION DEFINITION ======~%~%# Outcome of most recent controller cycle. Same values as in result~%uint32 outcome~%string message~%~%float32 dist_to_goal~%float32 angle_to_goal~%geometry_msgs/PoseStamped  current_pose~%geometry_msgs/TwistStamped last_cmd_vel  # last command calculated by the controller~%~%~%================================================================================~%MSG: geometry_msgs/TwistStamped~%# A twist with reference coordinate frame and timestamp~%Header header~%Twist twist~%~%================================================================================~%MSG: geometry_msgs/Twist~%# This expresses velocity in free space broken into its linear and angular parts.~%Vector3  linear~%Vector3  angular~%~%================================================================================~%MSG: geometry_msgs/Vector3~%# This represents a vector in free space. ~%# It is only meant to represent a direction. Therefore, it does not~%# make sense to apply a translation to it (e.g., when applying a ~%# generic rigid transformation to a Vector3, tf2 will only apply the~%# rotation). If you want your data to be translatable too, use the~%# geometry_msgs/Point message instead.~%~%float64 x~%float64 y~%float64 z~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'ExePathAction)))
  "Returns full string definition for message of type 'ExePathAction"
  (cl:format cl:nil "# ====== DO NOT MODIFY! AUTOGENERATED FROM AN ACTION DEFINITION ======~%~%ExePathActionGoal action_goal~%ExePathActionResult action_result~%ExePathActionFeedback action_feedback~%~%================================================================================~%MSG: mbf_msgs/ExePathActionGoal~%# ====== DO NOT MODIFY! AUTOGENERATED FROM AN ACTION DEFINITION ======~%~%Header header~%actionlib_msgs/GoalID goal_id~%ExePathGoal goal~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%string frame_id~%~%================================================================================~%MSG: actionlib_msgs/GoalID~%# The stamp should store the time at which this goal was requested.~%# It is used by an action server when it tries to preempt all~%# goals that were requested before a certain time~%time stamp~%~%# The id provides a way to associate feedback and~%# result message with specific goal requests. The id~%# specified must be unique.~%string id~%~%~%================================================================================~%MSG: mbf_msgs/ExePathGoal~%# ====== DO NOT MODIFY! AUTOGENERATED FROM AN ACTION DEFINITION ======~%# Follow the given path until completion or failure~%~%nav_msgs/Path path~%~%# Controller to use; defaults to the first one specified on \"controllers\" parameter~%string controller~%~%# use different slots for concurrency~%uint8 concurrency_slot~%~%# define goal tolerance for the action~%bool tolerance_from_action~%float32 dist_tolerance~%float32 angle_tolerance~%~%================================================================================~%MSG: nav_msgs/Path~%#An array of poses that represents a Path for a robot to follow~%Header header~%geometry_msgs/PoseStamped[] poses~%~%================================================================================~%MSG: geometry_msgs/PoseStamped~%# A Pose with reference coordinate frame and timestamp~%Header header~%Pose pose~%~%================================================================================~%MSG: geometry_msgs/Pose~%# A representation of pose in free space, composed of position and orientation. ~%Point position~%Quaternion orientation~%~%================================================================================~%MSG: geometry_msgs/Point~%# This contains the position of a point in free space~%float64 x~%float64 y~%float64 z~%~%================================================================================~%MSG: geometry_msgs/Quaternion~%# This represents an orientation in free space in quaternion form.~%~%float64 x~%float64 y~%float64 z~%float64 w~%~%================================================================================~%MSG: mbf_msgs/ExePathActionResult~%# ====== DO NOT MODIFY! AUTOGENERATED FROM AN ACTION DEFINITION ======~%~%Header header~%actionlib_msgs/GoalStatus status~%ExePathResult result~%~%================================================================================~%MSG: actionlib_msgs/GoalStatus~%GoalID goal_id~%uint8 status~%uint8 PENDING         = 0   # The goal has yet to be processed by the action server~%uint8 ACTIVE          = 1   # The goal is currently being processed by the action server~%uint8 PREEMPTED       = 2   # The goal received a cancel request after it started executing~%                            #   and has since completed its execution (Terminal State)~%uint8 SUCCEEDED       = 3   # The goal was achieved successfully by the action server (Terminal State)~%uint8 ABORTED         = 4   # The goal was aborted during execution by the action server due~%                            #    to some failure (Terminal State)~%uint8 REJECTED        = 5   # The goal was rejected by the action server without being processed,~%                            #    because the goal was unattainable or invalid (Terminal State)~%uint8 PREEMPTING      = 6   # The goal received a cancel request after it started executing~%                            #    and has not yet completed execution~%uint8 RECALLING       = 7   # The goal received a cancel request before it started executing,~%                            #    but the action server has not yet confirmed that the goal is canceled~%uint8 RECALLED        = 8   # The goal received a cancel request before it started executing~%                            #    and was successfully cancelled (Terminal State)~%uint8 LOST            = 9   # An action client can determine that a goal is LOST. This should not be~%                            #    sent over the wire by an action server~%~%#Allow for the user to associate a string with GoalStatus for debugging~%string text~%~%~%================================================================================~%MSG: mbf_msgs/ExePathResult~%# ====== DO NOT MODIFY! AUTOGENERATED FROM AN ACTION DEFINITION ======~%~%# Predefined success codes:~%uint8 SUCCESS           = 0~%# 1..9 are reserved as plugin specific non-error results~%~%# Predefined error codes:~%uint8 FAILURE           = 100  # Unspecified failure, only used for old, non-mfb_core based plugins~%uint8 CANCELED          = 101~%uint8 NO_VALID_CMD      = 102~%uint8 PAT_EXCEEDED      = 103~%uint8 COLLISION         = 104~%uint8 OSCILLATION       = 105~%uint8 ROBOT_STUCK       = 106  # The robot is not obeying the commanded velocity~%uint8 MISSED_GOAL       = 107  # The robot has overshot the goal and cannot reach it anymore~%uint8 MISSED_PATH       = 108  # The robot has diverged from the path and cannot rejoin it~%uint8 BLOCKED_GOAL      = 109  # There's an obstacle at the goal~%uint8 BLOCKED_PATH      = 110  # There's an obstacle on the path~%uint8 INVALID_PATH      = 111~%uint8 TF_ERROR          = 112~%uint8 NOT_INITIALIZED   = 113~%uint8 INVALID_PLUGIN    = 114~%uint8 INTERNAL_ERROR    = 115~%uint8 OUT_OF_MAP        = 116  # The start and / or the goal are outside the map~%uint8 MAP_ERROR         = 117  # The map is not available or not running properly~%uint8 STOPPED           = 118  # The controller execution has been stopped rigorously~%~%uint8 ERROR_RANGE_START = 100~%uint8 ERROR_RANGE_END   = 149~%~%# 121..149 are reserved as plugin specific errors:~%uint8 PLUGIN_ERROR_RANGE_START = 121~%uint8 PLUGIN_ERROR_RANGE_END   = 149~%~%uint32 outcome~%string message~%~%geometry_msgs/PoseStamped  final_pose~%float32 dist_to_goal~%float32 angle_to_goal~%~%~%================================================================================~%MSG: mbf_msgs/ExePathActionFeedback~%# ====== DO NOT MODIFY! AUTOGENERATED FROM AN ACTION DEFINITION ======~%~%Header header~%actionlib_msgs/GoalStatus status~%ExePathFeedback feedback~%~%================================================================================~%MSG: mbf_msgs/ExePathFeedback~%# ====== DO NOT MODIFY! AUTOGENERATED FROM AN ACTION DEFINITION ======~%~%# Outcome of most recent controller cycle. Same values as in result~%uint32 outcome~%string message~%~%float32 dist_to_goal~%float32 angle_to_goal~%geometry_msgs/PoseStamped  current_pose~%geometry_msgs/TwistStamped last_cmd_vel  # last command calculated by the controller~%~%~%================================================================================~%MSG: geometry_msgs/TwistStamped~%# A twist with reference coordinate frame and timestamp~%Header header~%Twist twist~%~%================================================================================~%MSG: geometry_msgs/Twist~%# This expresses velocity in free space broken into its linear and angular parts.~%Vector3  linear~%Vector3  angular~%~%================================================================================~%MSG: geometry_msgs/Vector3~%# This represents a vector in free space. ~%# It is only meant to represent a direction. Therefore, it does not~%# make sense to apply a translation to it (e.g., when applying a ~%# generic rigid transformation to a Vector3, tf2 will only apply the~%# rotation). If you want your data to be translatable too, use the~%# geometry_msgs/Point message instead.~%~%float64 x~%float64 y~%float64 z~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <ExePathAction>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'action_goal))
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'action_result))
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'action_feedback))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <ExePathAction>))
  "Converts a ROS message object to a list"
  (cl:list 'ExePathAction
    (cl:cons ':action_goal (action_goal msg))
    (cl:cons ':action_result (action_result msg))
    (cl:cons ':action_feedback (action_feedback msg))
))
