; Auto-generated. Do not edit!


(cl:in-package mbf_msgs-msg)


;//! \htmlinclude GetInterPathAction.msg.html

(cl:defclass <GetInterPathAction> (roslisp-msg-protocol:ros-message)
  ((action_goal
    :reader action_goal
    :initarg :action_goal
    :type mbf_msgs-msg:GetInterPathActionGoal
    :initform (cl:make-instance 'mbf_msgs-msg:GetInterPathActionGoal))
   (action_result
    :reader action_result
    :initarg :action_result
    :type mbf_msgs-msg:GetInterPathActionResult
    :initform (cl:make-instance 'mbf_msgs-msg:GetInterPathActionResult))
   (action_feedback
    :reader action_feedback
    :initarg :action_feedback
    :type mbf_msgs-msg:GetInterPathActionFeedback
    :initform (cl:make-instance 'mbf_msgs-msg:GetInterPathActionFeedback)))
)

(cl:defclass GetInterPathAction (<GetInterPathAction>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <GetInterPathAction>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'GetInterPathAction)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name mbf_msgs-msg:<GetInterPathAction> is deprecated: use mbf_msgs-msg:GetInterPathAction instead.")))

(cl:ensure-generic-function 'action_goal-val :lambda-list '(m))
(cl:defmethod action_goal-val ((m <GetInterPathAction>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader mbf_msgs-msg:action_goal-val is deprecated.  Use mbf_msgs-msg:action_goal instead.")
  (action_goal m))

(cl:ensure-generic-function 'action_result-val :lambda-list '(m))
(cl:defmethod action_result-val ((m <GetInterPathAction>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader mbf_msgs-msg:action_result-val is deprecated.  Use mbf_msgs-msg:action_result instead.")
  (action_result m))

(cl:ensure-generic-function 'action_feedback-val :lambda-list '(m))
(cl:defmethod action_feedback-val ((m <GetInterPathAction>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader mbf_msgs-msg:action_feedback-val is deprecated.  Use mbf_msgs-msg:action_feedback instead.")
  (action_feedback m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <GetInterPathAction>) ostream)
  "Serializes a message object of type '<GetInterPathAction>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'action_goal) ostream)
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'action_result) ostream)
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'action_feedback) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <GetInterPathAction>) istream)
  "Deserializes a message object of type '<GetInterPathAction>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'action_goal) istream)
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'action_result) istream)
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'action_feedback) istream)
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<GetInterPathAction>)))
  "Returns string type for a message object of type '<GetInterPathAction>"
  "mbf_msgs/GetInterPathAction")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'GetInterPathAction)))
  "Returns string type for a message object of type 'GetInterPathAction"
  "mbf_msgs/GetInterPathAction")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<GetInterPathAction>)))
  "Returns md5sum for a message object of type '<GetInterPathAction>"
  "7e257549ee3f0e439a67e31b9101c498")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'GetInterPathAction)))
  "Returns md5sum for a message object of type 'GetInterPathAction"
  "7e257549ee3f0e439a67e31b9101c498")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<GetInterPathAction>)))
  "Returns full string definition for message of type '<GetInterPathAction>"
  (cl:format cl:nil "# ====== DO NOT MODIFY! AUTOGENERATED FROM AN ACTION DEFINITION ======~%~%GetInterPathActionGoal action_goal~%GetInterPathActionResult action_result~%GetInterPathActionFeedback action_feedback~%~%================================================================================~%MSG: mbf_msgs/GetInterPathActionGoal~%# ====== DO NOT MODIFY! AUTOGENERATED FROM AN ACTION DEFINITION ======~%~%Header header~%actionlib_msgs/GoalID goal_id~%GetInterPathGoal goal~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%string frame_id~%~%================================================================================~%MSG: actionlib_msgs/GoalID~%# The stamp should store the time at which this goal was requested.~%# It is used by an action server when it tries to preempt all~%# goals that were requested before a certain time~%time stamp~%~%# The id provides a way to associate feedback and~%# result message with specific goal requests. The id~%# specified must be unique.~%string id~%~%~%================================================================================~%MSG: mbf_msgs/GetInterPathGoal~%# ====== DO NOT MODIFY! AUTOGENERATED FROM AN ACTION DEFINITION ======~%# Interpolate path from start_pose or current position to the target pose~%~%# Follow the given path until completion or failure~%nav_msgs/Path path~%~%# Use start_pose or current position as the beginning of the path~%bool use_start_pose~%~%# The start pose for the path; optional, used if use_start_pose is true~%geometry_msgs/PoseStamped start_pose~%~%# The pose to achieve with the path~%geometry_msgs/PoseStamped target_pose~%~%# Inter to use; defaults to the first one specified on \"inter\" parameter~%string inter~%~%# use different slots for concurrency~%uint8 concurrency_slot~%~%~%================================================================================~%MSG: nav_msgs/Path~%#An array of poses that represents a Path for a robot to follow~%Header header~%geometry_msgs/PoseStamped[] poses~%~%================================================================================~%MSG: geometry_msgs/PoseStamped~%# A Pose with reference coordinate frame and timestamp~%Header header~%Pose pose~%~%================================================================================~%MSG: geometry_msgs/Pose~%# A representation of pose in free space, composed of position and orientation. ~%Point position~%Quaternion orientation~%~%================================================================================~%MSG: geometry_msgs/Point~%# This contains the position of a point in free space~%float64 x~%float64 y~%float64 z~%~%================================================================================~%MSG: geometry_msgs/Quaternion~%# This represents an orientation in free space in quaternion form.~%~%float64 x~%float64 y~%float64 z~%float64 w~%~%================================================================================~%MSG: mbf_msgs/GetInterPathActionResult~%# ====== DO NOT MODIFY! AUTOGENERATED FROM AN ACTION DEFINITION ======~%~%Header header~%actionlib_msgs/GoalStatus status~%GetInterPathResult result~%~%================================================================================~%MSG: actionlib_msgs/GoalStatus~%GoalID goal_id~%uint8 status~%uint8 PENDING         = 0   # The goal has yet to be processed by the action server~%uint8 ACTIVE          = 1   # The goal is currently being processed by the action server~%uint8 PREEMPTED       = 2   # The goal received a cancel request after it started executing~%                            #   and has since completed its execution (Terminal State)~%uint8 SUCCEEDED       = 3   # The goal was achieved successfully by the action server (Terminal State)~%uint8 ABORTED         = 4   # The goal was aborted during execution by the action server due~%                            #    to some failure (Terminal State)~%uint8 REJECTED        = 5   # The goal was rejected by the action server without being processed,~%                            #    because the goal was unattainable or invalid (Terminal State)~%uint8 PREEMPTING      = 6   # The goal received a cancel request after it started executing~%                            #    and has not yet completed execution~%uint8 RECALLING       = 7   # The goal received a cancel request before it started executing,~%                            #    but the action server has not yet confirmed that the goal is canceled~%uint8 RECALLED        = 8   # The goal received a cancel request before it started executing~%                            #    and was successfully cancelled (Terminal State)~%uint8 LOST            = 9   # An action client can determine that a goal is LOST. This should not be~%                            #    sent over the wire by an action server~%~%#Allow for the user to associate a string with GoalStatus for debugging~%string text~%~%~%================================================================================~%MSG: mbf_msgs/GetInterPathResult~%# ====== DO NOT MODIFY! AUTOGENERATED FROM AN ACTION DEFINITION ======~%~%# Predefined success codes:~%uint8 SUCCESS           = 0~%# 1..9 are reserved as plugin specific non-error results~%~%# Possible error codes:~%uint8 FAILURE           = 50  # Unspecified failure, only used for old, non-mfb_core based plugins~%uint8 CANCELED          = 51  # The action has been canceled by a action client~%uint8 INVALID_START     = 52  # The start pose is inconsistent (e.g. frame is not valid)~%uint8 INVALID_GOAL      = 53  # The goal pose is inconsistent (e.g. frame is not valid)~%uint8 BLOCKED_START     = 54  # The start pose is in collision~%uint8 BLOCKED_GOAL      = 55  # The goal pose is in collision~%uint8 NO_PATH_FOUND     = 56~%uint8 PAT_EXCEEDED      = 57~%uint8 EMPTY_PATH        = 58~%uint8 TF_ERROR          = 59~%uint8 NOT_INITIALIZED   = 60~%uint8 INVALID_PLUGIN    = 61~%uint8 INTERNAL_ERROR    = 62~%uint8 OUT_OF_MAP        = 63  # The start and / or the goal are outside the map~%uint8 MAP_ERROR         = 64  # The map is not available or not running properly~%uint8 STOPPED           = 65  # The planner execution has been stopped rigorously~%~%uint8 ERROR_RANGE_START = 50~%uint8 ERROR_RANGE_END   = 99~%~%# 71..99 are reserved as plugin specific errors:~%uint8 PLUGIN_ERROR_RANGE_START = 71~%uint8 PLUGIN_ERROR_RANGE_END   = 99~%~%uint32 outcome~%string message~%~%nav_msgs/Path path~%~%float64 cost~%~%~%================================================================================~%MSG: mbf_msgs/GetInterPathActionFeedback~%# ====== DO NOT MODIFY! AUTOGENERATED FROM AN ACTION DEFINITION ======~%~%Header header~%actionlib_msgs/GoalStatus status~%GetInterPathFeedback feedback~%~%================================================================================~%MSG: mbf_msgs/GetInterPathFeedback~%# ====== DO NOT MODIFY! AUTOGENERATED FROM AN ACTION DEFINITION ======~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'GetInterPathAction)))
  "Returns full string definition for message of type 'GetInterPathAction"
  (cl:format cl:nil "# ====== DO NOT MODIFY! AUTOGENERATED FROM AN ACTION DEFINITION ======~%~%GetInterPathActionGoal action_goal~%GetInterPathActionResult action_result~%GetInterPathActionFeedback action_feedback~%~%================================================================================~%MSG: mbf_msgs/GetInterPathActionGoal~%# ====== DO NOT MODIFY! AUTOGENERATED FROM AN ACTION DEFINITION ======~%~%Header header~%actionlib_msgs/GoalID goal_id~%GetInterPathGoal goal~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%string frame_id~%~%================================================================================~%MSG: actionlib_msgs/GoalID~%# The stamp should store the time at which this goal was requested.~%# It is used by an action server when it tries to preempt all~%# goals that were requested before a certain time~%time stamp~%~%# The id provides a way to associate feedback and~%# result message with specific goal requests. The id~%# specified must be unique.~%string id~%~%~%================================================================================~%MSG: mbf_msgs/GetInterPathGoal~%# ====== DO NOT MODIFY! AUTOGENERATED FROM AN ACTION DEFINITION ======~%# Interpolate path from start_pose or current position to the target pose~%~%# Follow the given path until completion or failure~%nav_msgs/Path path~%~%# Use start_pose or current position as the beginning of the path~%bool use_start_pose~%~%# The start pose for the path; optional, used if use_start_pose is true~%geometry_msgs/PoseStamped start_pose~%~%# The pose to achieve with the path~%geometry_msgs/PoseStamped target_pose~%~%# Inter to use; defaults to the first one specified on \"inter\" parameter~%string inter~%~%# use different slots for concurrency~%uint8 concurrency_slot~%~%~%================================================================================~%MSG: nav_msgs/Path~%#An array of poses that represents a Path for a robot to follow~%Header header~%geometry_msgs/PoseStamped[] poses~%~%================================================================================~%MSG: geometry_msgs/PoseStamped~%# A Pose with reference coordinate frame and timestamp~%Header header~%Pose pose~%~%================================================================================~%MSG: geometry_msgs/Pose~%# A representation of pose in free space, composed of position and orientation. ~%Point position~%Quaternion orientation~%~%================================================================================~%MSG: geometry_msgs/Point~%# This contains the position of a point in free space~%float64 x~%float64 y~%float64 z~%~%================================================================================~%MSG: geometry_msgs/Quaternion~%# This represents an orientation in free space in quaternion form.~%~%float64 x~%float64 y~%float64 z~%float64 w~%~%================================================================================~%MSG: mbf_msgs/GetInterPathActionResult~%# ====== DO NOT MODIFY! AUTOGENERATED FROM AN ACTION DEFINITION ======~%~%Header header~%actionlib_msgs/GoalStatus status~%GetInterPathResult result~%~%================================================================================~%MSG: actionlib_msgs/GoalStatus~%GoalID goal_id~%uint8 status~%uint8 PENDING         = 0   # The goal has yet to be processed by the action server~%uint8 ACTIVE          = 1   # The goal is currently being processed by the action server~%uint8 PREEMPTED       = 2   # The goal received a cancel request after it started executing~%                            #   and has since completed its execution (Terminal State)~%uint8 SUCCEEDED       = 3   # The goal was achieved successfully by the action server (Terminal State)~%uint8 ABORTED         = 4   # The goal was aborted during execution by the action server due~%                            #    to some failure (Terminal State)~%uint8 REJECTED        = 5   # The goal was rejected by the action server without being processed,~%                            #    because the goal was unattainable or invalid (Terminal State)~%uint8 PREEMPTING      = 6   # The goal received a cancel request after it started executing~%                            #    and has not yet completed execution~%uint8 RECALLING       = 7   # The goal received a cancel request before it started executing,~%                            #    but the action server has not yet confirmed that the goal is canceled~%uint8 RECALLED        = 8   # The goal received a cancel request before it started executing~%                            #    and was successfully cancelled (Terminal State)~%uint8 LOST            = 9   # An action client can determine that a goal is LOST. This should not be~%                            #    sent over the wire by an action server~%~%#Allow for the user to associate a string with GoalStatus for debugging~%string text~%~%~%================================================================================~%MSG: mbf_msgs/GetInterPathResult~%# ====== DO NOT MODIFY! AUTOGENERATED FROM AN ACTION DEFINITION ======~%~%# Predefined success codes:~%uint8 SUCCESS           = 0~%# 1..9 are reserved as plugin specific non-error results~%~%# Possible error codes:~%uint8 FAILURE           = 50  # Unspecified failure, only used for old, non-mfb_core based plugins~%uint8 CANCELED          = 51  # The action has been canceled by a action client~%uint8 INVALID_START     = 52  # The start pose is inconsistent (e.g. frame is not valid)~%uint8 INVALID_GOAL      = 53  # The goal pose is inconsistent (e.g. frame is not valid)~%uint8 BLOCKED_START     = 54  # The start pose is in collision~%uint8 BLOCKED_GOAL      = 55  # The goal pose is in collision~%uint8 NO_PATH_FOUND     = 56~%uint8 PAT_EXCEEDED      = 57~%uint8 EMPTY_PATH        = 58~%uint8 TF_ERROR          = 59~%uint8 NOT_INITIALIZED   = 60~%uint8 INVALID_PLUGIN    = 61~%uint8 INTERNAL_ERROR    = 62~%uint8 OUT_OF_MAP        = 63  # The start and / or the goal are outside the map~%uint8 MAP_ERROR         = 64  # The map is not available or not running properly~%uint8 STOPPED           = 65  # The planner execution has been stopped rigorously~%~%uint8 ERROR_RANGE_START = 50~%uint8 ERROR_RANGE_END   = 99~%~%# 71..99 are reserved as plugin specific errors:~%uint8 PLUGIN_ERROR_RANGE_START = 71~%uint8 PLUGIN_ERROR_RANGE_END   = 99~%~%uint32 outcome~%string message~%~%nav_msgs/Path path~%~%float64 cost~%~%~%================================================================================~%MSG: mbf_msgs/GetInterPathActionFeedback~%# ====== DO NOT MODIFY! AUTOGENERATED FROM AN ACTION DEFINITION ======~%~%Header header~%actionlib_msgs/GoalStatus status~%GetInterPathFeedback feedback~%~%================================================================================~%MSG: mbf_msgs/GetInterPathFeedback~%# ====== DO NOT MODIFY! AUTOGENERATED FROM AN ACTION DEFINITION ======~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <GetInterPathAction>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'action_goal))
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'action_result))
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'action_feedback))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <GetInterPathAction>))
  "Converts a ROS message object to a list"
  (cl:list 'GetInterPathAction
    (cl:cons ':action_goal (action_goal msg))
    (cl:cons ':action_result (action_result msg))
    (cl:cons ':action_feedback (action_feedback msg))
))
