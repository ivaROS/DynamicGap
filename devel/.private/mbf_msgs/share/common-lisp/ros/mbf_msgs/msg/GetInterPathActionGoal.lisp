; Auto-generated. Do not edit!


(cl:in-package mbf_msgs-msg)


;//! \htmlinclude GetInterPathActionGoal.msg.html

(cl:defclass <GetInterPathActionGoal> (roslisp-msg-protocol:ros-message)
  ((header
    :reader header
    :initarg :header
    :type std_msgs-msg:Header
    :initform (cl:make-instance 'std_msgs-msg:Header))
   (goal_id
    :reader goal_id
    :initarg :goal_id
    :type actionlib_msgs-msg:GoalID
    :initform (cl:make-instance 'actionlib_msgs-msg:GoalID))
   (goal
    :reader goal
    :initarg :goal
    :type mbf_msgs-msg:GetInterPathGoal
    :initform (cl:make-instance 'mbf_msgs-msg:GetInterPathGoal)))
)

(cl:defclass GetInterPathActionGoal (<GetInterPathActionGoal>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <GetInterPathActionGoal>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'GetInterPathActionGoal)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name mbf_msgs-msg:<GetInterPathActionGoal> is deprecated: use mbf_msgs-msg:GetInterPathActionGoal instead.")))

(cl:ensure-generic-function 'header-val :lambda-list '(m))
(cl:defmethod header-val ((m <GetInterPathActionGoal>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader mbf_msgs-msg:header-val is deprecated.  Use mbf_msgs-msg:header instead.")
  (header m))

(cl:ensure-generic-function 'goal_id-val :lambda-list '(m))
(cl:defmethod goal_id-val ((m <GetInterPathActionGoal>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader mbf_msgs-msg:goal_id-val is deprecated.  Use mbf_msgs-msg:goal_id instead.")
  (goal_id m))

(cl:ensure-generic-function 'goal-val :lambda-list '(m))
(cl:defmethod goal-val ((m <GetInterPathActionGoal>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader mbf_msgs-msg:goal-val is deprecated.  Use mbf_msgs-msg:goal instead.")
  (goal m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <GetInterPathActionGoal>) ostream)
  "Serializes a message object of type '<GetInterPathActionGoal>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'header) ostream)
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'goal_id) ostream)
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'goal) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <GetInterPathActionGoal>) istream)
  "Deserializes a message object of type '<GetInterPathActionGoal>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'header) istream)
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'goal_id) istream)
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'goal) istream)
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<GetInterPathActionGoal>)))
  "Returns string type for a message object of type '<GetInterPathActionGoal>"
  "mbf_msgs/GetInterPathActionGoal")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'GetInterPathActionGoal)))
  "Returns string type for a message object of type 'GetInterPathActionGoal"
  "mbf_msgs/GetInterPathActionGoal")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<GetInterPathActionGoal>)))
  "Returns md5sum for a message object of type '<GetInterPathActionGoal>"
  "c204e8918f8c3bb3cb41979b11fb2e20")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'GetInterPathActionGoal)))
  "Returns md5sum for a message object of type 'GetInterPathActionGoal"
  "c204e8918f8c3bb3cb41979b11fb2e20")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<GetInterPathActionGoal>)))
  "Returns full string definition for message of type '<GetInterPathActionGoal>"
  (cl:format cl:nil "# ====== DO NOT MODIFY! AUTOGENERATED FROM AN ACTION DEFINITION ======~%~%Header header~%actionlib_msgs/GoalID goal_id~%GetInterPathGoal goal~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%string frame_id~%~%================================================================================~%MSG: actionlib_msgs/GoalID~%# The stamp should store the time at which this goal was requested.~%# It is used by an action server when it tries to preempt all~%# goals that were requested before a certain time~%time stamp~%~%# The id provides a way to associate feedback and~%# result message with specific goal requests. The id~%# specified must be unique.~%string id~%~%~%================================================================================~%MSG: mbf_msgs/GetInterPathGoal~%# ====== DO NOT MODIFY! AUTOGENERATED FROM AN ACTION DEFINITION ======~%# Interpolate path from start_pose or current position to the target pose~%~%# Follow the given path until completion or failure~%nav_msgs/Path path~%~%# Use start_pose or current position as the beginning of the path~%bool use_start_pose~%~%# The start pose for the path; optional, used if use_start_pose is true~%geometry_msgs/PoseStamped start_pose~%~%# The pose to achieve with the path~%geometry_msgs/PoseStamped target_pose~%~%# Inter to use; defaults to the first one specified on \"inter\" parameter~%string inter~%~%# use different slots for concurrency~%uint8 concurrency_slot~%~%~%================================================================================~%MSG: nav_msgs/Path~%#An array of poses that represents a Path for a robot to follow~%Header header~%geometry_msgs/PoseStamped[] poses~%~%================================================================================~%MSG: geometry_msgs/PoseStamped~%# A Pose with reference coordinate frame and timestamp~%Header header~%Pose pose~%~%================================================================================~%MSG: geometry_msgs/Pose~%# A representation of pose in free space, composed of position and orientation. ~%Point position~%Quaternion orientation~%~%================================================================================~%MSG: geometry_msgs/Point~%# This contains the position of a point in free space~%float64 x~%float64 y~%float64 z~%~%================================================================================~%MSG: geometry_msgs/Quaternion~%# This represents an orientation in free space in quaternion form.~%~%float64 x~%float64 y~%float64 z~%float64 w~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'GetInterPathActionGoal)))
  "Returns full string definition for message of type 'GetInterPathActionGoal"
  (cl:format cl:nil "# ====== DO NOT MODIFY! AUTOGENERATED FROM AN ACTION DEFINITION ======~%~%Header header~%actionlib_msgs/GoalID goal_id~%GetInterPathGoal goal~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%string frame_id~%~%================================================================================~%MSG: actionlib_msgs/GoalID~%# The stamp should store the time at which this goal was requested.~%# It is used by an action server when it tries to preempt all~%# goals that were requested before a certain time~%time stamp~%~%# The id provides a way to associate feedback and~%# result message with specific goal requests. The id~%# specified must be unique.~%string id~%~%~%================================================================================~%MSG: mbf_msgs/GetInterPathGoal~%# ====== DO NOT MODIFY! AUTOGENERATED FROM AN ACTION DEFINITION ======~%# Interpolate path from start_pose or current position to the target pose~%~%# Follow the given path until completion or failure~%nav_msgs/Path path~%~%# Use start_pose or current position as the beginning of the path~%bool use_start_pose~%~%# The start pose for the path; optional, used if use_start_pose is true~%geometry_msgs/PoseStamped start_pose~%~%# The pose to achieve with the path~%geometry_msgs/PoseStamped target_pose~%~%# Inter to use; defaults to the first one specified on \"inter\" parameter~%string inter~%~%# use different slots for concurrency~%uint8 concurrency_slot~%~%~%================================================================================~%MSG: nav_msgs/Path~%#An array of poses that represents a Path for a robot to follow~%Header header~%geometry_msgs/PoseStamped[] poses~%~%================================================================================~%MSG: geometry_msgs/PoseStamped~%# A Pose with reference coordinate frame and timestamp~%Header header~%Pose pose~%~%================================================================================~%MSG: geometry_msgs/Pose~%# A representation of pose in free space, composed of position and orientation. ~%Point position~%Quaternion orientation~%~%================================================================================~%MSG: geometry_msgs/Point~%# This contains the position of a point in free space~%float64 x~%float64 y~%float64 z~%~%================================================================================~%MSG: geometry_msgs/Quaternion~%# This represents an orientation in free space in quaternion form.~%~%float64 x~%float64 y~%float64 z~%float64 w~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <GetInterPathActionGoal>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'header))
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'goal_id))
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'goal))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <GetInterPathActionGoal>))
  "Converts a ROS message object to a list"
  (cl:list 'GetInterPathActionGoal
    (cl:cons ':header (header msg))
    (cl:cons ':goal_id (goal_id msg))
    (cl:cons ':goal (goal msg))
))
