; Auto-generated. Do not edit!


(cl:in-package pedsim_srvs-srv)


;//! \htmlinclude GetAllAgentsState-request.msg.html

(cl:defclass <GetAllAgentsState-request> (roslisp-msg-protocol:ros-message)
  ((agent_ids
    :reader agent_ids
    :initarg :agent_ids
    :type (cl:vector cl:string)
   :initform (cl:make-array 0 :element-type 'cl:string :initial-element "")))
)

(cl:defclass GetAllAgentsState-request (<GetAllAgentsState-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <GetAllAgentsState-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'GetAllAgentsState-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name pedsim_srvs-srv:<GetAllAgentsState-request> is deprecated: use pedsim_srvs-srv:GetAllAgentsState-request instead.")))

(cl:ensure-generic-function 'agent_ids-val :lambda-list '(m))
(cl:defmethod agent_ids-val ((m <GetAllAgentsState-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader pedsim_srvs-srv:agent_ids-val is deprecated.  Use pedsim_srvs-srv:agent_ids instead.")
  (agent_ids m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <GetAllAgentsState-request>) ostream)
  "Serializes a message object of type '<GetAllAgentsState-request>"
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'agent_ids))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (cl:let ((__ros_str_len (cl:length ele)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) ele))
   (cl:slot-value msg 'agent_ids))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <GetAllAgentsState-request>) istream)
  "Deserializes a message object of type '<GetAllAgentsState-request>"
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'agent_ids) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'agent_ids)))
    (cl:dotimes (i __ros_arr_len)
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:aref vals i) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:aref vals i) __ros_str_idx) (cl:code-char (cl:read-byte istream))))))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<GetAllAgentsState-request>)))
  "Returns string type for a service object of type '<GetAllAgentsState-request>"
  "pedsim_srvs/GetAllAgentsStateRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'GetAllAgentsState-request)))
  "Returns string type for a service object of type 'GetAllAgentsState-request"
  "pedsim_srvs/GetAllAgentsStateRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<GetAllAgentsState-request>)))
  "Returns md5sum for a message object of type '<GetAllAgentsState-request>"
  "cc78f1dc691ac5d2580b91cf13114720")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'GetAllAgentsState-request)))
  "Returns md5sum for a message object of type 'GetAllAgentsState-request"
  "cc78f1dc691ac5d2580b91cf13114720")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<GetAllAgentsState-request>)))
  "Returns full string definition for message of type '<GetAllAgentsState-request>"
  (cl:format cl:nil "string[] agent_ids~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'GetAllAgentsState-request)))
  "Returns full string definition for message of type 'GetAllAgentsState-request"
  (cl:format cl:nil "string[] agent_ids~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <GetAllAgentsState-request>))
  (cl:+ 0
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'agent_ids) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ 4 (cl:length ele))))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <GetAllAgentsState-request>))
  "Converts a ROS message object to a list"
  (cl:list 'GetAllAgentsState-request
    (cl:cons ':agent_ids (agent_ids msg))
))
;//! \htmlinclude GetAllAgentsState-response.msg.html

(cl:defclass <GetAllAgentsState-response> (roslisp-msg-protocol:ros-message)
  ((agent_states
    :reader agent_states
    :initarg :agent_states
    :type pedsim_msgs-msg:AgentStates
    :initform (cl:make-instance 'pedsim_msgs-msg:AgentStates)))
)

(cl:defclass GetAllAgentsState-response (<GetAllAgentsState-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <GetAllAgentsState-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'GetAllAgentsState-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name pedsim_srvs-srv:<GetAllAgentsState-response> is deprecated: use pedsim_srvs-srv:GetAllAgentsState-response instead.")))

(cl:ensure-generic-function 'agent_states-val :lambda-list '(m))
(cl:defmethod agent_states-val ((m <GetAllAgentsState-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader pedsim_srvs-srv:agent_states-val is deprecated.  Use pedsim_srvs-srv:agent_states instead.")
  (agent_states m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <GetAllAgentsState-response>) ostream)
  "Serializes a message object of type '<GetAllAgentsState-response>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'agent_states) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <GetAllAgentsState-response>) istream)
  "Deserializes a message object of type '<GetAllAgentsState-response>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'agent_states) istream)
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<GetAllAgentsState-response>)))
  "Returns string type for a service object of type '<GetAllAgentsState-response>"
  "pedsim_srvs/GetAllAgentsStateResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'GetAllAgentsState-response)))
  "Returns string type for a service object of type 'GetAllAgentsState-response"
  "pedsim_srvs/GetAllAgentsStateResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<GetAllAgentsState-response>)))
  "Returns md5sum for a message object of type '<GetAllAgentsState-response>"
  "cc78f1dc691ac5d2580b91cf13114720")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'GetAllAgentsState-response)))
  "Returns md5sum for a message object of type 'GetAllAgentsState-response"
  "cc78f1dc691ac5d2580b91cf13114720")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<GetAllAgentsState-response>)))
  "Returns full string definition for message of type '<GetAllAgentsState-response>"
  (cl:format cl:nil "pedsim_msgs/AgentStates agent_states~%~%~%================================================================================~%MSG: pedsim_msgs/AgentStates~%Header header~%pedsim_msgs/AgentState[] agent_states~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%string frame_id~%~%================================================================================~%MSG: pedsim_msgs/AgentState~%Header header~%string id~%string type~%string social_state~%geometry_msgs/Pose pose~%geometry_msgs/Twist twist~%pedsim_msgs/AgentForce forces~%string talking_to_id~%string listening_to_id~%geometry_msgs/Vector3 acceleration~%geometry_msgs/Vector3 destination~%float64 direction~%string configuration~%~%uint8 IDLE        = 0~%uint8 WALKING     = 1~%uint8 RUNNING     = 2~%uint8 INTERACTING = 3~%uint8 TALKING     = 4~%uint8 PHONE       = 5~%================================================================================~%MSG: geometry_msgs/Pose~%# A representation of pose in free space, composed of position and orientation. ~%Point position~%Quaternion orientation~%~%================================================================================~%MSG: geometry_msgs/Point~%# This contains the position of a point in free space~%float64 x~%float64 y~%float64 z~%~%================================================================================~%MSG: geometry_msgs/Quaternion~%# This represents an orientation in free space in quaternion form.~%~%float64 x~%float64 y~%float64 z~%float64 w~%~%================================================================================~%MSG: geometry_msgs/Twist~%# This expresses velocity in free space broken into its linear and angular parts.~%Vector3  linear~%Vector3  angular~%~%================================================================================~%MSG: geometry_msgs/Vector3~%# This represents a vector in free space. ~%# It is only meant to represent a direction. Therefore, it does not~%# make sense to apply a translation to it (e.g., when applying a ~%# generic rigid transformation to a Vector3, tf2 will only apply the~%# rotation). If you want your data to be translatable too, use the~%# geometry_msgs/Point message instead.~%~%float64 x~%float64 y~%float64 z~%================================================================================~%MSG: pedsim_msgs/AgentForce~%# Forces acting on an agent.~%~%# Max Speed~%float64 vmax~%~%# Force Factors~%float64 desired_ffactor~%float64 obstacle_ffactor~%float64 social_ffactor~%float64 robot_ffactor~%~%# Basic SFM forces.~%geometry_msgs/Vector3 desired_force~%geometry_msgs/Vector3 obstacle_force~%geometry_msgs/Vector3 social_force~%~%# Additional Group Forces~%geometry_msgs/Vector3 group_coherence_force~%geometry_msgs/Vector3 group_gaze_force~%geometry_msgs/Vector3 group_repulsion_force~%~%# Extra stabilization/custom forces.~%geometry_msgs/Vector3 random_force~%geometry_msgs/Vector3 keep_distance_force~%geometry_msgs/Vector3 robot_force~%~%# Total forces~%geometry_msgs/Vector3 force~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'GetAllAgentsState-response)))
  "Returns full string definition for message of type 'GetAllAgentsState-response"
  (cl:format cl:nil "pedsim_msgs/AgentStates agent_states~%~%~%================================================================================~%MSG: pedsim_msgs/AgentStates~%Header header~%pedsim_msgs/AgentState[] agent_states~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%string frame_id~%~%================================================================================~%MSG: pedsim_msgs/AgentState~%Header header~%string id~%string type~%string social_state~%geometry_msgs/Pose pose~%geometry_msgs/Twist twist~%pedsim_msgs/AgentForce forces~%string talking_to_id~%string listening_to_id~%geometry_msgs/Vector3 acceleration~%geometry_msgs/Vector3 destination~%float64 direction~%string configuration~%~%uint8 IDLE        = 0~%uint8 WALKING     = 1~%uint8 RUNNING     = 2~%uint8 INTERACTING = 3~%uint8 TALKING     = 4~%uint8 PHONE       = 5~%================================================================================~%MSG: geometry_msgs/Pose~%# A representation of pose in free space, composed of position and orientation. ~%Point position~%Quaternion orientation~%~%================================================================================~%MSG: geometry_msgs/Point~%# This contains the position of a point in free space~%float64 x~%float64 y~%float64 z~%~%================================================================================~%MSG: geometry_msgs/Quaternion~%# This represents an orientation in free space in quaternion form.~%~%float64 x~%float64 y~%float64 z~%float64 w~%~%================================================================================~%MSG: geometry_msgs/Twist~%# This expresses velocity in free space broken into its linear and angular parts.~%Vector3  linear~%Vector3  angular~%~%================================================================================~%MSG: geometry_msgs/Vector3~%# This represents a vector in free space. ~%# It is only meant to represent a direction. Therefore, it does not~%# make sense to apply a translation to it (e.g., when applying a ~%# generic rigid transformation to a Vector3, tf2 will only apply the~%# rotation). If you want your data to be translatable too, use the~%# geometry_msgs/Point message instead.~%~%float64 x~%float64 y~%float64 z~%================================================================================~%MSG: pedsim_msgs/AgentForce~%# Forces acting on an agent.~%~%# Max Speed~%float64 vmax~%~%# Force Factors~%float64 desired_ffactor~%float64 obstacle_ffactor~%float64 social_ffactor~%float64 robot_ffactor~%~%# Basic SFM forces.~%geometry_msgs/Vector3 desired_force~%geometry_msgs/Vector3 obstacle_force~%geometry_msgs/Vector3 social_force~%~%# Additional Group Forces~%geometry_msgs/Vector3 group_coherence_force~%geometry_msgs/Vector3 group_gaze_force~%geometry_msgs/Vector3 group_repulsion_force~%~%# Extra stabilization/custom forces.~%geometry_msgs/Vector3 random_force~%geometry_msgs/Vector3 keep_distance_force~%geometry_msgs/Vector3 robot_force~%~%# Total forces~%geometry_msgs/Vector3 force~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <GetAllAgentsState-response>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'agent_states))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <GetAllAgentsState-response>))
  "Converts a ROS message object to a list"
  (cl:list 'GetAllAgentsState-response
    (cl:cons ':agent_states (agent_states msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'GetAllAgentsState)))
  'GetAllAgentsState-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'GetAllAgentsState)))
  'GetAllAgentsState-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'GetAllAgentsState)))
  "Returns string type for a service object of type '<GetAllAgentsState>"
  "pedsim_srvs/GetAllAgentsState")