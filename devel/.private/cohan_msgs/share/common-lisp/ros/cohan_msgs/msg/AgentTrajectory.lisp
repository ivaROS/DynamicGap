; Auto-generated. Do not edit!


(cl:in-package cohan_msgs-msg)


;//! \htmlinclude AgentTrajectory.msg.html

(cl:defclass <AgentTrajectory> (roslisp-msg-protocol:ros-message)
  ((header
    :reader header
    :initarg :header
    :type std_msgs-msg:Header
    :initform (cl:make-instance 'std_msgs-msg:Header))
   (id
    :reader id
    :initarg :id
    :type cl:integer
    :initform 0)
   (type
    :reader type
    :initarg :type
    :type cl:fixnum
    :initform 0)
   (trajectory
    :reader trajectory
    :initarg :trajectory
    :type cohan_msgs-msg:Trajectory
    :initform (cl:make-instance 'cohan_msgs-msg:Trajectory)))
)

(cl:defclass AgentTrajectory (<AgentTrajectory>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <AgentTrajectory>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'AgentTrajectory)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name cohan_msgs-msg:<AgentTrajectory> is deprecated: use cohan_msgs-msg:AgentTrajectory instead.")))

(cl:ensure-generic-function 'header-val :lambda-list '(m))
(cl:defmethod header-val ((m <AgentTrajectory>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader cohan_msgs-msg:header-val is deprecated.  Use cohan_msgs-msg:header instead.")
  (header m))

(cl:ensure-generic-function 'id-val :lambda-list '(m))
(cl:defmethod id-val ((m <AgentTrajectory>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader cohan_msgs-msg:id-val is deprecated.  Use cohan_msgs-msg:id instead.")
  (id m))

(cl:ensure-generic-function 'type-val :lambda-list '(m))
(cl:defmethod type-val ((m <AgentTrajectory>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader cohan_msgs-msg:type-val is deprecated.  Use cohan_msgs-msg:type instead.")
  (type m))

(cl:ensure-generic-function 'trajectory-val :lambda-list '(m))
(cl:defmethod trajectory-val ((m <AgentTrajectory>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader cohan_msgs-msg:trajectory-val is deprecated.  Use cohan_msgs-msg:trajectory instead.")
  (trajectory m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <AgentTrajectory>) ostream)
  "Serializes a message object of type '<AgentTrajectory>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'header) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'id)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'id)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'id)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'id)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 32) (cl:slot-value msg 'id)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 40) (cl:slot-value msg 'id)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 48) (cl:slot-value msg 'id)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 56) (cl:slot-value msg 'id)) ostream)
  (cl:let* ((signed (cl:slot-value msg 'type)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 256) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    )
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'trajectory) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <AgentTrajectory>) istream)
  "Deserializes a message object of type '<AgentTrajectory>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'header) istream)
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'id)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'id)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'id)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'id)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 32) (cl:slot-value msg 'id)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 40) (cl:slot-value msg 'id)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 48) (cl:slot-value msg 'id)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 56) (cl:slot-value msg 'id)) (cl:read-byte istream))
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'type) (cl:if (cl:< unsigned 128) unsigned (cl:- unsigned 256))))
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'trajectory) istream)
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<AgentTrajectory>)))
  "Returns string type for a message object of type '<AgentTrajectory>"
  "cohan_msgs/AgentTrajectory")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'AgentTrajectory)))
  "Returns string type for a message object of type 'AgentTrajectory"
  "cohan_msgs/AgentTrajectory")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<AgentTrajectory>)))
  "Returns md5sum for a message object of type '<AgentTrajectory>"
  "c7853675d8929fd833768dcfd8f02698")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'AgentTrajectory)))
  "Returns md5sum for a message object of type 'AgentTrajectory"
  "c7853675d8929fd833768dcfd8f02698")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<AgentTrajectory>)))
  "Returns full string definition for message of type '<AgentTrajectory>"
  (cl:format cl:nil "std_msgs/Header         header~%uint64                  id~%int8                    type~%cohan_msgs/Trajectory    trajectory~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%string frame_id~%~%================================================================================~%MSG: cohan_msgs/Trajectory~%std_msgs/Header                  header~%cohan_msgs/TrajectoryPoint[]     points~%~%================================================================================~%MSG: cohan_msgs/TrajectoryPoint~%geometry_msgs/Transform     transform~%geometry_msgs/Twist         velocity~%geometry_msgs/Twist         acceleration~%duration                    time_from_start~%================================================================================~%MSG: geometry_msgs/Transform~%# This represents the transform between two coordinate frames in free space.~%~%Vector3 translation~%Quaternion rotation~%~%================================================================================~%MSG: geometry_msgs/Vector3~%# This represents a vector in free space. ~%# It is only meant to represent a direction. Therefore, it does not~%# make sense to apply a translation to it (e.g., when applying a ~%# generic rigid transformation to a Vector3, tf2 will only apply the~%# rotation). If you want your data to be translatable too, use the~%# geometry_msgs/Point message instead.~%~%float64 x~%float64 y~%float64 z~%================================================================================~%MSG: geometry_msgs/Quaternion~%# This represents an orientation in free space in quaternion form.~%~%float64 x~%float64 y~%float64 z~%float64 w~%~%================================================================================~%MSG: geometry_msgs/Twist~%# This expresses velocity in free space broken into its linear and angular parts.~%Vector3  linear~%Vector3  angular~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'AgentTrajectory)))
  "Returns full string definition for message of type 'AgentTrajectory"
  (cl:format cl:nil "std_msgs/Header         header~%uint64                  id~%int8                    type~%cohan_msgs/Trajectory    trajectory~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%string frame_id~%~%================================================================================~%MSG: cohan_msgs/Trajectory~%std_msgs/Header                  header~%cohan_msgs/TrajectoryPoint[]     points~%~%================================================================================~%MSG: cohan_msgs/TrajectoryPoint~%geometry_msgs/Transform     transform~%geometry_msgs/Twist         velocity~%geometry_msgs/Twist         acceleration~%duration                    time_from_start~%================================================================================~%MSG: geometry_msgs/Transform~%# This represents the transform between two coordinate frames in free space.~%~%Vector3 translation~%Quaternion rotation~%~%================================================================================~%MSG: geometry_msgs/Vector3~%# This represents a vector in free space. ~%# It is only meant to represent a direction. Therefore, it does not~%# make sense to apply a translation to it (e.g., when applying a ~%# generic rigid transformation to a Vector3, tf2 will only apply the~%# rotation). If you want your data to be translatable too, use the~%# geometry_msgs/Point message instead.~%~%float64 x~%float64 y~%float64 z~%================================================================================~%MSG: geometry_msgs/Quaternion~%# This represents an orientation in free space in quaternion form.~%~%float64 x~%float64 y~%float64 z~%float64 w~%~%================================================================================~%MSG: geometry_msgs/Twist~%# This expresses velocity in free space broken into its linear and angular parts.~%Vector3  linear~%Vector3  angular~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <AgentTrajectory>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'header))
     8
     1
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'trajectory))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <AgentTrajectory>))
  "Converts a ROS message object to a list"
  (cl:list 'AgentTrajectory
    (cl:cons ':header (header msg))
    (cl:cons ':id (id msg))
    (cl:cons ':type (type msg))
    (cl:cons ':trajectory (trajectory msg))
))
