; Auto-generated. Do not edit!


(cl:in-package ford_msgs-msg)


;//! \htmlinclude Pose2DStamped.msg.html

(cl:defclass <Pose2DStamped> (roslisp-msg-protocol:ros-message)
  ((header
    :reader header
    :initarg :header
    :type std_msgs-msg:Header
    :initform (cl:make-instance 'std_msgs-msg:Header))
   (pose
    :reader pose
    :initarg :pose
    :type geometry_msgs-msg:Pose2D
    :initform (cl:make-instance 'geometry_msgs-msg:Pose2D))
   (velocity
    :reader velocity
    :initarg :velocity
    :type geometry_msgs-msg:Vector3
    :initform (cl:make-instance 'geometry_msgs-msg:Vector3)))
)

(cl:defclass Pose2DStamped (<Pose2DStamped>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <Pose2DStamped>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'Pose2DStamped)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name ford_msgs-msg:<Pose2DStamped> is deprecated: use ford_msgs-msg:Pose2DStamped instead.")))

(cl:ensure-generic-function 'header-val :lambda-list '(m))
(cl:defmethod header-val ((m <Pose2DStamped>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader ford_msgs-msg:header-val is deprecated.  Use ford_msgs-msg:header instead.")
  (header m))

(cl:ensure-generic-function 'pose-val :lambda-list '(m))
(cl:defmethod pose-val ((m <Pose2DStamped>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader ford_msgs-msg:pose-val is deprecated.  Use ford_msgs-msg:pose instead.")
  (pose m))

(cl:ensure-generic-function 'velocity-val :lambda-list '(m))
(cl:defmethod velocity-val ((m <Pose2DStamped>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader ford_msgs-msg:velocity-val is deprecated.  Use ford_msgs-msg:velocity instead.")
  (velocity m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <Pose2DStamped>) ostream)
  "Serializes a message object of type '<Pose2DStamped>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'header) ostream)
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'pose) ostream)
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'velocity) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <Pose2DStamped>) istream)
  "Deserializes a message object of type '<Pose2DStamped>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'header) istream)
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'pose) istream)
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'velocity) istream)
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<Pose2DStamped>)))
  "Returns string type for a message object of type '<Pose2DStamped>"
  "ford_msgs/Pose2DStamped")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'Pose2DStamped)))
  "Returns string type for a message object of type 'Pose2DStamped"
  "ford_msgs/Pose2DStamped")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<Pose2DStamped>)))
  "Returns md5sum for a message object of type '<Pose2DStamped>"
  "f4e92c15aac28a0fa81ffcfffb7cd21d")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'Pose2DStamped)))
  "Returns md5sum for a message object of type 'Pose2DStamped"
  "f4e92c15aac28a0fa81ffcfffb7cd21d")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<Pose2DStamped>)))
  "Returns full string definition for message of type '<Pose2DStamped>"
  (cl:format cl:nil "std_msgs/Header header~%geometry_msgs/Pose2D pose~%geometry_msgs/Vector3 velocity~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%string frame_id~%~%================================================================================~%MSG: geometry_msgs/Pose2D~%# Deprecated~%# Please use the full 3D pose.~%~%# In general our recommendation is to use a full 3D representation of everything and for 2D specific applications make the appropriate projections into the plane for their calculations but optimally will preserve the 3D information during processing.~%~%# If we have parallel copies of 2D datatypes every UI and other pipeline will end up needing to have dual interfaces to plot everything. And you will end up with not being able to use 3D tools for 2D use cases even if they're completely valid, as you'd have to reimplement it with different inputs and outputs. It's not particularly hard to plot the 2D pose or compute the yaw error for the Pose message and there are already tools and libraries that can do this for you.~%~%~%# This expresses a position and orientation on a 2D manifold.~%~%float64 x~%float64 y~%float64 theta~%~%================================================================================~%MSG: geometry_msgs/Vector3~%# This represents a vector in free space. ~%# It is only meant to represent a direction. Therefore, it does not~%# make sense to apply a translation to it (e.g., when applying a ~%# generic rigid transformation to a Vector3, tf2 will only apply the~%# rotation). If you want your data to be translatable too, use the~%# geometry_msgs/Point message instead.~%~%float64 x~%float64 y~%float64 z~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'Pose2DStamped)))
  "Returns full string definition for message of type 'Pose2DStamped"
  (cl:format cl:nil "std_msgs/Header header~%geometry_msgs/Pose2D pose~%geometry_msgs/Vector3 velocity~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%string frame_id~%~%================================================================================~%MSG: geometry_msgs/Pose2D~%# Deprecated~%# Please use the full 3D pose.~%~%# In general our recommendation is to use a full 3D representation of everything and for 2D specific applications make the appropriate projections into the plane for their calculations but optimally will preserve the 3D information during processing.~%~%# If we have parallel copies of 2D datatypes every UI and other pipeline will end up needing to have dual interfaces to plot everything. And you will end up with not being able to use 3D tools for 2D use cases even if they're completely valid, as you'd have to reimplement it with different inputs and outputs. It's not particularly hard to plot the 2D pose or compute the yaw error for the Pose message and there are already tools and libraries that can do this for you.~%~%~%# This expresses a position and orientation on a 2D manifold.~%~%float64 x~%float64 y~%float64 theta~%~%================================================================================~%MSG: geometry_msgs/Vector3~%# This represents a vector in free space. ~%# It is only meant to represent a direction. Therefore, it does not~%# make sense to apply a translation to it (e.g., when applying a ~%# generic rigid transformation to a Vector3, tf2 will only apply the~%# rotation). If you want your data to be translatable too, use the~%# geometry_msgs/Point message instead.~%~%float64 x~%float64 y~%float64 z~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <Pose2DStamped>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'header))
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'pose))
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'velocity))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <Pose2DStamped>))
  "Converts a ROS message object to a list"
  (cl:list 'Pose2DStamped
    (cl:cons ':header (header msg))
    (cl:cons ':pose (pose msg))
    (cl:cons ':velocity (velocity msg))
))
