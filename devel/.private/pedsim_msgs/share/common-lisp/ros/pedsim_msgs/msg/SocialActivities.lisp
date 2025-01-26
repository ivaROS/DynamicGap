; Auto-generated. Do not edit!


(cl:in-package pedsim_msgs-msg)


;//! \htmlinclude SocialActivities.msg.html

(cl:defclass <SocialActivities> (roslisp-msg-protocol:ros-message)
  ((header
    :reader header
    :initarg :header
    :type std_msgs-msg:Header
    :initform (cl:make-instance 'std_msgs-msg:Header))
   (elements
    :reader elements
    :initarg :elements
    :type (cl:vector pedsim_msgs-msg:SocialActivity)
   :initform (cl:make-array 0 :element-type 'pedsim_msgs-msg:SocialActivity :initial-element (cl:make-instance 'pedsim_msgs-msg:SocialActivity))))
)

(cl:defclass SocialActivities (<SocialActivities>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <SocialActivities>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'SocialActivities)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name pedsim_msgs-msg:<SocialActivities> is deprecated: use pedsim_msgs-msg:SocialActivities instead.")))

(cl:ensure-generic-function 'header-val :lambda-list '(m))
(cl:defmethod header-val ((m <SocialActivities>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader pedsim_msgs-msg:header-val is deprecated.  Use pedsim_msgs-msg:header instead.")
  (header m))

(cl:ensure-generic-function 'elements-val :lambda-list '(m))
(cl:defmethod elements-val ((m <SocialActivities>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader pedsim_msgs-msg:elements-val is deprecated.  Use pedsim_msgs-msg:elements instead.")
  (elements m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <SocialActivities>) ostream)
  "Serializes a message object of type '<SocialActivities>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'header) ostream)
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'elements))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (roslisp-msg-protocol:serialize ele ostream))
   (cl:slot-value msg 'elements))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <SocialActivities>) istream)
  "Deserializes a message object of type '<SocialActivities>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'header) istream)
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'elements) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'elements)))
    (cl:dotimes (i __ros_arr_len)
    (cl:setf (cl:aref vals i) (cl:make-instance 'pedsim_msgs-msg:SocialActivity))
  (roslisp-msg-protocol:deserialize (cl:aref vals i) istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<SocialActivities>)))
  "Returns string type for a message object of type '<SocialActivities>"
  "pedsim_msgs/SocialActivities")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'SocialActivities)))
  "Returns string type for a message object of type 'SocialActivities"
  "pedsim_msgs/SocialActivities")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<SocialActivities>)))
  "Returns md5sum for a message object of type '<SocialActivities>"
  "ecc37bdf8b3c5b2b65fcb47f45b398ae")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'SocialActivities)))
  "Returns md5sum for a message object of type 'SocialActivities"
  "ecc37bdf8b3c5b2b65fcb47f45b398ae")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<SocialActivities>)))
  "Returns full string definition for message of type '<SocialActivities>"
  (cl:format cl:nil "std_msgs/Header     header~%~%# All social activities that have been detected in the current time step,~%# within sensor range of the robot.~%SocialActivity[]    elements~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%string frame_id~%~%================================================================================~%MSG: pedsim_msgs/SocialActivity~%string      type        # see constants below~%float32     confidence  # detection confidence~%~%string[]      track_ids   # IDs of all person tracks involved in the activity, might be one or multiple~%~%~%# Constants for social activity type (just examples at the moment)~%string      TYPE_SHOPPING = shopping~%string      TYPE_STANDING = standing~%string      TYPE_TALKING = talking~%string      TYPE_RUNNING = running~%string      TYPE_INDIVIDUAL_MOVING = individual_moving~%string      TYPE_WAITING_IN_QUEUE = waiting_in_queue~%string      TYPE_LOOKING_AT_INFORMATION_SCREEN = looking_at_information_screen~%string      TYPE_LOOKING_AT_KIOSK = looking_at_kiosk~%string      TYPE_GROUP_ASSEMBLING = group_assembling~%string      TYPE_GROUP_MOVING = group_moving~%string      TYPE_FLOW_WITH_ROBOT = flow~%string      TYPE_ANTIFLOW_AGAINST_ROBOT = antiflow~%string      TYPE_WAITING_FOR_OTHERS = waiting_for_others~%~%#string      TYPE_COMMUNICATING = communicating~%#string      TYPE_TAKING_PHOTOGRAPH = taking_photograph~%#string      TYPE_TALKING_ON_PHONE = talking_on_phone~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'SocialActivities)))
  "Returns full string definition for message of type 'SocialActivities"
  (cl:format cl:nil "std_msgs/Header     header~%~%# All social activities that have been detected in the current time step,~%# within sensor range of the robot.~%SocialActivity[]    elements~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%string frame_id~%~%================================================================================~%MSG: pedsim_msgs/SocialActivity~%string      type        # see constants below~%float32     confidence  # detection confidence~%~%string[]      track_ids   # IDs of all person tracks involved in the activity, might be one or multiple~%~%~%# Constants for social activity type (just examples at the moment)~%string      TYPE_SHOPPING = shopping~%string      TYPE_STANDING = standing~%string      TYPE_TALKING = talking~%string      TYPE_RUNNING = running~%string      TYPE_INDIVIDUAL_MOVING = individual_moving~%string      TYPE_WAITING_IN_QUEUE = waiting_in_queue~%string      TYPE_LOOKING_AT_INFORMATION_SCREEN = looking_at_information_screen~%string      TYPE_LOOKING_AT_KIOSK = looking_at_kiosk~%string      TYPE_GROUP_ASSEMBLING = group_assembling~%string      TYPE_GROUP_MOVING = group_moving~%string      TYPE_FLOW_WITH_ROBOT = flow~%string      TYPE_ANTIFLOW_AGAINST_ROBOT = antiflow~%string      TYPE_WAITING_FOR_OTHERS = waiting_for_others~%~%#string      TYPE_COMMUNICATING = communicating~%#string      TYPE_TAKING_PHOTOGRAPH = taking_photograph~%#string      TYPE_TALKING_ON_PHONE = talking_on_phone~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <SocialActivities>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'header))
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'elements) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ (roslisp-msg-protocol:serialization-length ele))))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <SocialActivities>))
  "Converts a ROS message object to a list"
  (cl:list 'SocialActivities
    (cl:cons ':header (header msg))
    (cl:cons ':elements (elements msg))
))
