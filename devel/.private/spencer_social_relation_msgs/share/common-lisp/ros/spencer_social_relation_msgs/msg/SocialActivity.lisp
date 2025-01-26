; Auto-generated. Do not edit!


(cl:in-package spencer_social_relation_msgs-msg)


;//! \htmlinclude SocialActivity.msg.html

(cl:defclass <SocialActivity> (roslisp-msg-protocol:ros-message)
  ((type
    :reader type
    :initarg :type
    :type cl:string
    :initform "")
   (confidence
    :reader confidence
    :initarg :confidence
    :type cl:float
    :initform 0.0)
   (track_ids
    :reader track_ids
    :initarg :track_ids
    :type (cl:vector cl:string)
   :initform (cl:make-array 0 :element-type 'cl:string :initial-element "")))
)

(cl:defclass SocialActivity (<SocialActivity>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <SocialActivity>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'SocialActivity)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name spencer_social_relation_msgs-msg:<SocialActivity> is deprecated: use spencer_social_relation_msgs-msg:SocialActivity instead.")))

(cl:ensure-generic-function 'type-val :lambda-list '(m))
(cl:defmethod type-val ((m <SocialActivity>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader spencer_social_relation_msgs-msg:type-val is deprecated.  Use spencer_social_relation_msgs-msg:type instead.")
  (type m))

(cl:ensure-generic-function 'confidence-val :lambda-list '(m))
(cl:defmethod confidence-val ((m <SocialActivity>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader spencer_social_relation_msgs-msg:confidence-val is deprecated.  Use spencer_social_relation_msgs-msg:confidence instead.")
  (confidence m))

(cl:ensure-generic-function 'track_ids-val :lambda-list '(m))
(cl:defmethod track_ids-val ((m <SocialActivity>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader spencer_social_relation_msgs-msg:track_ids-val is deprecated.  Use spencer_social_relation_msgs-msg:track_ids instead.")
  (track_ids m))
(cl:defmethod roslisp-msg-protocol:symbol-codes ((msg-type (cl:eql '<SocialActivity>)))
    "Constants for message type '<SocialActivity>"
  '((:TYPE_SHOPPING . "shopping")
    (:TYPE_STANDING . "standing")
    (:TYPE_INDIVIDUAL_MOVING . "individual_moving")
    (:TYPE_WAITING_IN_QUEUE . "waiting_in_queue")
    (:TYPE_LOOKING_AT_INFORMATION_SCREEN . "looking_at_information_screen")
    (:TYPE_LOOKING_AT_KIOSK . "looking_at_kiosk")
    (:TYPE_GROUP_ASSEMBLING . "group_assembling")
    (:TYPE_GROUP_MOVING . "group_moving")
    (:TYPE_FLOW_WITH_ROBOT . "flow")
    (:TYPE_ANTIFLOW_AGAINST_ROBOT . "antiflow")
    (:TYPE_WAITING_FOR_OTHERS . "waiting_for_others")
    (:TYPE_LOOKING_FOR_HELP . "looking_for_help"))
)
(cl:defmethod roslisp-msg-protocol:symbol-codes ((msg-type (cl:eql 'SocialActivity)))
    "Constants for message type 'SocialActivity"
  '((:TYPE_SHOPPING . "shopping")
    (:TYPE_STANDING . "standing")
    (:TYPE_INDIVIDUAL_MOVING . "individual_moving")
    (:TYPE_WAITING_IN_QUEUE . "waiting_in_queue")
    (:TYPE_LOOKING_AT_INFORMATION_SCREEN . "looking_at_information_screen")
    (:TYPE_LOOKING_AT_KIOSK . "looking_at_kiosk")
    (:TYPE_GROUP_ASSEMBLING . "group_assembling")
    (:TYPE_GROUP_MOVING . "group_moving")
    (:TYPE_FLOW_WITH_ROBOT . "flow")
    (:TYPE_ANTIFLOW_AGAINST_ROBOT . "antiflow")
    (:TYPE_WAITING_FOR_OTHERS . "waiting_for_others")
    (:TYPE_LOOKING_FOR_HELP . "looking_for_help"))
)
(cl:defmethod roslisp-msg-protocol:serialize ((msg <SocialActivity>) ostream)
  "Serializes a message object of type '<SocialActivity>"
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'type))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'type))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'confidence))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'track_ids))))
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
   (cl:slot-value msg 'track_ids))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <SocialActivity>) istream)
  "Deserializes a message object of type '<SocialActivity>"
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'type) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'type) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'confidence) (roslisp-utils:decode-single-float-bits bits)))
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'track_ids) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'track_ids)))
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
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<SocialActivity>)))
  "Returns string type for a message object of type '<SocialActivity>"
  "spencer_social_relation_msgs/SocialActivity")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'SocialActivity)))
  "Returns string type for a message object of type 'SocialActivity"
  "spencer_social_relation_msgs/SocialActivity")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<SocialActivity>)))
  "Returns md5sum for a message object of type '<SocialActivity>"
  "da62314a942310457724e5a4a9e960d0")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'SocialActivity)))
  "Returns md5sum for a message object of type 'SocialActivity"
  "da62314a942310457724e5a4a9e960d0")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<SocialActivity>)))
  "Returns full string definition for message of type '<SocialActivity>"
  (cl:format cl:nil "string      type        # see constants below~%float32     confidence  # detection confidence~%~%string[]      track_ids   # IDs of all person tracks involved in the activity, might be one or multiple~%~%~%# Constants for social activity type (just examples at the moment)~%string      TYPE_SHOPPING = shopping~%string      TYPE_STANDING = standing~%string      TYPE_INDIVIDUAL_MOVING = individual_moving~%string      TYPE_WAITING_IN_QUEUE = waiting_in_queue~%string      TYPE_LOOKING_AT_INFORMATION_SCREEN = looking_at_information_screen~%string      TYPE_LOOKING_AT_KIOSK = looking_at_kiosk~%string      TYPE_GROUP_ASSEMBLING = group_assembling~%string      TYPE_GROUP_MOVING = group_moving~%string      TYPE_FLOW_WITH_ROBOT = flow~%string      TYPE_ANTIFLOW_AGAINST_ROBOT = antiflow~%string      TYPE_WAITING_FOR_OTHERS = waiting_for_others~%string      TYPE_LOOKING_FOR_HELP = looking_for_help~%~%~%#string      TYPE_COMMUNICATING = communicating~%#string      TYPE_TAKING_PHOTOGRAPH = taking_photograph~%#string      TYPE_TALKING_ON_PHONE = talking_on_phone~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'SocialActivity)))
  "Returns full string definition for message of type 'SocialActivity"
  (cl:format cl:nil "string      type        # see constants below~%float32     confidence  # detection confidence~%~%string[]      track_ids   # IDs of all person tracks involved in the activity, might be one or multiple~%~%~%# Constants for social activity type (just examples at the moment)~%string      TYPE_SHOPPING = shopping~%string      TYPE_STANDING = standing~%string      TYPE_INDIVIDUAL_MOVING = individual_moving~%string      TYPE_WAITING_IN_QUEUE = waiting_in_queue~%string      TYPE_LOOKING_AT_INFORMATION_SCREEN = looking_at_information_screen~%string      TYPE_LOOKING_AT_KIOSK = looking_at_kiosk~%string      TYPE_GROUP_ASSEMBLING = group_assembling~%string      TYPE_GROUP_MOVING = group_moving~%string      TYPE_FLOW_WITH_ROBOT = flow~%string      TYPE_ANTIFLOW_AGAINST_ROBOT = antiflow~%string      TYPE_WAITING_FOR_OTHERS = waiting_for_others~%string      TYPE_LOOKING_FOR_HELP = looking_for_help~%~%~%#string      TYPE_COMMUNICATING = communicating~%#string      TYPE_TAKING_PHOTOGRAPH = taking_photograph~%#string      TYPE_TALKING_ON_PHONE = talking_on_phone~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <SocialActivity>))
  (cl:+ 0
     4 (cl:length (cl:slot-value msg 'type))
     4
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'track_ids) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ 4 (cl:length ele))))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <SocialActivity>))
  "Converts a ROS message object to a list"
  (cl:list 'SocialActivity
    (cl:cons ':type (type msg))
    (cl:cons ':confidence (confidence msg))
    (cl:cons ':track_ids (track_ids msg))
))
