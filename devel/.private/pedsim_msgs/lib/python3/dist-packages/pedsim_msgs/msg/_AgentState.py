# This Python file uses the following encoding: utf-8
"""autogenerated by genpy from pedsim_msgs/AgentState.msg. Do not edit."""
import codecs
import sys
python3 = True if sys.hexversion > 0x03000000 else False
import genpy
import struct

import geometry_msgs.msg
import pedsim_msgs.msg
import std_msgs.msg

class AgentState(genpy.Message):
  _md5sum = "870e02a1932fc0a93c5d52d2b22efada"
  _type = "pedsim_msgs/AgentState"
  _has_header = True  # flag to mark the presence of a Header object
  _full_text = """Header header
string id
string type
string social_state
geometry_msgs/Pose pose
geometry_msgs/Twist twist
pedsim_msgs/AgentForce forces
string talking_to_id
string listening_to_id
geometry_msgs/Vector3 acceleration
geometry_msgs/Vector3 destination
float64 direction
string configuration

uint8 IDLE        = 0
uint8 WALKING     = 1
uint8 RUNNING     = 2
uint8 INTERACTING = 3
uint8 TALKING     = 4
uint8 PHONE       = 5
================================================================================
MSG: std_msgs/Header
# Standard metadata for higher-level stamped data types.
# This is generally used to communicate timestamped data 
# in a particular coordinate frame.
# 
# sequence ID: consecutively increasing ID 
uint32 seq
#Two-integer timestamp that is expressed as:
# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')
# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')
# time-handling sugar is provided by the client library
time stamp
#Frame this data is associated with
string frame_id

================================================================================
MSG: geometry_msgs/Pose
# A representation of pose in free space, composed of position and orientation. 
Point position
Quaternion orientation

================================================================================
MSG: geometry_msgs/Point
# This contains the position of a point in free space
float64 x
float64 y
float64 z

================================================================================
MSG: geometry_msgs/Quaternion
# This represents an orientation in free space in quaternion form.

float64 x
float64 y
float64 z
float64 w

================================================================================
MSG: geometry_msgs/Twist
# This expresses velocity in free space broken into its linear and angular parts.
Vector3  linear
Vector3  angular

================================================================================
MSG: geometry_msgs/Vector3
# This represents a vector in free space. 
# It is only meant to represent a direction. Therefore, it does not
# make sense to apply a translation to it (e.g., when applying a 
# generic rigid transformation to a Vector3, tf2 will only apply the
# rotation). If you want your data to be translatable too, use the
# geometry_msgs/Point message instead.

float64 x
float64 y
float64 z
================================================================================
MSG: pedsim_msgs/AgentForce
# Forces acting on an agent.

# Max Speed
float64 vmax

# Force Factors
float64 desired_ffactor
float64 obstacle_ffactor
float64 social_ffactor
float64 robot_ffactor

# Basic SFM forces.
geometry_msgs/Vector3 desired_force
geometry_msgs/Vector3 obstacle_force
geometry_msgs/Vector3 social_force

# Additional Group Forces
geometry_msgs/Vector3 group_coherence_force
geometry_msgs/Vector3 group_gaze_force
geometry_msgs/Vector3 group_repulsion_force

# Extra stabilization/custom forces.
geometry_msgs/Vector3 random_force
geometry_msgs/Vector3 keep_distance_force
geometry_msgs/Vector3 robot_force

# Total forces
geometry_msgs/Vector3 force"""
  # Pseudo-constants
  IDLE = 0
  WALKING = 1
  RUNNING = 2
  INTERACTING = 3
  TALKING = 4
  PHONE = 5

  __slots__ = ['header','id','type','social_state','pose','twist','forces','talking_to_id','listening_to_id','acceleration','destination','direction','configuration']
  _slot_types = ['std_msgs/Header','string','string','string','geometry_msgs/Pose','geometry_msgs/Twist','pedsim_msgs/AgentForce','string','string','geometry_msgs/Vector3','geometry_msgs/Vector3','float64','string']

  def __init__(self, *args, **kwds):
    """
    Constructor. Any message fields that are implicitly/explicitly
    set to None will be assigned a default value. The recommend
    use is keyword arguments as this is more robust to future message
    changes.  You cannot mix in-order arguments and keyword arguments.

    The available fields are:
       header,id,type,social_state,pose,twist,forces,talking_to_id,listening_to_id,acceleration,destination,direction,configuration

    :param args: complete set of field values, in .msg order
    :param kwds: use keyword arguments corresponding to message field names
    to set specific fields.
    """
    if args or kwds:
      super(AgentState, self).__init__(*args, **kwds)
      # message fields cannot be None, assign default values for those that are
      if self.header is None:
        self.header = std_msgs.msg.Header()
      if self.id is None:
        self.id = ''
      if self.type is None:
        self.type = ''
      if self.social_state is None:
        self.social_state = ''
      if self.pose is None:
        self.pose = geometry_msgs.msg.Pose()
      if self.twist is None:
        self.twist = geometry_msgs.msg.Twist()
      if self.forces is None:
        self.forces = pedsim_msgs.msg.AgentForce()
      if self.talking_to_id is None:
        self.talking_to_id = ''
      if self.listening_to_id is None:
        self.listening_to_id = ''
      if self.acceleration is None:
        self.acceleration = geometry_msgs.msg.Vector3()
      if self.destination is None:
        self.destination = geometry_msgs.msg.Vector3()
      if self.direction is None:
        self.direction = 0.
      if self.configuration is None:
        self.configuration = ''
    else:
      self.header = std_msgs.msg.Header()
      self.id = ''
      self.type = ''
      self.social_state = ''
      self.pose = geometry_msgs.msg.Pose()
      self.twist = geometry_msgs.msg.Twist()
      self.forces = pedsim_msgs.msg.AgentForce()
      self.talking_to_id = ''
      self.listening_to_id = ''
      self.acceleration = geometry_msgs.msg.Vector3()
      self.destination = geometry_msgs.msg.Vector3()
      self.direction = 0.
      self.configuration = ''

  def _get_types(self):
    """
    internal API method
    """
    return self._slot_types

  def serialize(self, buff):
    """
    serialize message into buffer
    :param buff: buffer, ``StringIO``
    """
    try:
      _x = self
      buff.write(_get_struct_3I().pack(_x.header.seq, _x.header.stamp.secs, _x.header.stamp.nsecs))
      _x = self.header.frame_id
      length = len(_x)
      if python3 or type(_x) == unicode:
        _x = _x.encode('utf-8')
        length = len(_x)
      buff.write(struct.Struct('<I%ss'%length).pack(length, _x))
      _x = self.id
      length = len(_x)
      if python3 or type(_x) == unicode:
        _x = _x.encode('utf-8')
        length = len(_x)
      buff.write(struct.Struct('<I%ss'%length).pack(length, _x))
      _x = self.type
      length = len(_x)
      if python3 or type(_x) == unicode:
        _x = _x.encode('utf-8')
        length = len(_x)
      buff.write(struct.Struct('<I%ss'%length).pack(length, _x))
      _x = self.social_state
      length = len(_x)
      if python3 or type(_x) == unicode:
        _x = _x.encode('utf-8')
        length = len(_x)
      buff.write(struct.Struct('<I%ss'%length).pack(length, _x))
      _x = self
      buff.write(_get_struct_48d().pack(_x.pose.position.x, _x.pose.position.y, _x.pose.position.z, _x.pose.orientation.x, _x.pose.orientation.y, _x.pose.orientation.z, _x.pose.orientation.w, _x.twist.linear.x, _x.twist.linear.y, _x.twist.linear.z, _x.twist.angular.x, _x.twist.angular.y, _x.twist.angular.z, _x.forces.vmax, _x.forces.desired_ffactor, _x.forces.obstacle_ffactor, _x.forces.social_ffactor, _x.forces.robot_ffactor, _x.forces.desired_force.x, _x.forces.desired_force.y, _x.forces.desired_force.z, _x.forces.obstacle_force.x, _x.forces.obstacle_force.y, _x.forces.obstacle_force.z, _x.forces.social_force.x, _x.forces.social_force.y, _x.forces.social_force.z, _x.forces.group_coherence_force.x, _x.forces.group_coherence_force.y, _x.forces.group_coherence_force.z, _x.forces.group_gaze_force.x, _x.forces.group_gaze_force.y, _x.forces.group_gaze_force.z, _x.forces.group_repulsion_force.x, _x.forces.group_repulsion_force.y, _x.forces.group_repulsion_force.z, _x.forces.random_force.x, _x.forces.random_force.y, _x.forces.random_force.z, _x.forces.keep_distance_force.x, _x.forces.keep_distance_force.y, _x.forces.keep_distance_force.z, _x.forces.robot_force.x, _x.forces.robot_force.y, _x.forces.robot_force.z, _x.forces.force.x, _x.forces.force.y, _x.forces.force.z))
      _x = self.talking_to_id
      length = len(_x)
      if python3 or type(_x) == unicode:
        _x = _x.encode('utf-8')
        length = len(_x)
      buff.write(struct.Struct('<I%ss'%length).pack(length, _x))
      _x = self.listening_to_id
      length = len(_x)
      if python3 or type(_x) == unicode:
        _x = _x.encode('utf-8')
        length = len(_x)
      buff.write(struct.Struct('<I%ss'%length).pack(length, _x))
      _x = self
      buff.write(_get_struct_7d().pack(_x.acceleration.x, _x.acceleration.y, _x.acceleration.z, _x.destination.x, _x.destination.y, _x.destination.z, _x.direction))
      _x = self.configuration
      length = len(_x)
      if python3 or type(_x) == unicode:
        _x = _x.encode('utf-8')
        length = len(_x)
      buff.write(struct.Struct('<I%ss'%length).pack(length, _x))
    except struct.error as se: self._check_types(struct.error("%s: '%s' when writing '%s'" % (type(se), str(se), str(locals().get('_x', self)))))
    except TypeError as te: self._check_types(ValueError("%s: '%s' when writing '%s'" % (type(te), str(te), str(locals().get('_x', self)))))

  def deserialize(self, str):
    """
    unpack serialized message in str into this message instance
    :param str: byte array of serialized message, ``str``
    """
    if python3:
      codecs.lookup_error("rosmsg").msg_type = self._type
    try:
      if self.header is None:
        self.header = std_msgs.msg.Header()
      if self.pose is None:
        self.pose = geometry_msgs.msg.Pose()
      if self.twist is None:
        self.twist = geometry_msgs.msg.Twist()
      if self.forces is None:
        self.forces = pedsim_msgs.msg.AgentForce()
      if self.acceleration is None:
        self.acceleration = geometry_msgs.msg.Vector3()
      if self.destination is None:
        self.destination = geometry_msgs.msg.Vector3()
      end = 0
      _x = self
      start = end
      end += 12
      (_x.header.seq, _x.header.stamp.secs, _x.header.stamp.nsecs,) = _get_struct_3I().unpack(str[start:end])
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      start = end
      end += length
      if python3:
        self.header.frame_id = str[start:end].decode('utf-8', 'rosmsg')
      else:
        self.header.frame_id = str[start:end]
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      start = end
      end += length
      if python3:
        self.id = str[start:end].decode('utf-8', 'rosmsg')
      else:
        self.id = str[start:end]
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      start = end
      end += length
      if python3:
        self.type = str[start:end].decode('utf-8', 'rosmsg')
      else:
        self.type = str[start:end]
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      start = end
      end += length
      if python3:
        self.social_state = str[start:end].decode('utf-8', 'rosmsg')
      else:
        self.social_state = str[start:end]
      _x = self
      start = end
      end += 384
      (_x.pose.position.x, _x.pose.position.y, _x.pose.position.z, _x.pose.orientation.x, _x.pose.orientation.y, _x.pose.orientation.z, _x.pose.orientation.w, _x.twist.linear.x, _x.twist.linear.y, _x.twist.linear.z, _x.twist.angular.x, _x.twist.angular.y, _x.twist.angular.z, _x.forces.vmax, _x.forces.desired_ffactor, _x.forces.obstacle_ffactor, _x.forces.social_ffactor, _x.forces.robot_ffactor, _x.forces.desired_force.x, _x.forces.desired_force.y, _x.forces.desired_force.z, _x.forces.obstacle_force.x, _x.forces.obstacle_force.y, _x.forces.obstacle_force.z, _x.forces.social_force.x, _x.forces.social_force.y, _x.forces.social_force.z, _x.forces.group_coherence_force.x, _x.forces.group_coherence_force.y, _x.forces.group_coherence_force.z, _x.forces.group_gaze_force.x, _x.forces.group_gaze_force.y, _x.forces.group_gaze_force.z, _x.forces.group_repulsion_force.x, _x.forces.group_repulsion_force.y, _x.forces.group_repulsion_force.z, _x.forces.random_force.x, _x.forces.random_force.y, _x.forces.random_force.z, _x.forces.keep_distance_force.x, _x.forces.keep_distance_force.y, _x.forces.keep_distance_force.z, _x.forces.robot_force.x, _x.forces.robot_force.y, _x.forces.robot_force.z, _x.forces.force.x, _x.forces.force.y, _x.forces.force.z,) = _get_struct_48d().unpack(str[start:end])
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      start = end
      end += length
      if python3:
        self.talking_to_id = str[start:end].decode('utf-8', 'rosmsg')
      else:
        self.talking_to_id = str[start:end]
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      start = end
      end += length
      if python3:
        self.listening_to_id = str[start:end].decode('utf-8', 'rosmsg')
      else:
        self.listening_to_id = str[start:end]
      _x = self
      start = end
      end += 56
      (_x.acceleration.x, _x.acceleration.y, _x.acceleration.z, _x.destination.x, _x.destination.y, _x.destination.z, _x.direction,) = _get_struct_7d().unpack(str[start:end])
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      start = end
      end += length
      if python3:
        self.configuration = str[start:end].decode('utf-8', 'rosmsg')
      else:
        self.configuration = str[start:end]
      return self
    except struct.error as e:
      raise genpy.DeserializationError(e)  # most likely buffer underfill


  def serialize_numpy(self, buff, numpy):
    """
    serialize message with numpy array types into buffer
    :param buff: buffer, ``StringIO``
    :param numpy: numpy python module
    """
    try:
      _x = self
      buff.write(_get_struct_3I().pack(_x.header.seq, _x.header.stamp.secs, _x.header.stamp.nsecs))
      _x = self.header.frame_id
      length = len(_x)
      if python3 or type(_x) == unicode:
        _x = _x.encode('utf-8')
        length = len(_x)
      buff.write(struct.Struct('<I%ss'%length).pack(length, _x))
      _x = self.id
      length = len(_x)
      if python3 or type(_x) == unicode:
        _x = _x.encode('utf-8')
        length = len(_x)
      buff.write(struct.Struct('<I%ss'%length).pack(length, _x))
      _x = self.type
      length = len(_x)
      if python3 or type(_x) == unicode:
        _x = _x.encode('utf-8')
        length = len(_x)
      buff.write(struct.Struct('<I%ss'%length).pack(length, _x))
      _x = self.social_state
      length = len(_x)
      if python3 or type(_x) == unicode:
        _x = _x.encode('utf-8')
        length = len(_x)
      buff.write(struct.Struct('<I%ss'%length).pack(length, _x))
      _x = self
      buff.write(_get_struct_48d().pack(_x.pose.position.x, _x.pose.position.y, _x.pose.position.z, _x.pose.orientation.x, _x.pose.orientation.y, _x.pose.orientation.z, _x.pose.orientation.w, _x.twist.linear.x, _x.twist.linear.y, _x.twist.linear.z, _x.twist.angular.x, _x.twist.angular.y, _x.twist.angular.z, _x.forces.vmax, _x.forces.desired_ffactor, _x.forces.obstacle_ffactor, _x.forces.social_ffactor, _x.forces.robot_ffactor, _x.forces.desired_force.x, _x.forces.desired_force.y, _x.forces.desired_force.z, _x.forces.obstacle_force.x, _x.forces.obstacle_force.y, _x.forces.obstacle_force.z, _x.forces.social_force.x, _x.forces.social_force.y, _x.forces.social_force.z, _x.forces.group_coherence_force.x, _x.forces.group_coherence_force.y, _x.forces.group_coherence_force.z, _x.forces.group_gaze_force.x, _x.forces.group_gaze_force.y, _x.forces.group_gaze_force.z, _x.forces.group_repulsion_force.x, _x.forces.group_repulsion_force.y, _x.forces.group_repulsion_force.z, _x.forces.random_force.x, _x.forces.random_force.y, _x.forces.random_force.z, _x.forces.keep_distance_force.x, _x.forces.keep_distance_force.y, _x.forces.keep_distance_force.z, _x.forces.robot_force.x, _x.forces.robot_force.y, _x.forces.robot_force.z, _x.forces.force.x, _x.forces.force.y, _x.forces.force.z))
      _x = self.talking_to_id
      length = len(_x)
      if python3 or type(_x) == unicode:
        _x = _x.encode('utf-8')
        length = len(_x)
      buff.write(struct.Struct('<I%ss'%length).pack(length, _x))
      _x = self.listening_to_id
      length = len(_x)
      if python3 or type(_x) == unicode:
        _x = _x.encode('utf-8')
        length = len(_x)
      buff.write(struct.Struct('<I%ss'%length).pack(length, _x))
      _x = self
      buff.write(_get_struct_7d().pack(_x.acceleration.x, _x.acceleration.y, _x.acceleration.z, _x.destination.x, _x.destination.y, _x.destination.z, _x.direction))
      _x = self.configuration
      length = len(_x)
      if python3 or type(_x) == unicode:
        _x = _x.encode('utf-8')
        length = len(_x)
      buff.write(struct.Struct('<I%ss'%length).pack(length, _x))
    except struct.error as se: self._check_types(struct.error("%s: '%s' when writing '%s'" % (type(se), str(se), str(locals().get('_x', self)))))
    except TypeError as te: self._check_types(ValueError("%s: '%s' when writing '%s'" % (type(te), str(te), str(locals().get('_x', self)))))

  def deserialize_numpy(self, str, numpy):
    """
    unpack serialized message in str into this message instance using numpy for array types
    :param str: byte array of serialized message, ``str``
    :param numpy: numpy python module
    """
    if python3:
      codecs.lookup_error("rosmsg").msg_type = self._type
    try:
      if self.header is None:
        self.header = std_msgs.msg.Header()
      if self.pose is None:
        self.pose = geometry_msgs.msg.Pose()
      if self.twist is None:
        self.twist = geometry_msgs.msg.Twist()
      if self.forces is None:
        self.forces = pedsim_msgs.msg.AgentForce()
      if self.acceleration is None:
        self.acceleration = geometry_msgs.msg.Vector3()
      if self.destination is None:
        self.destination = geometry_msgs.msg.Vector3()
      end = 0
      _x = self
      start = end
      end += 12
      (_x.header.seq, _x.header.stamp.secs, _x.header.stamp.nsecs,) = _get_struct_3I().unpack(str[start:end])
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      start = end
      end += length
      if python3:
        self.header.frame_id = str[start:end].decode('utf-8', 'rosmsg')
      else:
        self.header.frame_id = str[start:end]
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      start = end
      end += length
      if python3:
        self.id = str[start:end].decode('utf-8', 'rosmsg')
      else:
        self.id = str[start:end]
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      start = end
      end += length
      if python3:
        self.type = str[start:end].decode('utf-8', 'rosmsg')
      else:
        self.type = str[start:end]
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      start = end
      end += length
      if python3:
        self.social_state = str[start:end].decode('utf-8', 'rosmsg')
      else:
        self.social_state = str[start:end]
      _x = self
      start = end
      end += 384
      (_x.pose.position.x, _x.pose.position.y, _x.pose.position.z, _x.pose.orientation.x, _x.pose.orientation.y, _x.pose.orientation.z, _x.pose.orientation.w, _x.twist.linear.x, _x.twist.linear.y, _x.twist.linear.z, _x.twist.angular.x, _x.twist.angular.y, _x.twist.angular.z, _x.forces.vmax, _x.forces.desired_ffactor, _x.forces.obstacle_ffactor, _x.forces.social_ffactor, _x.forces.robot_ffactor, _x.forces.desired_force.x, _x.forces.desired_force.y, _x.forces.desired_force.z, _x.forces.obstacle_force.x, _x.forces.obstacle_force.y, _x.forces.obstacle_force.z, _x.forces.social_force.x, _x.forces.social_force.y, _x.forces.social_force.z, _x.forces.group_coherence_force.x, _x.forces.group_coherence_force.y, _x.forces.group_coherence_force.z, _x.forces.group_gaze_force.x, _x.forces.group_gaze_force.y, _x.forces.group_gaze_force.z, _x.forces.group_repulsion_force.x, _x.forces.group_repulsion_force.y, _x.forces.group_repulsion_force.z, _x.forces.random_force.x, _x.forces.random_force.y, _x.forces.random_force.z, _x.forces.keep_distance_force.x, _x.forces.keep_distance_force.y, _x.forces.keep_distance_force.z, _x.forces.robot_force.x, _x.forces.robot_force.y, _x.forces.robot_force.z, _x.forces.force.x, _x.forces.force.y, _x.forces.force.z,) = _get_struct_48d().unpack(str[start:end])
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      start = end
      end += length
      if python3:
        self.talking_to_id = str[start:end].decode('utf-8', 'rosmsg')
      else:
        self.talking_to_id = str[start:end]
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      start = end
      end += length
      if python3:
        self.listening_to_id = str[start:end].decode('utf-8', 'rosmsg')
      else:
        self.listening_to_id = str[start:end]
      _x = self
      start = end
      end += 56
      (_x.acceleration.x, _x.acceleration.y, _x.acceleration.z, _x.destination.x, _x.destination.y, _x.destination.z, _x.direction,) = _get_struct_7d().unpack(str[start:end])
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      start = end
      end += length
      if python3:
        self.configuration = str[start:end].decode('utf-8', 'rosmsg')
      else:
        self.configuration = str[start:end]
      return self
    except struct.error as e:
      raise genpy.DeserializationError(e)  # most likely buffer underfill

_struct_I = genpy.struct_I
def _get_struct_I():
    global _struct_I
    return _struct_I
_struct_3I = None
def _get_struct_3I():
    global _struct_3I
    if _struct_3I is None:
        _struct_3I = struct.Struct("<3I")
    return _struct_3I
_struct_48d = None
def _get_struct_48d():
    global _struct_48d
    if _struct_48d is None:
        _struct_48d = struct.Struct("<48d")
    return _struct_48d
_struct_7d = None
def _get_struct_7d():
    global _struct_7d
    if _struct_7d is None:
        _struct_7d = struct.Struct("<7d")
    return _struct_7d
