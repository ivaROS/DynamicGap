# This Python file uses the following encoding: utf-8
"""autogenerated by genpy from agent_path_prediction/AgentPosePredictRequest.msg. Do not edit."""
import codecs
import sys
python3 = True if sys.hexversion > 0x03000000 else False
import genpy
import struct


class AgentPosePredictRequest(genpy.Message):
  _md5sum = "770992d52ec17eb262325de18b4c8f01"
  _type = "agent_path_prediction/AgentPosePredictRequest"
  _has_header = False  # flag to mark the presence of a Header object
  _full_text = """# request constants
uint8 VELOCITY_SCALE=0
uint8 VELOCITY_OBSTACLE=1
uint8 EXTERNAL=2
uint8 BEHIND_ROBOT=3
uint8 PREDICTED_GOAL=4
# request fields
uint8                               type
float64[]                           predict_times
int64[]                             ids
"""
  # Pseudo-constants
  VELOCITY_SCALE = 0
  VELOCITY_OBSTACLE = 1
  EXTERNAL = 2
  BEHIND_ROBOT = 3
  PREDICTED_GOAL = 4

  __slots__ = ['type','predict_times','ids']
  _slot_types = ['uint8','float64[]','int64[]']

  def __init__(self, *args, **kwds):
    """
    Constructor. Any message fields that are implicitly/explicitly
    set to None will be assigned a default value. The recommend
    use is keyword arguments as this is more robust to future message
    changes.  You cannot mix in-order arguments and keyword arguments.

    The available fields are:
       type,predict_times,ids

    :param args: complete set of field values, in .msg order
    :param kwds: use keyword arguments corresponding to message field names
    to set specific fields.
    """
    if args or kwds:
      super(AgentPosePredictRequest, self).__init__(*args, **kwds)
      # message fields cannot be None, assign default values for those that are
      if self.type is None:
        self.type = 0
      if self.predict_times is None:
        self.predict_times = []
      if self.ids is None:
        self.ids = []
    else:
      self.type = 0
      self.predict_times = []
      self.ids = []

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
      _x = self.type
      buff.write(_get_struct_B().pack(_x))
      length = len(self.predict_times)
      buff.write(_struct_I.pack(length))
      pattern = '<%sd'%length
      buff.write(struct.Struct(pattern).pack(*self.predict_times))
      length = len(self.ids)
      buff.write(_struct_I.pack(length))
      pattern = '<%sq'%length
      buff.write(struct.Struct(pattern).pack(*self.ids))
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
      end = 0
      start = end
      end += 1
      (self.type,) = _get_struct_B().unpack(str[start:end])
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      pattern = '<%sd'%length
      start = end
      s = struct.Struct(pattern)
      end += s.size
      self.predict_times = s.unpack(str[start:end])
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      pattern = '<%sq'%length
      start = end
      s = struct.Struct(pattern)
      end += s.size
      self.ids = s.unpack(str[start:end])
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
      _x = self.type
      buff.write(_get_struct_B().pack(_x))
      length = len(self.predict_times)
      buff.write(_struct_I.pack(length))
      pattern = '<%sd'%length
      buff.write(self.predict_times.tostring())
      length = len(self.ids)
      buff.write(_struct_I.pack(length))
      pattern = '<%sq'%length
      buff.write(self.ids.tostring())
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
      end = 0
      start = end
      end += 1
      (self.type,) = _get_struct_B().unpack(str[start:end])
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      pattern = '<%sd'%length
      start = end
      s = struct.Struct(pattern)
      end += s.size
      self.predict_times = numpy.frombuffer(str[start:end], dtype=numpy.float64, count=length)
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      pattern = '<%sq'%length
      start = end
      s = struct.Struct(pattern)
      end += s.size
      self.ids = numpy.frombuffer(str[start:end], dtype=numpy.int64, count=length)
      return self
    except struct.error as e:
      raise genpy.DeserializationError(e)  # most likely buffer underfill

_struct_I = genpy.struct_I
def _get_struct_I():
    global _struct_I
    return _struct_I
_struct_B = None
def _get_struct_B():
    global _struct_B
    if _struct_B is None:
        _struct_B = struct.Struct("<B")
    return _struct_B
# This Python file uses the following encoding: utf-8
"""autogenerated by genpy from agent_path_prediction/AgentPosePredictResponse.msg. Do not edit."""
import codecs
import sys
python3 = True if sys.hexversion > 0x03000000 else False
import genpy
import struct

import agent_path_prediction.msg
import geometry_msgs.msg
import std_msgs.msg

class AgentPosePredictResponse(genpy.Message):
  _md5sum = "a5b2c241f62055f1098b5d523ca4f073"
  _type = "agent_path_prediction/AgentPosePredictResponse"
  _has_header = False  # flag to mark the presence of a Header object
  _full_text = """# response fields
agent_path_prediction/PredictedPoses[]    predicted_agents_poses


================================================================================
MSG: agent_path_prediction/PredictedPoses
uint64                                      id
geometry_msgs/PoseWithCovarianceStamped[]   poses
geometry_msgs/TwistStamped                  start_velocity

================================================================================
MSG: geometry_msgs/PoseWithCovarianceStamped
# This expresses an estimated pose with a reference coordinate frame and timestamp

Header header
PoseWithCovariance pose

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
MSG: geometry_msgs/PoseWithCovariance
# This represents a pose in free space with uncertainty.

Pose pose

# Row-major representation of the 6x6 covariance matrix
# The orientation parameters use a fixed-axis representation.
# In order, the parameters are:
# (x, y, z, rotation about X axis, rotation about Y axis, rotation about Z axis)
float64[36] covariance

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
MSG: geometry_msgs/TwistStamped
# A twist with reference coordinate frame and timestamp
Header header
Twist twist

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
float64 z"""
  __slots__ = ['predicted_agents_poses']
  _slot_types = ['agent_path_prediction/PredictedPoses[]']

  def __init__(self, *args, **kwds):
    """
    Constructor. Any message fields that are implicitly/explicitly
    set to None will be assigned a default value. The recommend
    use is keyword arguments as this is more robust to future message
    changes.  You cannot mix in-order arguments and keyword arguments.

    The available fields are:
       predicted_agents_poses

    :param args: complete set of field values, in .msg order
    :param kwds: use keyword arguments corresponding to message field names
    to set specific fields.
    """
    if args or kwds:
      super(AgentPosePredictResponse, self).__init__(*args, **kwds)
      # message fields cannot be None, assign default values for those that are
      if self.predicted_agents_poses is None:
        self.predicted_agents_poses = []
    else:
      self.predicted_agents_poses = []

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
      length = len(self.predicted_agents_poses)
      buff.write(_struct_I.pack(length))
      for val1 in self.predicted_agents_poses:
        _x = val1.id
        buff.write(_get_struct_Q().pack(_x))
        length = len(val1.poses)
        buff.write(_struct_I.pack(length))
        for val2 in val1.poses:
          _v1 = val2.header
          _x = _v1.seq
          buff.write(_get_struct_I().pack(_x))
          _v2 = _v1.stamp
          _x = _v2
          buff.write(_get_struct_2I().pack(_x.secs, _x.nsecs))
          _x = _v1.frame_id
          length = len(_x)
          if python3 or type(_x) == unicode:
            _x = _x.encode('utf-8')
            length = len(_x)
          buff.write(struct.Struct('<I%ss'%length).pack(length, _x))
          _v3 = val2.pose
          _v4 = _v3.pose
          _v5 = _v4.position
          _x = _v5
          buff.write(_get_struct_3d().pack(_x.x, _x.y, _x.z))
          _v6 = _v4.orientation
          _x = _v6
          buff.write(_get_struct_4d().pack(_x.x, _x.y, _x.z, _x.w))
          buff.write(_get_struct_36d().pack(*_v3.covariance))
        _v7 = val1.start_velocity
        _v8 = _v7.header
        _x = _v8.seq
        buff.write(_get_struct_I().pack(_x))
        _v9 = _v8.stamp
        _x = _v9
        buff.write(_get_struct_2I().pack(_x.secs, _x.nsecs))
        _x = _v8.frame_id
        length = len(_x)
        if python3 or type(_x) == unicode:
          _x = _x.encode('utf-8')
          length = len(_x)
        buff.write(struct.Struct('<I%ss'%length).pack(length, _x))
        _v10 = _v7.twist
        _v11 = _v10.linear
        _x = _v11
        buff.write(_get_struct_3d().pack(_x.x, _x.y, _x.z))
        _v12 = _v10.angular
        _x = _v12
        buff.write(_get_struct_3d().pack(_x.x, _x.y, _x.z))
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
      if self.predicted_agents_poses is None:
        self.predicted_agents_poses = None
      end = 0
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      self.predicted_agents_poses = []
      for i in range(0, length):
        val1 = agent_path_prediction.msg.PredictedPoses()
        start = end
        end += 8
        (val1.id,) = _get_struct_Q().unpack(str[start:end])
        start = end
        end += 4
        (length,) = _struct_I.unpack(str[start:end])
        val1.poses = []
        for i in range(0, length):
          val2 = geometry_msgs.msg.PoseWithCovarianceStamped()
          _v13 = val2.header
          start = end
          end += 4
          (_v13.seq,) = _get_struct_I().unpack(str[start:end])
          _v14 = _v13.stamp
          _x = _v14
          start = end
          end += 8
          (_x.secs, _x.nsecs,) = _get_struct_2I().unpack(str[start:end])
          start = end
          end += 4
          (length,) = _struct_I.unpack(str[start:end])
          start = end
          end += length
          if python3:
            _v13.frame_id = str[start:end].decode('utf-8', 'rosmsg')
          else:
            _v13.frame_id = str[start:end]
          _v15 = val2.pose
          _v16 = _v15.pose
          _v17 = _v16.position
          _x = _v17
          start = end
          end += 24
          (_x.x, _x.y, _x.z,) = _get_struct_3d().unpack(str[start:end])
          _v18 = _v16.orientation
          _x = _v18
          start = end
          end += 32
          (_x.x, _x.y, _x.z, _x.w,) = _get_struct_4d().unpack(str[start:end])
          start = end
          end += 288
          _v15.covariance = _get_struct_36d().unpack(str[start:end])
          val1.poses.append(val2)
        _v19 = val1.start_velocity
        _v20 = _v19.header
        start = end
        end += 4
        (_v20.seq,) = _get_struct_I().unpack(str[start:end])
        _v21 = _v20.stamp
        _x = _v21
        start = end
        end += 8
        (_x.secs, _x.nsecs,) = _get_struct_2I().unpack(str[start:end])
        start = end
        end += 4
        (length,) = _struct_I.unpack(str[start:end])
        start = end
        end += length
        if python3:
          _v20.frame_id = str[start:end].decode('utf-8', 'rosmsg')
        else:
          _v20.frame_id = str[start:end]
        _v22 = _v19.twist
        _v23 = _v22.linear
        _x = _v23
        start = end
        end += 24
        (_x.x, _x.y, _x.z,) = _get_struct_3d().unpack(str[start:end])
        _v24 = _v22.angular
        _x = _v24
        start = end
        end += 24
        (_x.x, _x.y, _x.z,) = _get_struct_3d().unpack(str[start:end])
        self.predicted_agents_poses.append(val1)
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
      length = len(self.predicted_agents_poses)
      buff.write(_struct_I.pack(length))
      for val1 in self.predicted_agents_poses:
        _x = val1.id
        buff.write(_get_struct_Q().pack(_x))
        length = len(val1.poses)
        buff.write(_struct_I.pack(length))
        for val2 in val1.poses:
          _v25 = val2.header
          _x = _v25.seq
          buff.write(_get_struct_I().pack(_x))
          _v26 = _v25.stamp
          _x = _v26
          buff.write(_get_struct_2I().pack(_x.secs, _x.nsecs))
          _x = _v25.frame_id
          length = len(_x)
          if python3 or type(_x) == unicode:
            _x = _x.encode('utf-8')
            length = len(_x)
          buff.write(struct.Struct('<I%ss'%length).pack(length, _x))
          _v27 = val2.pose
          _v28 = _v27.pose
          _v29 = _v28.position
          _x = _v29
          buff.write(_get_struct_3d().pack(_x.x, _x.y, _x.z))
          _v30 = _v28.orientation
          _x = _v30
          buff.write(_get_struct_4d().pack(_x.x, _x.y, _x.z, _x.w))
          buff.write(_v27.covariance.tostring())
        _v31 = val1.start_velocity
        _v32 = _v31.header
        _x = _v32.seq
        buff.write(_get_struct_I().pack(_x))
        _v33 = _v32.stamp
        _x = _v33
        buff.write(_get_struct_2I().pack(_x.secs, _x.nsecs))
        _x = _v32.frame_id
        length = len(_x)
        if python3 or type(_x) == unicode:
          _x = _x.encode('utf-8')
          length = len(_x)
        buff.write(struct.Struct('<I%ss'%length).pack(length, _x))
        _v34 = _v31.twist
        _v35 = _v34.linear
        _x = _v35
        buff.write(_get_struct_3d().pack(_x.x, _x.y, _x.z))
        _v36 = _v34.angular
        _x = _v36
        buff.write(_get_struct_3d().pack(_x.x, _x.y, _x.z))
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
      if self.predicted_agents_poses is None:
        self.predicted_agents_poses = None
      end = 0
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      self.predicted_agents_poses = []
      for i in range(0, length):
        val1 = agent_path_prediction.msg.PredictedPoses()
        start = end
        end += 8
        (val1.id,) = _get_struct_Q().unpack(str[start:end])
        start = end
        end += 4
        (length,) = _struct_I.unpack(str[start:end])
        val1.poses = []
        for i in range(0, length):
          val2 = geometry_msgs.msg.PoseWithCovarianceStamped()
          _v37 = val2.header
          start = end
          end += 4
          (_v37.seq,) = _get_struct_I().unpack(str[start:end])
          _v38 = _v37.stamp
          _x = _v38
          start = end
          end += 8
          (_x.secs, _x.nsecs,) = _get_struct_2I().unpack(str[start:end])
          start = end
          end += 4
          (length,) = _struct_I.unpack(str[start:end])
          start = end
          end += length
          if python3:
            _v37.frame_id = str[start:end].decode('utf-8', 'rosmsg')
          else:
            _v37.frame_id = str[start:end]
          _v39 = val2.pose
          _v40 = _v39.pose
          _v41 = _v40.position
          _x = _v41
          start = end
          end += 24
          (_x.x, _x.y, _x.z,) = _get_struct_3d().unpack(str[start:end])
          _v42 = _v40.orientation
          _x = _v42
          start = end
          end += 32
          (_x.x, _x.y, _x.z, _x.w,) = _get_struct_4d().unpack(str[start:end])
          start = end
          end += 288
          _v39.covariance = numpy.frombuffer(str[start:end], dtype=numpy.float64, count=36)
          val1.poses.append(val2)
        _v43 = val1.start_velocity
        _v44 = _v43.header
        start = end
        end += 4
        (_v44.seq,) = _get_struct_I().unpack(str[start:end])
        _v45 = _v44.stamp
        _x = _v45
        start = end
        end += 8
        (_x.secs, _x.nsecs,) = _get_struct_2I().unpack(str[start:end])
        start = end
        end += 4
        (length,) = _struct_I.unpack(str[start:end])
        start = end
        end += length
        if python3:
          _v44.frame_id = str[start:end].decode('utf-8', 'rosmsg')
        else:
          _v44.frame_id = str[start:end]
        _v46 = _v43.twist
        _v47 = _v46.linear
        _x = _v47
        start = end
        end += 24
        (_x.x, _x.y, _x.z,) = _get_struct_3d().unpack(str[start:end])
        _v48 = _v46.angular
        _x = _v48
        start = end
        end += 24
        (_x.x, _x.y, _x.z,) = _get_struct_3d().unpack(str[start:end])
        self.predicted_agents_poses.append(val1)
      return self
    except struct.error as e:
      raise genpy.DeserializationError(e)  # most likely buffer underfill

_struct_I = genpy.struct_I
def _get_struct_I():
    global _struct_I
    return _struct_I
_struct_2I = None
def _get_struct_2I():
    global _struct_2I
    if _struct_2I is None:
        _struct_2I = struct.Struct("<2I")
    return _struct_2I
_struct_36d = None
def _get_struct_36d():
    global _struct_36d
    if _struct_36d is None:
        _struct_36d = struct.Struct("<36d")
    return _struct_36d
_struct_3d = None
def _get_struct_3d():
    global _struct_3d
    if _struct_3d is None:
        _struct_3d = struct.Struct("<3d")
    return _struct_3d
_struct_4d = None
def _get_struct_4d():
    global _struct_4d
    if _struct_4d is None:
        _struct_4d = struct.Struct("<4d")
    return _struct_4d
_struct_Q = None
def _get_struct_Q():
    global _struct_Q
    if _struct_Q is None:
        _struct_Q = struct.Struct("<Q")
    return _struct_Q
class AgentPosePredict(object):
  _type          = 'agent_path_prediction/AgentPosePredict'
  _md5sum = 'bc118b82a57269022e8ca4cc5d1d18ac'
  _request_class  = AgentPosePredictRequest
  _response_class = AgentPosePredictResponse
