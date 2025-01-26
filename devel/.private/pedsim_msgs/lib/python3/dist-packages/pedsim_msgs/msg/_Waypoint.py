# This Python file uses the following encoding: utf-8
"""autogenerated by genpy from pedsim_msgs/Waypoint.msg. Do not edit."""
import codecs
import sys
python3 = True if sys.hexversion > 0x03000000 else False
import genpy
import struct

import geometry_msgs.msg

class Waypoint(genpy.Message):
  _md5sum = "0711f152cb4a80e128296762f14a6fbe"
  _type = "pedsim_msgs/Waypoint"
  _has_header = False  # flag to mark the presence of a Header object
  _full_text = """int8 BHV_SIMPLE = 0
int8 BHV_SOURCE = 1
int8 BHV_SINK = 2

string name
int8 type
int8 behavior
geometry_msgs/Point position
float32 radius
float32 interaction_radius

================================================================================
MSG: geometry_msgs/Point
# This contains the position of a point in free space
float64 x
float64 y
float64 z
"""
  # Pseudo-constants
  BHV_SIMPLE = 0
  BHV_SOURCE = 1
  BHV_SINK = 2

  __slots__ = ['name','type','behavior','position','radius','interaction_radius']
  _slot_types = ['string','int8','int8','geometry_msgs/Point','float32','float32']

  def __init__(self, *args, **kwds):
    """
    Constructor. Any message fields that are implicitly/explicitly
    set to None will be assigned a default value. The recommend
    use is keyword arguments as this is more robust to future message
    changes.  You cannot mix in-order arguments and keyword arguments.

    The available fields are:
       name,type,behavior,position,radius,interaction_radius

    :param args: complete set of field values, in .msg order
    :param kwds: use keyword arguments corresponding to message field names
    to set specific fields.
    """
    if args or kwds:
      super(Waypoint, self).__init__(*args, **kwds)
      # message fields cannot be None, assign default values for those that are
      if self.name is None:
        self.name = ''
      if self.type is None:
        self.type = 0
      if self.behavior is None:
        self.behavior = 0
      if self.position is None:
        self.position = geometry_msgs.msg.Point()
      if self.radius is None:
        self.radius = 0.
      if self.interaction_radius is None:
        self.interaction_radius = 0.
    else:
      self.name = ''
      self.type = 0
      self.behavior = 0
      self.position = geometry_msgs.msg.Point()
      self.radius = 0.
      self.interaction_radius = 0.

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
      _x = self.name
      length = len(_x)
      if python3 or type(_x) == unicode:
        _x = _x.encode('utf-8')
        length = len(_x)
      buff.write(struct.Struct('<I%ss'%length).pack(length, _x))
      _x = self
      buff.write(_get_struct_2b3d2f().pack(_x.type, _x.behavior, _x.position.x, _x.position.y, _x.position.z, _x.radius, _x.interaction_radius))
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
      if self.position is None:
        self.position = geometry_msgs.msg.Point()
      end = 0
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      start = end
      end += length
      if python3:
        self.name = str[start:end].decode('utf-8', 'rosmsg')
      else:
        self.name = str[start:end]
      _x = self
      start = end
      end += 34
      (_x.type, _x.behavior, _x.position.x, _x.position.y, _x.position.z, _x.radius, _x.interaction_radius,) = _get_struct_2b3d2f().unpack(str[start:end])
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
      _x = self.name
      length = len(_x)
      if python3 or type(_x) == unicode:
        _x = _x.encode('utf-8')
        length = len(_x)
      buff.write(struct.Struct('<I%ss'%length).pack(length, _x))
      _x = self
      buff.write(_get_struct_2b3d2f().pack(_x.type, _x.behavior, _x.position.x, _x.position.y, _x.position.z, _x.radius, _x.interaction_radius))
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
      if self.position is None:
        self.position = geometry_msgs.msg.Point()
      end = 0
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      start = end
      end += length
      if python3:
        self.name = str[start:end].decode('utf-8', 'rosmsg')
      else:
        self.name = str[start:end]
      _x = self
      start = end
      end += 34
      (_x.type, _x.behavior, _x.position.x, _x.position.y, _x.position.z, _x.radius, _x.interaction_radius,) = _get_struct_2b3d2f().unpack(str[start:end])
      return self
    except struct.error as e:
      raise genpy.DeserializationError(e)  # most likely buffer underfill

_struct_I = genpy.struct_I
def _get_struct_I():
    global _struct_I
    return _struct_I
_struct_2b3d2f = None
def _get_struct_2b3d2f():
    global _struct_2b3d2f
    if _struct_2b3d2f is None:
        _struct_2b3d2f = struct.Struct("<2b3d2f")
    return _struct_2b3d2f
