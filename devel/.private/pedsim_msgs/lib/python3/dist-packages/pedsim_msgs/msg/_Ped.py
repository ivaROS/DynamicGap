# This Python file uses the following encoding: utf-8
"""autogenerated by genpy from pedsim_msgs/Ped.msg. Do not edit."""
import codecs
import sys
python3 = True if sys.hexversion > 0x03000000 else False
import genpy
import struct

import geometry_msgs.msg

class Ped(genpy.Message):
  _md5sum = "d8575a29ff9a6fbcc93400a28e9925d4"
  _type = "pedsim_msgs/Ped"
  _has_header = False  # flag to mark the presence of a Header object
  _full_text = """# Added by Ronja Gueldenring
# For spawning agents dynamically in pedsim and forwarding them to flatland
string id
geometry_msgs/Point pos
string type  # "adult", "child", "elder", "vehicle", "servicerobot"
string yaml_file
int16 number_of_peds
float64 vmax

string start_up_mode  # "default", "wait_timer", "trigger_zone"
float64 wait_time
float64 trigger_zone_radius

float64 max_talking_distance
float64 max_servicing_radius

float64 chatting_probability
float64 tell_story_probability
float64 group_talking_probability
float64 talking_and_walking_probability
float64 requesting_service_probability
float64 requesting_guide_probability
float64 requesting_follower_probability

float64 talking_base_time
float64 tell_story_base_time
float64 group_talking_base_time
float64 talking_and_walking_base_time
float64 receiving_service_base_time
float64 requesting_service_base_time

# forces
float64 force_factor_desired
float64 force_factor_obstacle
float64 force_factor_social
float64 force_factor_robot

geometry_msgs/Point[] waypoints
int16 waypoint_mode

string configuration
================================================================================
MSG: geometry_msgs/Point
# This contains the position of a point in free space
float64 x
float64 y
float64 z
"""
  __slots__ = ['id','pos','type','yaml_file','number_of_peds','vmax','start_up_mode','wait_time','trigger_zone_radius','max_talking_distance','max_servicing_radius','chatting_probability','tell_story_probability','group_talking_probability','talking_and_walking_probability','requesting_service_probability','requesting_guide_probability','requesting_follower_probability','talking_base_time','tell_story_base_time','group_talking_base_time','talking_and_walking_base_time','receiving_service_base_time','requesting_service_base_time','force_factor_desired','force_factor_obstacle','force_factor_social','force_factor_robot','waypoints','waypoint_mode','configuration']
  _slot_types = ['string','geometry_msgs/Point','string','string','int16','float64','string','float64','float64','float64','float64','float64','float64','float64','float64','float64','float64','float64','float64','float64','float64','float64','float64','float64','float64','float64','float64','float64','geometry_msgs/Point[]','int16','string']

  def __init__(self, *args, **kwds):
    """
    Constructor. Any message fields that are implicitly/explicitly
    set to None will be assigned a default value. The recommend
    use is keyword arguments as this is more robust to future message
    changes.  You cannot mix in-order arguments and keyword arguments.

    The available fields are:
       id,pos,type,yaml_file,number_of_peds,vmax,start_up_mode,wait_time,trigger_zone_radius,max_talking_distance,max_servicing_radius,chatting_probability,tell_story_probability,group_talking_probability,talking_and_walking_probability,requesting_service_probability,requesting_guide_probability,requesting_follower_probability,talking_base_time,tell_story_base_time,group_talking_base_time,talking_and_walking_base_time,receiving_service_base_time,requesting_service_base_time,force_factor_desired,force_factor_obstacle,force_factor_social,force_factor_robot,waypoints,waypoint_mode,configuration

    :param args: complete set of field values, in .msg order
    :param kwds: use keyword arguments corresponding to message field names
    to set specific fields.
    """
    if args or kwds:
      super(Ped, self).__init__(*args, **kwds)
      # message fields cannot be None, assign default values for those that are
      if self.id is None:
        self.id = ''
      if self.pos is None:
        self.pos = geometry_msgs.msg.Point()
      if self.type is None:
        self.type = ''
      if self.yaml_file is None:
        self.yaml_file = ''
      if self.number_of_peds is None:
        self.number_of_peds = 0
      if self.vmax is None:
        self.vmax = 0.
      if self.start_up_mode is None:
        self.start_up_mode = ''
      if self.wait_time is None:
        self.wait_time = 0.
      if self.trigger_zone_radius is None:
        self.trigger_zone_radius = 0.
      if self.max_talking_distance is None:
        self.max_talking_distance = 0.
      if self.max_servicing_radius is None:
        self.max_servicing_radius = 0.
      if self.chatting_probability is None:
        self.chatting_probability = 0.
      if self.tell_story_probability is None:
        self.tell_story_probability = 0.
      if self.group_talking_probability is None:
        self.group_talking_probability = 0.
      if self.talking_and_walking_probability is None:
        self.talking_and_walking_probability = 0.
      if self.requesting_service_probability is None:
        self.requesting_service_probability = 0.
      if self.requesting_guide_probability is None:
        self.requesting_guide_probability = 0.
      if self.requesting_follower_probability is None:
        self.requesting_follower_probability = 0.
      if self.talking_base_time is None:
        self.talking_base_time = 0.
      if self.tell_story_base_time is None:
        self.tell_story_base_time = 0.
      if self.group_talking_base_time is None:
        self.group_talking_base_time = 0.
      if self.talking_and_walking_base_time is None:
        self.talking_and_walking_base_time = 0.
      if self.receiving_service_base_time is None:
        self.receiving_service_base_time = 0.
      if self.requesting_service_base_time is None:
        self.requesting_service_base_time = 0.
      if self.force_factor_desired is None:
        self.force_factor_desired = 0.
      if self.force_factor_obstacle is None:
        self.force_factor_obstacle = 0.
      if self.force_factor_social is None:
        self.force_factor_social = 0.
      if self.force_factor_robot is None:
        self.force_factor_robot = 0.
      if self.waypoints is None:
        self.waypoints = []
      if self.waypoint_mode is None:
        self.waypoint_mode = 0
      if self.configuration is None:
        self.configuration = ''
    else:
      self.id = ''
      self.pos = geometry_msgs.msg.Point()
      self.type = ''
      self.yaml_file = ''
      self.number_of_peds = 0
      self.vmax = 0.
      self.start_up_mode = ''
      self.wait_time = 0.
      self.trigger_zone_radius = 0.
      self.max_talking_distance = 0.
      self.max_servicing_radius = 0.
      self.chatting_probability = 0.
      self.tell_story_probability = 0.
      self.group_talking_probability = 0.
      self.talking_and_walking_probability = 0.
      self.requesting_service_probability = 0.
      self.requesting_guide_probability = 0.
      self.requesting_follower_probability = 0.
      self.talking_base_time = 0.
      self.tell_story_base_time = 0.
      self.group_talking_base_time = 0.
      self.talking_and_walking_base_time = 0.
      self.receiving_service_base_time = 0.
      self.requesting_service_base_time = 0.
      self.force_factor_desired = 0.
      self.force_factor_obstacle = 0.
      self.force_factor_social = 0.
      self.force_factor_robot = 0.
      self.waypoints = []
      self.waypoint_mode = 0
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
      _x = self.id
      length = len(_x)
      if python3 or type(_x) == unicode:
        _x = _x.encode('utf-8')
        length = len(_x)
      buff.write(struct.Struct('<I%ss'%length).pack(length, _x))
      _x = self
      buff.write(_get_struct_3d().pack(_x.pos.x, _x.pos.y, _x.pos.z))
      _x = self.type
      length = len(_x)
      if python3 or type(_x) == unicode:
        _x = _x.encode('utf-8')
        length = len(_x)
      buff.write(struct.Struct('<I%ss'%length).pack(length, _x))
      _x = self.yaml_file
      length = len(_x)
      if python3 or type(_x) == unicode:
        _x = _x.encode('utf-8')
        length = len(_x)
      buff.write(struct.Struct('<I%ss'%length).pack(length, _x))
      _x = self
      buff.write(_get_struct_hd().pack(_x.number_of_peds, _x.vmax))
      _x = self.start_up_mode
      length = len(_x)
      if python3 or type(_x) == unicode:
        _x = _x.encode('utf-8')
        length = len(_x)
      buff.write(struct.Struct('<I%ss'%length).pack(length, _x))
      _x = self
      buff.write(_get_struct_21d().pack(_x.wait_time, _x.trigger_zone_radius, _x.max_talking_distance, _x.max_servicing_radius, _x.chatting_probability, _x.tell_story_probability, _x.group_talking_probability, _x.talking_and_walking_probability, _x.requesting_service_probability, _x.requesting_guide_probability, _x.requesting_follower_probability, _x.talking_base_time, _x.tell_story_base_time, _x.group_talking_base_time, _x.talking_and_walking_base_time, _x.receiving_service_base_time, _x.requesting_service_base_time, _x.force_factor_desired, _x.force_factor_obstacle, _x.force_factor_social, _x.force_factor_robot))
      length = len(self.waypoints)
      buff.write(_struct_I.pack(length))
      for val1 in self.waypoints:
        _x = val1
        buff.write(_get_struct_3d().pack(_x.x, _x.y, _x.z))
      _x = self.waypoint_mode
      buff.write(_get_struct_h().pack(_x))
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
      if self.pos is None:
        self.pos = geometry_msgs.msg.Point()
      if self.waypoints is None:
        self.waypoints = None
      end = 0
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      start = end
      end += length
      if python3:
        self.id = str[start:end].decode('utf-8', 'rosmsg')
      else:
        self.id = str[start:end]
      _x = self
      start = end
      end += 24
      (_x.pos.x, _x.pos.y, _x.pos.z,) = _get_struct_3d().unpack(str[start:end])
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
        self.yaml_file = str[start:end].decode('utf-8', 'rosmsg')
      else:
        self.yaml_file = str[start:end]
      _x = self
      start = end
      end += 10
      (_x.number_of_peds, _x.vmax,) = _get_struct_hd().unpack(str[start:end])
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      start = end
      end += length
      if python3:
        self.start_up_mode = str[start:end].decode('utf-8', 'rosmsg')
      else:
        self.start_up_mode = str[start:end]
      _x = self
      start = end
      end += 168
      (_x.wait_time, _x.trigger_zone_radius, _x.max_talking_distance, _x.max_servicing_radius, _x.chatting_probability, _x.tell_story_probability, _x.group_talking_probability, _x.talking_and_walking_probability, _x.requesting_service_probability, _x.requesting_guide_probability, _x.requesting_follower_probability, _x.talking_base_time, _x.tell_story_base_time, _x.group_talking_base_time, _x.talking_and_walking_base_time, _x.receiving_service_base_time, _x.requesting_service_base_time, _x.force_factor_desired, _x.force_factor_obstacle, _x.force_factor_social, _x.force_factor_robot,) = _get_struct_21d().unpack(str[start:end])
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      self.waypoints = []
      for i in range(0, length):
        val1 = geometry_msgs.msg.Point()
        _x = val1
        start = end
        end += 24
        (_x.x, _x.y, _x.z,) = _get_struct_3d().unpack(str[start:end])
        self.waypoints.append(val1)
      start = end
      end += 2
      (self.waypoint_mode,) = _get_struct_h().unpack(str[start:end])
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
      _x = self.id
      length = len(_x)
      if python3 or type(_x) == unicode:
        _x = _x.encode('utf-8')
        length = len(_x)
      buff.write(struct.Struct('<I%ss'%length).pack(length, _x))
      _x = self
      buff.write(_get_struct_3d().pack(_x.pos.x, _x.pos.y, _x.pos.z))
      _x = self.type
      length = len(_x)
      if python3 or type(_x) == unicode:
        _x = _x.encode('utf-8')
        length = len(_x)
      buff.write(struct.Struct('<I%ss'%length).pack(length, _x))
      _x = self.yaml_file
      length = len(_x)
      if python3 or type(_x) == unicode:
        _x = _x.encode('utf-8')
        length = len(_x)
      buff.write(struct.Struct('<I%ss'%length).pack(length, _x))
      _x = self
      buff.write(_get_struct_hd().pack(_x.number_of_peds, _x.vmax))
      _x = self.start_up_mode
      length = len(_x)
      if python3 or type(_x) == unicode:
        _x = _x.encode('utf-8')
        length = len(_x)
      buff.write(struct.Struct('<I%ss'%length).pack(length, _x))
      _x = self
      buff.write(_get_struct_21d().pack(_x.wait_time, _x.trigger_zone_radius, _x.max_talking_distance, _x.max_servicing_radius, _x.chatting_probability, _x.tell_story_probability, _x.group_talking_probability, _x.talking_and_walking_probability, _x.requesting_service_probability, _x.requesting_guide_probability, _x.requesting_follower_probability, _x.talking_base_time, _x.tell_story_base_time, _x.group_talking_base_time, _x.talking_and_walking_base_time, _x.receiving_service_base_time, _x.requesting_service_base_time, _x.force_factor_desired, _x.force_factor_obstacle, _x.force_factor_social, _x.force_factor_robot))
      length = len(self.waypoints)
      buff.write(_struct_I.pack(length))
      for val1 in self.waypoints:
        _x = val1
        buff.write(_get_struct_3d().pack(_x.x, _x.y, _x.z))
      _x = self.waypoint_mode
      buff.write(_get_struct_h().pack(_x))
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
      if self.pos is None:
        self.pos = geometry_msgs.msg.Point()
      if self.waypoints is None:
        self.waypoints = None
      end = 0
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      start = end
      end += length
      if python3:
        self.id = str[start:end].decode('utf-8', 'rosmsg')
      else:
        self.id = str[start:end]
      _x = self
      start = end
      end += 24
      (_x.pos.x, _x.pos.y, _x.pos.z,) = _get_struct_3d().unpack(str[start:end])
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
        self.yaml_file = str[start:end].decode('utf-8', 'rosmsg')
      else:
        self.yaml_file = str[start:end]
      _x = self
      start = end
      end += 10
      (_x.number_of_peds, _x.vmax,) = _get_struct_hd().unpack(str[start:end])
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      start = end
      end += length
      if python3:
        self.start_up_mode = str[start:end].decode('utf-8', 'rosmsg')
      else:
        self.start_up_mode = str[start:end]
      _x = self
      start = end
      end += 168
      (_x.wait_time, _x.trigger_zone_radius, _x.max_talking_distance, _x.max_servicing_radius, _x.chatting_probability, _x.tell_story_probability, _x.group_talking_probability, _x.talking_and_walking_probability, _x.requesting_service_probability, _x.requesting_guide_probability, _x.requesting_follower_probability, _x.talking_base_time, _x.tell_story_base_time, _x.group_talking_base_time, _x.talking_and_walking_base_time, _x.receiving_service_base_time, _x.requesting_service_base_time, _x.force_factor_desired, _x.force_factor_obstacle, _x.force_factor_social, _x.force_factor_robot,) = _get_struct_21d().unpack(str[start:end])
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      self.waypoints = []
      for i in range(0, length):
        val1 = geometry_msgs.msg.Point()
        _x = val1
        start = end
        end += 24
        (_x.x, _x.y, _x.z,) = _get_struct_3d().unpack(str[start:end])
        self.waypoints.append(val1)
      start = end
      end += 2
      (self.waypoint_mode,) = _get_struct_h().unpack(str[start:end])
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
_struct_21d = None
def _get_struct_21d():
    global _struct_21d
    if _struct_21d is None:
        _struct_21d = struct.Struct("<21d")
    return _struct_21d
_struct_3d = None
def _get_struct_3d():
    global _struct_3d
    if _struct_3d is None:
        _struct_3d = struct.Struct("<3d")
    return _struct_3d
_struct_h = None
def _get_struct_h():
    global _struct_h
    if _struct_h is None:
        _struct_h = struct.Struct("<h")
    return _struct_h
_struct_hd = None
def _get_struct_hd():
    global _struct_hd
    if _struct_hd is None:
        _struct_hd = struct.Struct("<hd")
    return _struct_hd
