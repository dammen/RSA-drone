# This Python file uses the following encoding: utf-8
"""autogenerated by genpy from ardrone_autonomy/Navdata.msg. Do not edit."""
import sys
python3 = True if sys.hexversion > 0x03000000 else False
import genpy
import struct

import std_msgs.msg

class Navdata(genpy.Message):
  _md5sum = "e1169f766234363125ac62c9a3f87eeb"
  _type = "ardrone_autonomy/Navdata"
  _has_header = True #flag to mark the presence of a Header object
  _full_text = """Header header

# Navdata including the ARDrone 2 specifica sensors
# (magnetometer, barometer)

# 0 means no battery, 100 means full battery
float32 batteryPercent

# 0: Unknown, 1: Init, 2: Landed, 3: Flying, 4: Hovering, 5: Test
# 6: Taking off, 7: Goto Fix Point, 8: Landing, 9: Looping
# Note: 3,7 seems to discriminate type of flying (isFly = 3 | 7)
uint32 state

int32 magX
int32 magY
int32 magZ

# pressure sensor
int32 pressure

# apparently, there was a temperature sensor added as well.
int32 temp

# wind sensing...
float32 wind_speed
float32 wind_angle
float32 wind_comp_angle

# left/right tilt in degrees (rotation about the X axis)
float32 rotX

# forward/backward tilt in degrees (rotation about the Y axis)
float32 rotY

# orientation in degrees (rotation about the Z axis)
float32 rotZ

# estimated altitude (cm)
int32 altd

# linear velocity (mm/sec)
float32 vx

# linear velocity (mm/sec)
float32 vy

# linear velocity (mm/sec)
float32 vz

#linear accelerations (unit: g)
float32 ax
float32 ay
float32 az

#motor commands (unit 0 to 255)
uint8 motor1
uint8 motor2
uint8 motor3
uint8 motor4

#Tags in Vision Detectoion
uint32 tags_count
uint32[] tags_type
uint32[] tags_xc
uint32[] tags_yc
uint32[] tags_width
uint32[] tags_height
float32[] tags_orientation
float32[] tags_distance

#time stamp
float32 tm

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
# 0: no frame
# 1: global frame
string frame_id
"""
  __slots__ = ['header','batteryPercent','state','magX','magY','magZ','pressure','temp','wind_speed','wind_angle','wind_comp_angle','rotX','rotY','rotZ','altd','vx','vy','vz','ax','ay','az','motor1','motor2','motor3','motor4','tags_count','tags_type','tags_xc','tags_yc','tags_width','tags_height','tags_orientation','tags_distance','tm']
  _slot_types = ['std_msgs/Header','float32','uint32','int32','int32','int32','int32','int32','float32','float32','float32','float32','float32','float32','int32','float32','float32','float32','float32','float32','float32','uint8','uint8','uint8','uint8','uint32','uint32[]','uint32[]','uint32[]','uint32[]','uint32[]','float32[]','float32[]','float32']

  def __init__(self, *args, **kwds):
    """
    Constructor. Any message fields that are implicitly/explicitly
    set to None will be assigned a default value. The recommend
    use is keyword arguments as this is more robust to future message
    changes.  You cannot mix in-order arguments and keyword arguments.

    The available fields are:
       header,batteryPercent,state,magX,magY,magZ,pressure,temp,wind_speed,wind_angle,wind_comp_angle,rotX,rotY,rotZ,altd,vx,vy,vz,ax,ay,az,motor1,motor2,motor3,motor4,tags_count,tags_type,tags_xc,tags_yc,tags_width,tags_height,tags_orientation,tags_distance,tm

    :param args: complete set of field values, in .msg order
    :param kwds: use keyword arguments corresponding to message field names
    to set specific fields.
    """
    if args or kwds:
      super(Navdata, self).__init__(*args, **kwds)
      #message fields cannot be None, assign default values for those that are
      if self.header is None:
        self.header = std_msgs.msg.Header()
      if self.batteryPercent is None:
        self.batteryPercent = 0.
      if self.state is None:
        self.state = 0
      if self.magX is None:
        self.magX = 0
      if self.magY is None:
        self.magY = 0
      if self.magZ is None:
        self.magZ = 0
      if self.pressure is None:
        self.pressure = 0
      if self.temp is None:
        self.temp = 0
      if self.wind_speed is None:
        self.wind_speed = 0.
      if self.wind_angle is None:
        self.wind_angle = 0.
      if self.wind_comp_angle is None:
        self.wind_comp_angle = 0.
      if self.rotX is None:
        self.rotX = 0.
      if self.rotY is None:
        self.rotY = 0.
      if self.rotZ is None:
        self.rotZ = 0.
      if self.altd is None:
        self.altd = 0
      if self.vx is None:
        self.vx = 0.
      if self.vy is None:
        self.vy = 0.
      if self.vz is None:
        self.vz = 0.
      if self.ax is None:
        self.ax = 0.
      if self.ay is None:
        self.ay = 0.
      if self.az is None:
        self.az = 0.
      if self.motor1 is None:
        self.motor1 = 0
      if self.motor2 is None:
        self.motor2 = 0
      if self.motor3 is None:
        self.motor3 = 0
      if self.motor4 is None:
        self.motor4 = 0
      if self.tags_count is None:
        self.tags_count = 0
      if self.tags_type is None:
        self.tags_type = []
      if self.tags_xc is None:
        self.tags_xc = []
      if self.tags_yc is None:
        self.tags_yc = []
      if self.tags_width is None:
        self.tags_width = []
      if self.tags_height is None:
        self.tags_height = []
      if self.tags_orientation is None:
        self.tags_orientation = []
      if self.tags_distance is None:
        self.tags_distance = []
      if self.tm is None:
        self.tm = 0.
    else:
      self.header = std_msgs.msg.Header()
      self.batteryPercent = 0.
      self.state = 0
      self.magX = 0
      self.magY = 0
      self.magZ = 0
      self.pressure = 0
      self.temp = 0
      self.wind_speed = 0.
      self.wind_angle = 0.
      self.wind_comp_angle = 0.
      self.rotX = 0.
      self.rotY = 0.
      self.rotZ = 0.
      self.altd = 0
      self.vx = 0.
      self.vy = 0.
      self.vz = 0.
      self.ax = 0.
      self.ay = 0.
      self.az = 0.
      self.motor1 = 0
      self.motor2 = 0
      self.motor3 = 0
      self.motor4 = 0
      self.tags_count = 0
      self.tags_type = []
      self.tags_xc = []
      self.tags_yc = []
      self.tags_width = []
      self.tags_height = []
      self.tags_orientation = []
      self.tags_distance = []
      self.tm = 0.

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
      buff.write(_struct_3I.pack(_x.header.seq, _x.header.stamp.secs, _x.header.stamp.nsecs))
      _x = self.header.frame_id
      length = len(_x)
      if python3 or type(_x) == unicode:
        _x = _x.encode('utf-8')
        length = len(_x)
      if python3:
        buff.write(struct.pack('<I%sB'%length, length, *_x))
      else:
        buff.write(struct.pack('<I%ss'%length, length, _x))
      _x = self
      buff.write(_struct_fI5i6fi6f4BI.pack(_x.batteryPercent, _x.state, _x.magX, _x.magY, _x.magZ, _x.pressure, _x.temp, _x.wind_speed, _x.wind_angle, _x.wind_comp_angle, _x.rotX, _x.rotY, _x.rotZ, _x.altd, _x.vx, _x.vy, _x.vz, _x.ax, _x.ay, _x.az, _x.motor1, _x.motor2, _x.motor3, _x.motor4, _x.tags_count))
      length = len(self.tags_type)
      buff.write(_struct_I.pack(length))
      pattern = '<%sI'%length
      buff.write(struct.pack(pattern, *self.tags_type))
      length = len(self.tags_xc)
      buff.write(_struct_I.pack(length))
      pattern = '<%sI'%length
      buff.write(struct.pack(pattern, *self.tags_xc))
      length = len(self.tags_yc)
      buff.write(_struct_I.pack(length))
      pattern = '<%sI'%length
      buff.write(struct.pack(pattern, *self.tags_yc))
      length = len(self.tags_width)
      buff.write(_struct_I.pack(length))
      pattern = '<%sI'%length
      buff.write(struct.pack(pattern, *self.tags_width))
      length = len(self.tags_height)
      buff.write(_struct_I.pack(length))
      pattern = '<%sI'%length
      buff.write(struct.pack(pattern, *self.tags_height))
      length = len(self.tags_orientation)
      buff.write(_struct_I.pack(length))
      pattern = '<%sf'%length
      buff.write(struct.pack(pattern, *self.tags_orientation))
      length = len(self.tags_distance)
      buff.write(_struct_I.pack(length))
      pattern = '<%sf'%length
      buff.write(struct.pack(pattern, *self.tags_distance))
      buff.write(_struct_f.pack(self.tm))
    except struct.error as se: self._check_types(struct.error("%s: '%s' when writing '%s'" % (type(se), str(se), str(locals().get('_x', self)))))
    except TypeError as te: self._check_types(ValueError("%s: '%s' when writing '%s'" % (type(te), str(te), str(locals().get('_x', self)))))

  def deserialize(self, str):
    """
    unpack serialized message in str into this message instance
    :param str: byte array of serialized message, ``str``
    """
    try:
      if self.header is None:
        self.header = std_msgs.msg.Header()
      end = 0
      _x = self
      start = end
      end += 12
      (_x.header.seq, _x.header.stamp.secs, _x.header.stamp.nsecs,) = _struct_3I.unpack(str[start:end])
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      start = end
      end += length
      if python3:
        self.header.frame_id = str[start:end].decode('utf-8')
      else:
        self.header.frame_id = str[start:end]
      _x = self
      start = end
      end += 88
      (_x.batteryPercent, _x.state, _x.magX, _x.magY, _x.magZ, _x.pressure, _x.temp, _x.wind_speed, _x.wind_angle, _x.wind_comp_angle, _x.rotX, _x.rotY, _x.rotZ, _x.altd, _x.vx, _x.vy, _x.vz, _x.ax, _x.ay, _x.az, _x.motor1, _x.motor2, _x.motor3, _x.motor4, _x.tags_count,) = _struct_fI5i6fi6f4BI.unpack(str[start:end])
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      pattern = '<%sI'%length
      start = end
      end += struct.calcsize(pattern)
      self.tags_type = struct.unpack(pattern, str[start:end])
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      pattern = '<%sI'%length
      start = end
      end += struct.calcsize(pattern)
      self.tags_xc = struct.unpack(pattern, str[start:end])
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      pattern = '<%sI'%length
      start = end
      end += struct.calcsize(pattern)
      self.tags_yc = struct.unpack(pattern, str[start:end])
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      pattern = '<%sI'%length
      start = end
      end += struct.calcsize(pattern)
      self.tags_width = struct.unpack(pattern, str[start:end])
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      pattern = '<%sI'%length
      start = end
      end += struct.calcsize(pattern)
      self.tags_height = struct.unpack(pattern, str[start:end])
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      pattern = '<%sf'%length
      start = end
      end += struct.calcsize(pattern)
      self.tags_orientation = struct.unpack(pattern, str[start:end])
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      pattern = '<%sf'%length
      start = end
      end += struct.calcsize(pattern)
      self.tags_distance = struct.unpack(pattern, str[start:end])
      start = end
      end += 4
      (self.tm,) = _struct_f.unpack(str[start:end])
      return self
    except struct.error as e:
      raise genpy.DeserializationError(e) #most likely buffer underfill


  def serialize_numpy(self, buff, numpy):
    """
    serialize message with numpy array types into buffer
    :param buff: buffer, ``StringIO``
    :param numpy: numpy python module
    """
    try:
      _x = self
      buff.write(_struct_3I.pack(_x.header.seq, _x.header.stamp.secs, _x.header.stamp.nsecs))
      _x = self.header.frame_id
      length = len(_x)
      if python3 or type(_x) == unicode:
        _x = _x.encode('utf-8')
        length = len(_x)
      if python3:
        buff.write(struct.pack('<I%sB'%length, length, *_x))
      else:
        buff.write(struct.pack('<I%ss'%length, length, _x))
      _x = self
      buff.write(_struct_fI5i6fi6f4BI.pack(_x.batteryPercent, _x.state, _x.magX, _x.magY, _x.magZ, _x.pressure, _x.temp, _x.wind_speed, _x.wind_angle, _x.wind_comp_angle, _x.rotX, _x.rotY, _x.rotZ, _x.altd, _x.vx, _x.vy, _x.vz, _x.ax, _x.ay, _x.az, _x.motor1, _x.motor2, _x.motor3, _x.motor4, _x.tags_count))
      length = len(self.tags_type)
      buff.write(_struct_I.pack(length))
      pattern = '<%sI'%length
      buff.write(self.tags_type.tostring())
      length = len(self.tags_xc)
      buff.write(_struct_I.pack(length))
      pattern = '<%sI'%length
      buff.write(self.tags_xc.tostring())
      length = len(self.tags_yc)
      buff.write(_struct_I.pack(length))
      pattern = '<%sI'%length
      buff.write(self.tags_yc.tostring())
      length = len(self.tags_width)
      buff.write(_struct_I.pack(length))
      pattern = '<%sI'%length
      buff.write(self.tags_width.tostring())
      length = len(self.tags_height)
      buff.write(_struct_I.pack(length))
      pattern = '<%sI'%length
      buff.write(self.tags_height.tostring())
      length = len(self.tags_orientation)
      buff.write(_struct_I.pack(length))
      pattern = '<%sf'%length
      buff.write(self.tags_orientation.tostring())
      length = len(self.tags_distance)
      buff.write(_struct_I.pack(length))
      pattern = '<%sf'%length
      buff.write(self.tags_distance.tostring())
      buff.write(_struct_f.pack(self.tm))
    except struct.error as se: self._check_types(struct.error("%s: '%s' when writing '%s'" % (type(se), str(se), str(locals().get('_x', self)))))
    except TypeError as te: self._check_types(ValueError("%s: '%s' when writing '%s'" % (type(te), str(te), str(locals().get('_x', self)))))

  def deserialize_numpy(self, str, numpy):
    """
    unpack serialized message in str into this message instance using numpy for array types
    :param str: byte array of serialized message, ``str``
    :param numpy: numpy python module
    """
    try:
      if self.header is None:
        self.header = std_msgs.msg.Header()
      end = 0
      _x = self
      start = end
      end += 12
      (_x.header.seq, _x.header.stamp.secs, _x.header.stamp.nsecs,) = _struct_3I.unpack(str[start:end])
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      start = end
      end += length
      if python3:
        self.header.frame_id = str[start:end].decode('utf-8')
      else:
        self.header.frame_id = str[start:end]
      _x = self
      start = end
      end += 88
      (_x.batteryPercent, _x.state, _x.magX, _x.magY, _x.magZ, _x.pressure, _x.temp, _x.wind_speed, _x.wind_angle, _x.wind_comp_angle, _x.rotX, _x.rotY, _x.rotZ, _x.altd, _x.vx, _x.vy, _x.vz, _x.ax, _x.ay, _x.az, _x.motor1, _x.motor2, _x.motor3, _x.motor4, _x.tags_count,) = _struct_fI5i6fi6f4BI.unpack(str[start:end])
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      pattern = '<%sI'%length
      start = end
      end += struct.calcsize(pattern)
      self.tags_type = numpy.frombuffer(str[start:end], dtype=numpy.uint32, count=length)
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      pattern = '<%sI'%length
      start = end
      end += struct.calcsize(pattern)
      self.tags_xc = numpy.frombuffer(str[start:end], dtype=numpy.uint32, count=length)
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      pattern = '<%sI'%length
      start = end
      end += struct.calcsize(pattern)
      self.tags_yc = numpy.frombuffer(str[start:end], dtype=numpy.uint32, count=length)
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      pattern = '<%sI'%length
      start = end
      end += struct.calcsize(pattern)
      self.tags_width = numpy.frombuffer(str[start:end], dtype=numpy.uint32, count=length)
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      pattern = '<%sI'%length
      start = end
      end += struct.calcsize(pattern)
      self.tags_height = numpy.frombuffer(str[start:end], dtype=numpy.uint32, count=length)
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      pattern = '<%sf'%length
      start = end
      end += struct.calcsize(pattern)
      self.tags_orientation = numpy.frombuffer(str[start:end], dtype=numpy.float32, count=length)
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      pattern = '<%sf'%length
      start = end
      end += struct.calcsize(pattern)
      self.tags_distance = numpy.frombuffer(str[start:end], dtype=numpy.float32, count=length)
      start = end
      end += 4
      (self.tm,) = _struct_f.unpack(str[start:end])
      return self
    except struct.error as e:
      raise genpy.DeserializationError(e) #most likely buffer underfill

_struct_I = genpy.struct_I
_struct_fI5i6fi6f4BI = struct.Struct("<fI5i6fi6f4BI")
_struct_3I = struct.Struct("<3I")
_struct_f = struct.Struct("<f")