// Generated by gencpp from file ardrone_autonomy/navdata_vision.msg
// DO NOT EDIT!


#ifndef ARDRONE_AUTONOMY_MESSAGE_NAVDATA_VISION_H
#define ARDRONE_AUTONOMY_MESSAGE_NAVDATA_VISION_H


#include <string>
#include <vector>
#include <map>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>

#include <std_msgs/Header.h>
#include <ardrone_autonomy/vector31.h>

namespace ardrone_autonomy
{
template <class ContainerAllocator>
struct navdata_vision_
{
  typedef navdata_vision_<ContainerAllocator> Type;

  navdata_vision_()
    : header()
    , drone_time(0.0)
    , tag(0)
    , size(0)
    , vision_state(0)
    , vision_misc(0)
    , vision_phi_trim(0.0)
    , vision_phi_ref_prop(0.0)
    , vision_theta_trim(0.0)
    , vision_theta_ref_prop(0.0)
    , new_raw_picture(0)
    , theta_capture(0.0)
    , phi_capture(0.0)
    , psi_capture(0.0)
    , altitude_capture(0)
    , time_capture(0)
    , body_v()
    , delta_phi(0.0)
    , delta_theta(0.0)
    , delta_psi(0.0)
    , gold_defined(0)
    , gold_reset(0)
    , gold_x(0.0)
    , gold_y(0.0)  {
    }
  navdata_vision_(const ContainerAllocator& _alloc)
    : header(_alloc)
    , drone_time(0.0)
    , tag(0)
    , size(0)
    , vision_state(0)
    , vision_misc(0)
    , vision_phi_trim(0.0)
    , vision_phi_ref_prop(0.0)
    , vision_theta_trim(0.0)
    , vision_theta_ref_prop(0.0)
    , new_raw_picture(0)
    , theta_capture(0.0)
    , phi_capture(0.0)
    , psi_capture(0.0)
    , altitude_capture(0)
    , time_capture(0)
    , body_v(_alloc)
    , delta_phi(0.0)
    , delta_theta(0.0)
    , delta_psi(0.0)
    , gold_defined(0)
    , gold_reset(0)
    , gold_x(0.0)
    , gold_y(0.0)  {
  (void)_alloc;
    }



   typedef  ::std_msgs::Header_<ContainerAllocator>  _header_type;
  _header_type header;

   typedef double _drone_time_type;
  _drone_time_type drone_time;

   typedef uint16_t _tag_type;
  _tag_type tag;

   typedef uint16_t _size_type;
  _size_type size;

   typedef uint32_t _vision_state_type;
  _vision_state_type vision_state;

   typedef int32_t _vision_misc_type;
  _vision_misc_type vision_misc;

   typedef float _vision_phi_trim_type;
  _vision_phi_trim_type vision_phi_trim;

   typedef float _vision_phi_ref_prop_type;
  _vision_phi_ref_prop_type vision_phi_ref_prop;

   typedef float _vision_theta_trim_type;
  _vision_theta_trim_type vision_theta_trim;

   typedef float _vision_theta_ref_prop_type;
  _vision_theta_ref_prop_type vision_theta_ref_prop;

   typedef int32_t _new_raw_picture_type;
  _new_raw_picture_type new_raw_picture;

   typedef float _theta_capture_type;
  _theta_capture_type theta_capture;

   typedef float _phi_capture_type;
  _phi_capture_type phi_capture;

   typedef float _psi_capture_type;
  _psi_capture_type psi_capture;

   typedef int32_t _altitude_capture_type;
  _altitude_capture_type altitude_capture;

   typedef uint32_t _time_capture_type;
  _time_capture_type time_capture;

   typedef  ::ardrone_autonomy::vector31_<ContainerAllocator>  _body_v_type;
  _body_v_type body_v;

   typedef float _delta_phi_type;
  _delta_phi_type delta_phi;

   typedef float _delta_theta_type;
  _delta_theta_type delta_theta;

   typedef float _delta_psi_type;
  _delta_psi_type delta_psi;

   typedef uint32_t _gold_defined_type;
  _gold_defined_type gold_defined;

   typedef uint32_t _gold_reset_type;
  _gold_reset_type gold_reset;

   typedef float _gold_x_type;
  _gold_x_type gold_x;

   typedef float _gold_y_type;
  _gold_y_type gold_y;




  typedef boost::shared_ptr< ::ardrone_autonomy::navdata_vision_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::ardrone_autonomy::navdata_vision_<ContainerAllocator> const> ConstPtr;

}; // struct navdata_vision_

typedef ::ardrone_autonomy::navdata_vision_<std::allocator<void> > navdata_vision;

typedef boost::shared_ptr< ::ardrone_autonomy::navdata_vision > navdata_visionPtr;
typedef boost::shared_ptr< ::ardrone_autonomy::navdata_vision const> navdata_visionConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::ardrone_autonomy::navdata_vision_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::ardrone_autonomy::navdata_vision_<ContainerAllocator> >::stream(s, "", v);
return s;
}

} // namespace ardrone_autonomy

namespace ros
{
namespace message_traits
{



// BOOLTRAITS {'IsFixedSize': False, 'IsMessage': True, 'HasHeader': True}
// {'std_msgs': ['/opt/ros/indigo/share/std_msgs/cmake/../msg'], 'ardrone_autonomy': ['/home/rsa/rsa_drone/src/ardrone_autonomy/msg'], 'geometry_msgs': ['/opt/ros/indigo/share/geometry_msgs/cmake/../msg']}

// !!!!!!!!!!! ['__class__', '__delattr__', '__dict__', '__doc__', '__eq__', '__format__', '__getattribute__', '__hash__', '__init__', '__module__', '__ne__', '__new__', '__reduce__', '__reduce_ex__', '__repr__', '__setattr__', '__sizeof__', '__str__', '__subclasshook__', '__weakref__', '_parsed_fields', 'constants', 'fields', 'full_name', 'has_header', 'header_present', 'names', 'package', 'parsed_fields', 'short_name', 'text', 'types']




template <class ContainerAllocator>
struct IsFixedSize< ::ardrone_autonomy::navdata_vision_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::ardrone_autonomy::navdata_vision_<ContainerAllocator> const>
  : FalseType
  { };

template <class ContainerAllocator>
struct IsMessage< ::ardrone_autonomy::navdata_vision_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::ardrone_autonomy::navdata_vision_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::ardrone_autonomy::navdata_vision_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::ardrone_autonomy::navdata_vision_<ContainerAllocator> const>
  : TrueType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::ardrone_autonomy::navdata_vision_<ContainerAllocator> >
{
  static const char* value()
  {
    return "55dd9a6c13917332d6d39025ed1318ab";
  }

  static const char* value(const ::ardrone_autonomy::navdata_vision_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0x55dd9a6c13917332ULL;
  static const uint64_t static_value2 = 0xd6d39025ed1318abULL;
};

template<class ContainerAllocator>
struct DataType< ::ardrone_autonomy::navdata_vision_<ContainerAllocator> >
{
  static const char* value()
  {
    return "ardrone_autonomy/navdata_vision";
  }

  static const char* value(const ::ardrone_autonomy::navdata_vision_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::ardrone_autonomy::navdata_vision_<ContainerAllocator> >
{
  static const char* value()
  {
    return "Header header\n\
float64 drone_time\n\
uint16 tag\n\
uint16 size\n\
uint32 vision_state\n\
int32 vision_misc\n\
float32 vision_phi_trim\n\
float32 vision_phi_ref_prop\n\
float32 vision_theta_trim\n\
float32 vision_theta_ref_prop\n\
int32 new_raw_picture\n\
float32 theta_capture\n\
float32 phi_capture\n\
float32 psi_capture\n\
int32 altitude_capture\n\
uint32 time_capture\n\
vector31 body_v\n\
float32 delta_phi\n\
float32 delta_theta\n\
float32 delta_psi\n\
uint32 gold_defined\n\
uint32 gold_reset\n\
float32 gold_x\n\
float32 gold_y\n\
\n\
================================================================================\n\
MSG: std_msgs/Header\n\
# Standard metadata for higher-level stamped data types.\n\
# This is generally used to communicate timestamped data \n\
# in a particular coordinate frame.\n\
# \n\
# sequence ID: consecutively increasing ID \n\
uint32 seq\n\
#Two-integer timestamp that is expressed as:\n\
# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')\n\
# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')\n\
# time-handling sugar is provided by the client library\n\
time stamp\n\
#Frame this data is associated with\n\
# 0: no frame\n\
# 1: global frame\n\
string frame_id\n\
\n\
================================================================================\n\
MSG: ardrone_autonomy/vector31\n\
float32 x\n\
float32 y\n\
float32 z\n\
";
  }

  static const char* value(const ::ardrone_autonomy::navdata_vision_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::ardrone_autonomy::navdata_vision_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.header);
      stream.next(m.drone_time);
      stream.next(m.tag);
      stream.next(m.size);
      stream.next(m.vision_state);
      stream.next(m.vision_misc);
      stream.next(m.vision_phi_trim);
      stream.next(m.vision_phi_ref_prop);
      stream.next(m.vision_theta_trim);
      stream.next(m.vision_theta_ref_prop);
      stream.next(m.new_raw_picture);
      stream.next(m.theta_capture);
      stream.next(m.phi_capture);
      stream.next(m.psi_capture);
      stream.next(m.altitude_capture);
      stream.next(m.time_capture);
      stream.next(m.body_v);
      stream.next(m.delta_phi);
      stream.next(m.delta_theta);
      stream.next(m.delta_psi);
      stream.next(m.gold_defined);
      stream.next(m.gold_reset);
      stream.next(m.gold_x);
      stream.next(m.gold_y);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct navdata_vision_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::ardrone_autonomy::navdata_vision_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::ardrone_autonomy::navdata_vision_<ContainerAllocator>& v)
  {
    s << indent << "header: ";
    s << std::endl;
    Printer< ::std_msgs::Header_<ContainerAllocator> >::stream(s, indent + "  ", v.header);
    s << indent << "drone_time: ";
    Printer<double>::stream(s, indent + "  ", v.drone_time);
    s << indent << "tag: ";
    Printer<uint16_t>::stream(s, indent + "  ", v.tag);
    s << indent << "size: ";
    Printer<uint16_t>::stream(s, indent + "  ", v.size);
    s << indent << "vision_state: ";
    Printer<uint32_t>::stream(s, indent + "  ", v.vision_state);
    s << indent << "vision_misc: ";
    Printer<int32_t>::stream(s, indent + "  ", v.vision_misc);
    s << indent << "vision_phi_trim: ";
    Printer<float>::stream(s, indent + "  ", v.vision_phi_trim);
    s << indent << "vision_phi_ref_prop: ";
    Printer<float>::stream(s, indent + "  ", v.vision_phi_ref_prop);
    s << indent << "vision_theta_trim: ";
    Printer<float>::stream(s, indent + "  ", v.vision_theta_trim);
    s << indent << "vision_theta_ref_prop: ";
    Printer<float>::stream(s, indent + "  ", v.vision_theta_ref_prop);
    s << indent << "new_raw_picture: ";
    Printer<int32_t>::stream(s, indent + "  ", v.new_raw_picture);
    s << indent << "theta_capture: ";
    Printer<float>::stream(s, indent + "  ", v.theta_capture);
    s << indent << "phi_capture: ";
    Printer<float>::stream(s, indent + "  ", v.phi_capture);
    s << indent << "psi_capture: ";
    Printer<float>::stream(s, indent + "  ", v.psi_capture);
    s << indent << "altitude_capture: ";
    Printer<int32_t>::stream(s, indent + "  ", v.altitude_capture);
    s << indent << "time_capture: ";
    Printer<uint32_t>::stream(s, indent + "  ", v.time_capture);
    s << indent << "body_v: ";
    s << std::endl;
    Printer< ::ardrone_autonomy::vector31_<ContainerAllocator> >::stream(s, indent + "  ", v.body_v);
    s << indent << "delta_phi: ";
    Printer<float>::stream(s, indent + "  ", v.delta_phi);
    s << indent << "delta_theta: ";
    Printer<float>::stream(s, indent + "  ", v.delta_theta);
    s << indent << "delta_psi: ";
    Printer<float>::stream(s, indent + "  ", v.delta_psi);
    s << indent << "gold_defined: ";
    Printer<uint32_t>::stream(s, indent + "  ", v.gold_defined);
    s << indent << "gold_reset: ";
    Printer<uint32_t>::stream(s, indent + "  ", v.gold_reset);
    s << indent << "gold_x: ";
    Printer<float>::stream(s, indent + "  ", v.gold_x);
    s << indent << "gold_y: ";
    Printer<float>::stream(s, indent + "  ", v.gold_y);
  }
};

} // namespace message_operations
} // namespace ros

#endif // ARDRONE_AUTONOMY_MESSAGE_NAVDATA_VISION_H
