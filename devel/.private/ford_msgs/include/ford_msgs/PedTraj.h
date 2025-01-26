// Generated by gencpp from file ford_msgs/PedTraj.msg
// DO NOT EDIT!


#ifndef FORD_MSGS_MESSAGE_PEDTRAJ_H
#define FORD_MSGS_MESSAGE_PEDTRAJ_H


#include <string>
#include <vector>
#include <memory>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>

#include <ford_msgs/Pose2DStamped.h>

namespace ford_msgs
{
template <class ContainerAllocator>
struct PedTraj_
{
  typedef PedTraj_<ContainerAllocator> Type;

  PedTraj_()
    : ped_id(0)
    , traj()
    , value(0.0)
    , type(0)  {
    }
  PedTraj_(const ContainerAllocator& _alloc)
    : ped_id(0)
    , traj(_alloc)
    , value(0.0)
    , type(0)  {
  (void)_alloc;
    }



   typedef uint32_t _ped_id_type;
  _ped_id_type ped_id;

   typedef std::vector< ::ford_msgs::Pose2DStamped_<ContainerAllocator> , typename std::allocator_traits<ContainerAllocator>::template rebind_alloc< ::ford_msgs::Pose2DStamped_<ContainerAllocator> >> _traj_type;
  _traj_type traj;

   typedef float _value_type;
  _value_type value;

   typedef uint8_t _type_type;
  _type_type type;



// reducing the odds to have name collisions with Windows.h 
#if defined(_WIN32) && defined(OBSERVATION)
  #undef OBSERVATION
#endif
#if defined(_WIN32) && defined(PREDICTION_LINEAR)
  #undef PREDICTION_LINEAR
#endif
#if defined(_WIN32) && defined(PREDICTION_GP)
  #undef PREDICTION_GP
#endif

  enum {
    OBSERVATION = 0u,
    PREDICTION_LINEAR = 1u,
    PREDICTION_GP = 2u,
  };


  typedef boost::shared_ptr< ::ford_msgs::PedTraj_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::ford_msgs::PedTraj_<ContainerAllocator> const> ConstPtr;

}; // struct PedTraj_

typedef ::ford_msgs::PedTraj_<std::allocator<void> > PedTraj;

typedef boost::shared_ptr< ::ford_msgs::PedTraj > PedTrajPtr;
typedef boost::shared_ptr< ::ford_msgs::PedTraj const> PedTrajConstPtr;

// constants requiring out of line definition

   

   

   



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::ford_msgs::PedTraj_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::ford_msgs::PedTraj_<ContainerAllocator> >::stream(s, "", v);
return s;
}


template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator==(const ::ford_msgs::PedTraj_<ContainerAllocator1> & lhs, const ::ford_msgs::PedTraj_<ContainerAllocator2> & rhs)
{
  return lhs.ped_id == rhs.ped_id &&
    lhs.traj == rhs.traj &&
    lhs.value == rhs.value &&
    lhs.type == rhs.type;
}

template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator!=(const ::ford_msgs::PedTraj_<ContainerAllocator1> & lhs, const ::ford_msgs::PedTraj_<ContainerAllocator2> & rhs)
{
  return !(lhs == rhs);
}


} // namespace ford_msgs

namespace ros
{
namespace message_traits
{





template <class ContainerAllocator>
struct IsMessage< ::ford_msgs::PedTraj_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::ford_msgs::PedTraj_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::ford_msgs::PedTraj_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::ford_msgs::PedTraj_<ContainerAllocator> const>
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::ford_msgs::PedTraj_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::ford_msgs::PedTraj_<ContainerAllocator> const>
  : FalseType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::ford_msgs::PedTraj_<ContainerAllocator> >
{
  static const char* value()
  {
    return "8925770871a7e9ee7020a76e368dc696";
  }

  static const char* value(const ::ford_msgs::PedTraj_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0x8925770871a7e9eeULL;
  static const uint64_t static_value2 = 0x7020a76e368dc696ULL;
};

template<class ContainerAllocator>
struct DataType< ::ford_msgs::PedTraj_<ContainerAllocator> >
{
  static const char* value()
  {
    return "ford_msgs/PedTraj";
  }

  static const char* value(const ::ford_msgs::PedTraj_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::ford_msgs::PedTraj_<ContainerAllocator> >
{
  static const char* value()
  {
    return "uint32 ped_id\n"
"ford_msgs/Pose2DStamped[] traj\n"
"float32 value #Used for PREDICTION types\n"
"uint8 type \n"
"#ENUM for type\n"
"uint8 OBSERVATION=0\n"
"uint8 PREDICTION_LINEAR=1\n"
"uint8 PREDICTION_GP=2\n"
"================================================================================\n"
"MSG: ford_msgs/Pose2DStamped\n"
"std_msgs/Header header\n"
"geometry_msgs/Pose2D pose\n"
"geometry_msgs/Vector3 velocity\n"
"================================================================================\n"
"MSG: std_msgs/Header\n"
"# Standard metadata for higher-level stamped data types.\n"
"# This is generally used to communicate timestamped data \n"
"# in a particular coordinate frame.\n"
"# \n"
"# sequence ID: consecutively increasing ID \n"
"uint32 seq\n"
"#Two-integer timestamp that is expressed as:\n"
"# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')\n"
"# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')\n"
"# time-handling sugar is provided by the client library\n"
"time stamp\n"
"#Frame this data is associated with\n"
"string frame_id\n"
"\n"
"================================================================================\n"
"MSG: geometry_msgs/Pose2D\n"
"# Deprecated\n"
"# Please use the full 3D pose.\n"
"\n"
"# In general our recommendation is to use a full 3D representation of everything and for 2D specific applications make the appropriate projections into the plane for their calculations but optimally will preserve the 3D information during processing.\n"
"\n"
"# If we have parallel copies of 2D datatypes every UI and other pipeline will end up needing to have dual interfaces to plot everything. And you will end up with not being able to use 3D tools for 2D use cases even if they're completely valid, as you'd have to reimplement it with different inputs and outputs. It's not particularly hard to plot the 2D pose or compute the yaw error for the Pose message and there are already tools and libraries that can do this for you.\n"
"\n"
"\n"
"# This expresses a position and orientation on a 2D manifold.\n"
"\n"
"float64 x\n"
"float64 y\n"
"float64 theta\n"
"\n"
"================================================================================\n"
"MSG: geometry_msgs/Vector3\n"
"# This represents a vector in free space. \n"
"# It is only meant to represent a direction. Therefore, it does not\n"
"# make sense to apply a translation to it (e.g., when applying a \n"
"# generic rigid transformation to a Vector3, tf2 will only apply the\n"
"# rotation). If you want your data to be translatable too, use the\n"
"# geometry_msgs/Point message instead.\n"
"\n"
"float64 x\n"
"float64 y\n"
"float64 z\n"
;
  }

  static const char* value(const ::ford_msgs::PedTraj_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::ford_msgs::PedTraj_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.ped_id);
      stream.next(m.traj);
      stream.next(m.value);
      stream.next(m.type);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct PedTraj_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::ford_msgs::PedTraj_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::ford_msgs::PedTraj_<ContainerAllocator>& v)
  {
    s << indent << "ped_id: ";
    Printer<uint32_t>::stream(s, indent + "  ", v.ped_id);
    s << indent << "traj[]" << std::endl;
    for (size_t i = 0; i < v.traj.size(); ++i)
    {
      s << indent << "  traj[" << i << "]: ";
      s << std::endl;
      s << indent;
      Printer< ::ford_msgs::Pose2DStamped_<ContainerAllocator> >::stream(s, indent + "    ", v.traj[i]);
    }
    s << indent << "value: ";
    Printer<float>::stream(s, indent + "  ", v.value);
    s << indent << "type: ";
    Printer<uint8_t>::stream(s, indent + "  ", v.type);
  }
};

} // namespace message_operations
} // namespace ros

#endif // FORD_MSGS_MESSAGE_PEDTRAJ_H
