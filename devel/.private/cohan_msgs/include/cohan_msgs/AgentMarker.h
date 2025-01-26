// Generated by gencpp from file cohan_msgs/AgentMarker.msg
// DO NOT EDIT!


#ifndef COHAN_MSGS_MESSAGE_AGENTMARKER_H
#define COHAN_MSGS_MESSAGE_AGENTMARKER_H


#include <string>
#include <vector>
#include <memory>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>

#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Twist.h>

namespace cohan_msgs
{
template <class ContainerAllocator>
struct AgentMarker_
{
  typedef AgentMarker_<ContainerAllocator> Type;

  AgentMarker_()
    : id(0)
    , active(false)
    , pose()
    , velocity()  {
    }
  AgentMarker_(const ContainerAllocator& _alloc)
    : id(0)
    , active(false)
    , pose(_alloc)
    , velocity(_alloc)  {
  (void)_alloc;
    }



   typedef uint64_t _id_type;
  _id_type id;

   typedef uint8_t _active_type;
  _active_type active;

   typedef  ::geometry_msgs::Pose_<ContainerAllocator>  _pose_type;
  _pose_type pose;

   typedef  ::geometry_msgs::Twist_<ContainerAllocator>  _velocity_type;
  _velocity_type velocity;





  typedef boost::shared_ptr< ::cohan_msgs::AgentMarker_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::cohan_msgs::AgentMarker_<ContainerAllocator> const> ConstPtr;

}; // struct AgentMarker_

typedef ::cohan_msgs::AgentMarker_<std::allocator<void> > AgentMarker;

typedef boost::shared_ptr< ::cohan_msgs::AgentMarker > AgentMarkerPtr;
typedef boost::shared_ptr< ::cohan_msgs::AgentMarker const> AgentMarkerConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::cohan_msgs::AgentMarker_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::cohan_msgs::AgentMarker_<ContainerAllocator> >::stream(s, "", v);
return s;
}


template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator==(const ::cohan_msgs::AgentMarker_<ContainerAllocator1> & lhs, const ::cohan_msgs::AgentMarker_<ContainerAllocator2> & rhs)
{
  return lhs.id == rhs.id &&
    lhs.active == rhs.active &&
    lhs.pose == rhs.pose &&
    lhs.velocity == rhs.velocity;
}

template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator!=(const ::cohan_msgs::AgentMarker_<ContainerAllocator1> & lhs, const ::cohan_msgs::AgentMarker_<ContainerAllocator2> & rhs)
{
  return !(lhs == rhs);
}


} // namespace cohan_msgs

namespace ros
{
namespace message_traits
{





template <class ContainerAllocator>
struct IsMessage< ::cohan_msgs::AgentMarker_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::cohan_msgs::AgentMarker_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::cohan_msgs::AgentMarker_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::cohan_msgs::AgentMarker_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::cohan_msgs::AgentMarker_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::cohan_msgs::AgentMarker_<ContainerAllocator> const>
  : FalseType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::cohan_msgs::AgentMarker_<ContainerAllocator> >
{
  static const char* value()
  {
    return "7e3f0ea93981f0d1c92f80214d1d82f6";
  }

  static const char* value(const ::cohan_msgs::AgentMarker_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0x7e3f0ea93981f0d1ULL;
  static const uint64_t static_value2 = 0xc92f80214d1d82f6ULL;
};

template<class ContainerAllocator>
struct DataType< ::cohan_msgs::AgentMarker_<ContainerAllocator> >
{
  static const char* value()
  {
    return "cohan_msgs/AgentMarker";
  }

  static const char* value(const ::cohan_msgs::AgentMarker_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::cohan_msgs::AgentMarker_<ContainerAllocator> >
{
  static const char* value()
  {
    return "uint64              id\n"
"bool                active\n"
"geometry_msgs/Pose  pose\n"
"geometry_msgs/Twist velocity\n"
"\n"
"================================================================================\n"
"MSG: geometry_msgs/Pose\n"
"# A representation of pose in free space, composed of position and orientation. \n"
"Point position\n"
"Quaternion orientation\n"
"\n"
"================================================================================\n"
"MSG: geometry_msgs/Point\n"
"# This contains the position of a point in free space\n"
"float64 x\n"
"float64 y\n"
"float64 z\n"
"\n"
"================================================================================\n"
"MSG: geometry_msgs/Quaternion\n"
"# This represents an orientation in free space in quaternion form.\n"
"\n"
"float64 x\n"
"float64 y\n"
"float64 z\n"
"float64 w\n"
"\n"
"================================================================================\n"
"MSG: geometry_msgs/Twist\n"
"# This expresses velocity in free space broken into its linear and angular parts.\n"
"Vector3  linear\n"
"Vector3  angular\n"
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

  static const char* value(const ::cohan_msgs::AgentMarker_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::cohan_msgs::AgentMarker_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.id);
      stream.next(m.active);
      stream.next(m.pose);
      stream.next(m.velocity);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct AgentMarker_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::cohan_msgs::AgentMarker_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::cohan_msgs::AgentMarker_<ContainerAllocator>& v)
  {
    s << indent << "id: ";
    Printer<uint64_t>::stream(s, indent + "  ", v.id);
    s << indent << "active: ";
    Printer<uint8_t>::stream(s, indent + "  ", v.active);
    s << indent << "pose: ";
    s << std::endl;
    Printer< ::geometry_msgs::Pose_<ContainerAllocator> >::stream(s, indent + "  ", v.pose);
    s << indent << "velocity: ";
    s << std::endl;
    Printer< ::geometry_msgs::Twist_<ContainerAllocator> >::stream(s, indent + "  ", v.velocity);
  }
};

} // namespace message_operations
} // namespace ros

#endif // COHAN_MSGS_MESSAGE_AGENTMARKER_H
