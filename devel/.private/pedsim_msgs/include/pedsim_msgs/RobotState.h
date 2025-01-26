// Generated by gencpp from file pedsim_msgs/RobotState.msg
// DO NOT EDIT!


#ifndef PEDSIM_MSGS_MESSAGE_ROBOTSTATE_H
#define PEDSIM_MSGS_MESSAGE_ROBOTSTATE_H


#include <string>
#include <vector>
#include <memory>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>

#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Twist.h>

namespace pedsim_msgs
{
template <class ContainerAllocator>
struct RobotState_
{
  typedef RobotState_<ContainerAllocator> Type;

  RobotState_()
    : name()
    , pose()
    , twist()  {
    }
  RobotState_(const ContainerAllocator& _alloc)
    : name(_alloc)
    , pose(_alloc)
    , twist(_alloc)  {
  (void)_alloc;
    }



   typedef std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>> _name_type;
  _name_type name;

   typedef  ::geometry_msgs::Pose_<ContainerAllocator>  _pose_type;
  _pose_type pose;

   typedef  ::geometry_msgs::Twist_<ContainerAllocator>  _twist_type;
  _twist_type twist;





  typedef boost::shared_ptr< ::pedsim_msgs::RobotState_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::pedsim_msgs::RobotState_<ContainerAllocator> const> ConstPtr;

}; // struct RobotState_

typedef ::pedsim_msgs::RobotState_<std::allocator<void> > RobotState;

typedef boost::shared_ptr< ::pedsim_msgs::RobotState > RobotStatePtr;
typedef boost::shared_ptr< ::pedsim_msgs::RobotState const> RobotStateConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::pedsim_msgs::RobotState_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::pedsim_msgs::RobotState_<ContainerAllocator> >::stream(s, "", v);
return s;
}


template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator==(const ::pedsim_msgs::RobotState_<ContainerAllocator1> & lhs, const ::pedsim_msgs::RobotState_<ContainerAllocator2> & rhs)
{
  return lhs.name == rhs.name &&
    lhs.pose == rhs.pose &&
    lhs.twist == rhs.twist;
}

template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator!=(const ::pedsim_msgs::RobotState_<ContainerAllocator1> & lhs, const ::pedsim_msgs::RobotState_<ContainerAllocator2> & rhs)
{
  return !(lhs == rhs);
}


} // namespace pedsim_msgs

namespace ros
{
namespace message_traits
{





template <class ContainerAllocator>
struct IsMessage< ::pedsim_msgs::RobotState_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::pedsim_msgs::RobotState_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::pedsim_msgs::RobotState_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::pedsim_msgs::RobotState_<ContainerAllocator> const>
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::pedsim_msgs::RobotState_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::pedsim_msgs::RobotState_<ContainerAllocator> const>
  : FalseType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::pedsim_msgs::RobotState_<ContainerAllocator> >
{
  static const char* value()
  {
    return "9d6c189ebf50f7f50c552b96ac2c620a";
  }

  static const char* value(const ::pedsim_msgs::RobotState_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0x9d6c189ebf50f7f5ULL;
  static const uint64_t static_value2 = 0x0c552b96ac2c620aULL;
};

template<class ContainerAllocator>
struct DataType< ::pedsim_msgs::RobotState_<ContainerAllocator> >
{
  static const char* value()
  {
    return "pedsim_msgs/RobotState";
  }

  static const char* value(const ::pedsim_msgs::RobotState_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::pedsim_msgs::RobotState_<ContainerAllocator> >
{
  static const char* value()
  {
    return "string name\n"
"geometry_msgs/Pose pose\n"
"geometry_msgs/Twist twist\n"
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

  static const char* value(const ::pedsim_msgs::RobotState_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::pedsim_msgs::RobotState_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.name);
      stream.next(m.pose);
      stream.next(m.twist);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct RobotState_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::pedsim_msgs::RobotState_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::pedsim_msgs::RobotState_<ContainerAllocator>& v)
  {
    s << indent << "name: ";
    Printer<std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>>::stream(s, indent + "  ", v.name);
    s << indent << "pose: ";
    s << std::endl;
    Printer< ::geometry_msgs::Pose_<ContainerAllocator> >::stream(s, indent + "  ", v.pose);
    s << indent << "twist: ";
    s << std::endl;
    Printer< ::geometry_msgs::Twist_<ContainerAllocator> >::stream(s, indent + "  ", v.twist);
  }
};

} // namespace message_operations
} // namespace ros

#endif // PEDSIM_MSGS_MESSAGE_ROBOTSTATE_H
