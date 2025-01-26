// Generated by gencpp from file hateb_local_planner/getOptimVelResponse.msg
// DO NOT EDIT!


#ifndef HATEB_LOCAL_PLANNER_MESSAGE_GETOPTIMVELRESPONSE_H
#define HATEB_LOCAL_PLANNER_MESSAGE_GETOPTIMVELRESPONSE_H


#include <string>
#include <vector>
#include <memory>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>

#include <geometry_msgs/Twist.h>

namespace hateb_local_planner
{
template <class ContainerAllocator>
struct getOptimVelResponse_
{
  typedef getOptimVelResponse_<ContainerAllocator> Type;

  getOptimVelResponse_()
    : success(false)
    , message()
    , cmd_vel()  {
    }
  getOptimVelResponse_(const ContainerAllocator& _alloc)
    : success(false)
    , message(_alloc)
    , cmd_vel(_alloc)  {
  (void)_alloc;
    }



   typedef uint8_t _success_type;
  _success_type success;

   typedef std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>> _message_type;
  _message_type message;

   typedef  ::geometry_msgs::Twist_<ContainerAllocator>  _cmd_vel_type;
  _cmd_vel_type cmd_vel;





  typedef boost::shared_ptr< ::hateb_local_planner::getOptimVelResponse_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::hateb_local_planner::getOptimVelResponse_<ContainerAllocator> const> ConstPtr;

}; // struct getOptimVelResponse_

typedef ::hateb_local_planner::getOptimVelResponse_<std::allocator<void> > getOptimVelResponse;

typedef boost::shared_ptr< ::hateb_local_planner::getOptimVelResponse > getOptimVelResponsePtr;
typedef boost::shared_ptr< ::hateb_local_planner::getOptimVelResponse const> getOptimVelResponseConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::hateb_local_planner::getOptimVelResponse_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::hateb_local_planner::getOptimVelResponse_<ContainerAllocator> >::stream(s, "", v);
return s;
}


template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator==(const ::hateb_local_planner::getOptimVelResponse_<ContainerAllocator1> & lhs, const ::hateb_local_planner::getOptimVelResponse_<ContainerAllocator2> & rhs)
{
  return lhs.success == rhs.success &&
    lhs.message == rhs.message &&
    lhs.cmd_vel == rhs.cmd_vel;
}

template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator!=(const ::hateb_local_planner::getOptimVelResponse_<ContainerAllocator1> & lhs, const ::hateb_local_planner::getOptimVelResponse_<ContainerAllocator2> & rhs)
{
  return !(lhs == rhs);
}


} // namespace hateb_local_planner

namespace ros
{
namespace message_traits
{





template <class ContainerAllocator>
struct IsMessage< ::hateb_local_planner::getOptimVelResponse_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::hateb_local_planner::getOptimVelResponse_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::hateb_local_planner::getOptimVelResponse_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::hateb_local_planner::getOptimVelResponse_<ContainerAllocator> const>
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::hateb_local_planner::getOptimVelResponse_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::hateb_local_planner::getOptimVelResponse_<ContainerAllocator> const>
  : FalseType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::hateb_local_planner::getOptimVelResponse_<ContainerAllocator> >
{
  static const char* value()
  {
    return "09e6d25ee858159d0ccff5501db19855";
  }

  static const char* value(const ::hateb_local_planner::getOptimVelResponse_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0x09e6d25ee858159dULL;
  static const uint64_t static_value2 = 0x0ccff5501db19855ULL;
};

template<class ContainerAllocator>
struct DataType< ::hateb_local_planner::getOptimVelResponse_<ContainerAllocator> >
{
  static const char* value()
  {
    return "hateb_local_planner/getOptimVelResponse";
  }

  static const char* value(const ::hateb_local_planner::getOptimVelResponse_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::hateb_local_planner::getOptimVelResponse_<ContainerAllocator> >
{
  static const char* value()
  {
    return "bool success\n"
"string message\n"
"geometry_msgs/Twist cmd_vel\n"
"\n"
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

  static const char* value(const ::hateb_local_planner::getOptimVelResponse_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::hateb_local_planner::getOptimVelResponse_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.success);
      stream.next(m.message);
      stream.next(m.cmd_vel);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct getOptimVelResponse_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::hateb_local_planner::getOptimVelResponse_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::hateb_local_planner::getOptimVelResponse_<ContainerAllocator>& v)
  {
    s << indent << "success: ";
    Printer<uint8_t>::stream(s, indent + "  ", v.success);
    s << indent << "message: ";
    Printer<std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>>::stream(s, indent + "  ", v.message);
    s << indent << "cmd_vel: ";
    s << std::endl;
    Printer< ::geometry_msgs::Twist_<ContainerAllocator> >::stream(s, indent + "  ", v.cmd_vel);
  }
};

} // namespace message_operations
} // namespace ros

#endif // HATEB_LOCAL_PLANNER_MESSAGE_GETOPTIMVELRESPONSE_H
