// Generated by gencpp from file all_in_one_local_planner_interface/GetVelCmdResponse.msg
// DO NOT EDIT!


#ifndef ALL_IN_ONE_LOCAL_PLANNER_INTERFACE_MESSAGE_GETVELCMDRESPONSE_H
#define ALL_IN_ONE_LOCAL_PLANNER_INTERFACE_MESSAGE_GETVELCMDRESPONSE_H


#include <string>
#include <vector>
#include <memory>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>

#include <geometry_msgs/Twist.h>

namespace all_in_one_local_planner_interface
{
template <class ContainerAllocator>
struct GetVelCmdResponse_
{
  typedef GetVelCmdResponse_<ContainerAllocator> Type;

  GetVelCmdResponse_()
    : vel()
    , costmaps_resetted(false)
    , successful(false)  {
    }
  GetVelCmdResponse_(const ContainerAllocator& _alloc)
    : vel(_alloc)
    , costmaps_resetted(false)
    , successful(false)  {
  (void)_alloc;
    }



   typedef  ::geometry_msgs::Twist_<ContainerAllocator>  _vel_type;
  _vel_type vel;

   typedef uint8_t _costmaps_resetted_type;
  _costmaps_resetted_type costmaps_resetted;

   typedef uint8_t _successful_type;
  _successful_type successful;





  typedef boost::shared_ptr< ::all_in_one_local_planner_interface::GetVelCmdResponse_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::all_in_one_local_planner_interface::GetVelCmdResponse_<ContainerAllocator> const> ConstPtr;

}; // struct GetVelCmdResponse_

typedef ::all_in_one_local_planner_interface::GetVelCmdResponse_<std::allocator<void> > GetVelCmdResponse;

typedef boost::shared_ptr< ::all_in_one_local_planner_interface::GetVelCmdResponse > GetVelCmdResponsePtr;
typedef boost::shared_ptr< ::all_in_one_local_planner_interface::GetVelCmdResponse const> GetVelCmdResponseConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::all_in_one_local_planner_interface::GetVelCmdResponse_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::all_in_one_local_planner_interface::GetVelCmdResponse_<ContainerAllocator> >::stream(s, "", v);
return s;
}


template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator==(const ::all_in_one_local_planner_interface::GetVelCmdResponse_<ContainerAllocator1> & lhs, const ::all_in_one_local_planner_interface::GetVelCmdResponse_<ContainerAllocator2> & rhs)
{
  return lhs.vel == rhs.vel &&
    lhs.costmaps_resetted == rhs.costmaps_resetted &&
    lhs.successful == rhs.successful;
}

template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator!=(const ::all_in_one_local_planner_interface::GetVelCmdResponse_<ContainerAllocator1> & lhs, const ::all_in_one_local_planner_interface::GetVelCmdResponse_<ContainerAllocator2> & rhs)
{
  return !(lhs == rhs);
}


} // namespace all_in_one_local_planner_interface

namespace ros
{
namespace message_traits
{





template <class ContainerAllocator>
struct IsMessage< ::all_in_one_local_planner_interface::GetVelCmdResponse_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::all_in_one_local_planner_interface::GetVelCmdResponse_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::all_in_one_local_planner_interface::GetVelCmdResponse_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::all_in_one_local_planner_interface::GetVelCmdResponse_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::all_in_one_local_planner_interface::GetVelCmdResponse_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::all_in_one_local_planner_interface::GetVelCmdResponse_<ContainerAllocator> const>
  : FalseType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::all_in_one_local_planner_interface::GetVelCmdResponse_<ContainerAllocator> >
{
  static const char* value()
  {
    return "374e901b019d93b77917983b7fa0d888";
  }

  static const char* value(const ::all_in_one_local_planner_interface::GetVelCmdResponse_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0x374e901b019d93b7ULL;
  static const uint64_t static_value2 = 0x7917983b7fa0d888ULL;
};

template<class ContainerAllocator>
struct DataType< ::all_in_one_local_planner_interface::GetVelCmdResponse_<ContainerAllocator> >
{
  static const char* value()
  {
    return "all_in_one_local_planner_interface/GetVelCmdResponse";
  }

  static const char* value(const ::all_in_one_local_planner_interface::GetVelCmdResponse_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::all_in_one_local_planner_interface::GetVelCmdResponse_<ContainerAllocator> >
{
  static const char* value()
  {
    return "# response fields\n"
"geometry_msgs/Twist vel\n"
"bool costmaps_resetted\n"
"bool successful\n"
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

  static const char* value(const ::all_in_one_local_planner_interface::GetVelCmdResponse_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::all_in_one_local_planner_interface::GetVelCmdResponse_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.vel);
      stream.next(m.costmaps_resetted);
      stream.next(m.successful);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct GetVelCmdResponse_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::all_in_one_local_planner_interface::GetVelCmdResponse_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::all_in_one_local_planner_interface::GetVelCmdResponse_<ContainerAllocator>& v)
  {
    s << indent << "vel: ";
    s << std::endl;
    Printer< ::geometry_msgs::Twist_<ContainerAllocator> >::stream(s, indent + "  ", v.vel);
    s << indent << "costmaps_resetted: ";
    Printer<uint8_t>::stream(s, indent + "  ", v.costmaps_resetted);
    s << indent << "successful: ";
    Printer<uint8_t>::stream(s, indent + "  ", v.successful);
  }
};

} // namespace message_operations
} // namespace ros

#endif // ALL_IN_ONE_LOCAL_PLANNER_INTERFACE_MESSAGE_GETVELCMDRESPONSE_H
