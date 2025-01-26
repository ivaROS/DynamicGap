// Generated by gencpp from file pedsim_srvs/GetAgentStateResponse.msg
// DO NOT EDIT!


#ifndef PEDSIM_SRVS_MESSAGE_GETAGENTSTATERESPONSE_H
#define PEDSIM_SRVS_MESSAGE_GETAGENTSTATERESPONSE_H


#include <string>
#include <vector>
#include <memory>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>

#include <pedsim_msgs/AgentState.h>

namespace pedsim_srvs
{
template <class ContainerAllocator>
struct GetAgentStateResponse_
{
  typedef GetAgentStateResponse_<ContainerAllocator> Type;

  GetAgentStateResponse_()
    : state()  {
    }
  GetAgentStateResponse_(const ContainerAllocator& _alloc)
    : state(_alloc)  {
  (void)_alloc;
    }



   typedef  ::pedsim_msgs::AgentState_<ContainerAllocator>  _state_type;
  _state_type state;





  typedef boost::shared_ptr< ::pedsim_srvs::GetAgentStateResponse_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::pedsim_srvs::GetAgentStateResponse_<ContainerAllocator> const> ConstPtr;

}; // struct GetAgentStateResponse_

typedef ::pedsim_srvs::GetAgentStateResponse_<std::allocator<void> > GetAgentStateResponse;

typedef boost::shared_ptr< ::pedsim_srvs::GetAgentStateResponse > GetAgentStateResponsePtr;
typedef boost::shared_ptr< ::pedsim_srvs::GetAgentStateResponse const> GetAgentStateResponseConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::pedsim_srvs::GetAgentStateResponse_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::pedsim_srvs::GetAgentStateResponse_<ContainerAllocator> >::stream(s, "", v);
return s;
}


template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator==(const ::pedsim_srvs::GetAgentStateResponse_<ContainerAllocator1> & lhs, const ::pedsim_srvs::GetAgentStateResponse_<ContainerAllocator2> & rhs)
{
  return lhs.state == rhs.state;
}

template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator!=(const ::pedsim_srvs::GetAgentStateResponse_<ContainerAllocator1> & lhs, const ::pedsim_srvs::GetAgentStateResponse_<ContainerAllocator2> & rhs)
{
  return !(lhs == rhs);
}


} // namespace pedsim_srvs

namespace ros
{
namespace message_traits
{





template <class ContainerAllocator>
struct IsMessage< ::pedsim_srvs::GetAgentStateResponse_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::pedsim_srvs::GetAgentStateResponse_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::pedsim_srvs::GetAgentStateResponse_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::pedsim_srvs::GetAgentStateResponse_<ContainerAllocator> const>
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::pedsim_srvs::GetAgentStateResponse_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::pedsim_srvs::GetAgentStateResponse_<ContainerAllocator> const>
  : FalseType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::pedsim_srvs::GetAgentStateResponse_<ContainerAllocator> >
{
  static const char* value()
  {
    return "9b06316bc5ce7c69f46ecaadc6ca88d1";
  }

  static const char* value(const ::pedsim_srvs::GetAgentStateResponse_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0x9b06316bc5ce7c69ULL;
  static const uint64_t static_value2 = 0xf46ecaadc6ca88d1ULL;
};

template<class ContainerAllocator>
struct DataType< ::pedsim_srvs::GetAgentStateResponse_<ContainerAllocator> >
{
  static const char* value()
  {
    return "pedsim_srvs/GetAgentStateResponse";
  }

  static const char* value(const ::pedsim_srvs::GetAgentStateResponse_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::pedsim_srvs::GetAgentStateResponse_<ContainerAllocator> >
{
  static const char* value()
  {
    return "pedsim_msgs/AgentState state\n"
"\n"
"================================================================================\n"
"MSG: pedsim_msgs/AgentState\n"
"Header header\n"
"string id\n"
"string type\n"
"string social_state\n"
"geometry_msgs/Pose pose\n"
"geometry_msgs/Twist twist\n"
"pedsim_msgs/AgentForce forces\n"
"string talking_to_id\n"
"string listening_to_id\n"
"geometry_msgs/Vector3 acceleration\n"
"geometry_msgs/Vector3 destination\n"
"float64 direction\n"
"string configuration\n"
"\n"
"uint8 IDLE        = 0\n"
"uint8 WALKING     = 1\n"
"uint8 RUNNING     = 2\n"
"uint8 INTERACTING = 3\n"
"uint8 TALKING     = 4\n"
"uint8 PHONE       = 5\n"
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
"================================================================================\n"
"MSG: pedsim_msgs/AgentForce\n"
"# Forces acting on an agent.\n"
"\n"
"# Max Speed\n"
"float64 vmax\n"
"\n"
"# Force Factors\n"
"float64 desired_ffactor\n"
"float64 obstacle_ffactor\n"
"float64 social_ffactor\n"
"float64 robot_ffactor\n"
"\n"
"# Basic SFM forces.\n"
"geometry_msgs/Vector3 desired_force\n"
"geometry_msgs/Vector3 obstacle_force\n"
"geometry_msgs/Vector3 social_force\n"
"\n"
"# Additional Group Forces\n"
"geometry_msgs/Vector3 group_coherence_force\n"
"geometry_msgs/Vector3 group_gaze_force\n"
"geometry_msgs/Vector3 group_repulsion_force\n"
"\n"
"# Extra stabilization/custom forces.\n"
"geometry_msgs/Vector3 random_force\n"
"geometry_msgs/Vector3 keep_distance_force\n"
"geometry_msgs/Vector3 robot_force\n"
"\n"
"# Total forces\n"
"geometry_msgs/Vector3 force\n"
;
  }

  static const char* value(const ::pedsim_srvs::GetAgentStateResponse_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::pedsim_srvs::GetAgentStateResponse_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.state);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct GetAgentStateResponse_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::pedsim_srvs::GetAgentStateResponse_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::pedsim_srvs::GetAgentStateResponse_<ContainerAllocator>& v)
  {
    s << indent << "state: ";
    s << std::endl;
    Printer< ::pedsim_msgs::AgentState_<ContainerAllocator> >::stream(s, indent + "  ", v.state);
  }
};

} // namespace message_operations
} // namespace ros

#endif // PEDSIM_SRVS_MESSAGE_GETAGENTSTATERESPONSE_H
