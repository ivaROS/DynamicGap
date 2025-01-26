// Generated by gencpp from file pedsim_msgs/AgentState.msg
// DO NOT EDIT!


#ifndef PEDSIM_MSGS_MESSAGE_AGENTSTATE_H
#define PEDSIM_MSGS_MESSAGE_AGENTSTATE_H


#include <string>
#include <vector>
#include <memory>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>

#include <std_msgs/Header.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Twist.h>
#include <pedsim_msgs/AgentForce.h>
#include <geometry_msgs/Vector3.h>
#include <geometry_msgs/Vector3.h>

namespace pedsim_msgs
{
template <class ContainerAllocator>
struct AgentState_
{
  typedef AgentState_<ContainerAllocator> Type;

  AgentState_()
    : header()
    , id()
    , type()
    , social_state()
    , pose()
    , twist()
    , forces()
    , talking_to_id()
    , listening_to_id()
    , acceleration()
    , destination()
    , direction(0.0)
    , configuration()  {
    }
  AgentState_(const ContainerAllocator& _alloc)
    : header(_alloc)
    , id(_alloc)
    , type(_alloc)
    , social_state(_alloc)
    , pose(_alloc)
    , twist(_alloc)
    , forces(_alloc)
    , talking_to_id(_alloc)
    , listening_to_id(_alloc)
    , acceleration(_alloc)
    , destination(_alloc)
    , direction(0.0)
    , configuration(_alloc)  {
  (void)_alloc;
    }



   typedef  ::std_msgs::Header_<ContainerAllocator>  _header_type;
  _header_type header;

   typedef std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>> _id_type;
  _id_type id;

   typedef std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>> _type_type;
  _type_type type;

   typedef std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>> _social_state_type;
  _social_state_type social_state;

   typedef  ::geometry_msgs::Pose_<ContainerAllocator>  _pose_type;
  _pose_type pose;

   typedef  ::geometry_msgs::Twist_<ContainerAllocator>  _twist_type;
  _twist_type twist;

   typedef  ::pedsim_msgs::AgentForce_<ContainerAllocator>  _forces_type;
  _forces_type forces;

   typedef std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>> _talking_to_id_type;
  _talking_to_id_type talking_to_id;

   typedef std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>> _listening_to_id_type;
  _listening_to_id_type listening_to_id;

   typedef  ::geometry_msgs::Vector3_<ContainerAllocator>  _acceleration_type;
  _acceleration_type acceleration;

   typedef  ::geometry_msgs::Vector3_<ContainerAllocator>  _destination_type;
  _destination_type destination;

   typedef double _direction_type;
  _direction_type direction;

   typedef std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>> _configuration_type;
  _configuration_type configuration;



// reducing the odds to have name collisions with Windows.h 
#if defined(_WIN32) && defined(IDLE)
  #undef IDLE
#endif
#if defined(_WIN32) && defined(WALKING)
  #undef WALKING
#endif
#if defined(_WIN32) && defined(RUNNING)
  #undef RUNNING
#endif
#if defined(_WIN32) && defined(INTERACTING)
  #undef INTERACTING
#endif
#if defined(_WIN32) && defined(TALKING)
  #undef TALKING
#endif
#if defined(_WIN32) && defined(PHONE)
  #undef PHONE
#endif

  enum {
    IDLE = 0u,
    WALKING = 1u,
    RUNNING = 2u,
    INTERACTING = 3u,
    TALKING = 4u,
    PHONE = 5u,
  };


  typedef boost::shared_ptr< ::pedsim_msgs::AgentState_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::pedsim_msgs::AgentState_<ContainerAllocator> const> ConstPtr;

}; // struct AgentState_

typedef ::pedsim_msgs::AgentState_<std::allocator<void> > AgentState;

typedef boost::shared_ptr< ::pedsim_msgs::AgentState > AgentStatePtr;
typedef boost::shared_ptr< ::pedsim_msgs::AgentState const> AgentStateConstPtr;

// constants requiring out of line definition

   

   

   

   

   

   



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::pedsim_msgs::AgentState_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::pedsim_msgs::AgentState_<ContainerAllocator> >::stream(s, "", v);
return s;
}


template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator==(const ::pedsim_msgs::AgentState_<ContainerAllocator1> & lhs, const ::pedsim_msgs::AgentState_<ContainerAllocator2> & rhs)
{
  return lhs.header == rhs.header &&
    lhs.id == rhs.id &&
    lhs.type == rhs.type &&
    lhs.social_state == rhs.social_state &&
    lhs.pose == rhs.pose &&
    lhs.twist == rhs.twist &&
    lhs.forces == rhs.forces &&
    lhs.talking_to_id == rhs.talking_to_id &&
    lhs.listening_to_id == rhs.listening_to_id &&
    lhs.acceleration == rhs.acceleration &&
    lhs.destination == rhs.destination &&
    lhs.direction == rhs.direction &&
    lhs.configuration == rhs.configuration;
}

template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator!=(const ::pedsim_msgs::AgentState_<ContainerAllocator1> & lhs, const ::pedsim_msgs::AgentState_<ContainerAllocator2> & rhs)
{
  return !(lhs == rhs);
}


} // namespace pedsim_msgs

namespace ros
{
namespace message_traits
{





template <class ContainerAllocator>
struct IsMessage< ::pedsim_msgs::AgentState_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::pedsim_msgs::AgentState_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::pedsim_msgs::AgentState_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::pedsim_msgs::AgentState_<ContainerAllocator> const>
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::pedsim_msgs::AgentState_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::pedsim_msgs::AgentState_<ContainerAllocator> const>
  : TrueType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::pedsim_msgs::AgentState_<ContainerAllocator> >
{
  static const char* value()
  {
    return "870e02a1932fc0a93c5d52d2b22efada";
  }

  static const char* value(const ::pedsim_msgs::AgentState_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0x870e02a1932fc0a9ULL;
  static const uint64_t static_value2 = 0x3c5d52d2b22efadaULL;
};

template<class ContainerAllocator>
struct DataType< ::pedsim_msgs::AgentState_<ContainerAllocator> >
{
  static const char* value()
  {
    return "pedsim_msgs/AgentState";
  }

  static const char* value(const ::pedsim_msgs::AgentState_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::pedsim_msgs::AgentState_<ContainerAllocator> >
{
  static const char* value()
  {
    return "Header header\n"
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

  static const char* value(const ::pedsim_msgs::AgentState_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::pedsim_msgs::AgentState_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.header);
      stream.next(m.id);
      stream.next(m.type);
      stream.next(m.social_state);
      stream.next(m.pose);
      stream.next(m.twist);
      stream.next(m.forces);
      stream.next(m.talking_to_id);
      stream.next(m.listening_to_id);
      stream.next(m.acceleration);
      stream.next(m.destination);
      stream.next(m.direction);
      stream.next(m.configuration);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct AgentState_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::pedsim_msgs::AgentState_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::pedsim_msgs::AgentState_<ContainerAllocator>& v)
  {
    s << indent << "header: ";
    s << std::endl;
    Printer< ::std_msgs::Header_<ContainerAllocator> >::stream(s, indent + "  ", v.header);
    s << indent << "id: ";
    Printer<std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>>::stream(s, indent + "  ", v.id);
    s << indent << "type: ";
    Printer<std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>>::stream(s, indent + "  ", v.type);
    s << indent << "social_state: ";
    Printer<std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>>::stream(s, indent + "  ", v.social_state);
    s << indent << "pose: ";
    s << std::endl;
    Printer< ::geometry_msgs::Pose_<ContainerAllocator> >::stream(s, indent + "  ", v.pose);
    s << indent << "twist: ";
    s << std::endl;
    Printer< ::geometry_msgs::Twist_<ContainerAllocator> >::stream(s, indent + "  ", v.twist);
    s << indent << "forces: ";
    s << std::endl;
    Printer< ::pedsim_msgs::AgentForce_<ContainerAllocator> >::stream(s, indent + "  ", v.forces);
    s << indent << "talking_to_id: ";
    Printer<std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>>::stream(s, indent + "  ", v.talking_to_id);
    s << indent << "listening_to_id: ";
    Printer<std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>>::stream(s, indent + "  ", v.listening_to_id);
    s << indent << "acceleration: ";
    s << std::endl;
    Printer< ::geometry_msgs::Vector3_<ContainerAllocator> >::stream(s, indent + "  ", v.acceleration);
    s << indent << "destination: ";
    s << std::endl;
    Printer< ::geometry_msgs::Vector3_<ContainerAllocator> >::stream(s, indent + "  ", v.destination);
    s << indent << "direction: ";
    Printer<double>::stream(s, indent + "  ", v.direction);
    s << indent << "configuration: ";
    Printer<std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>>::stream(s, indent + "  ", v.configuration);
  }
};

} // namespace message_operations
} // namespace ros

#endif // PEDSIM_MSGS_MESSAGE_AGENTSTATE_H
