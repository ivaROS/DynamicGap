// Generated by gencpp from file pedsim_msgs/PedsimAgentsDataframe.msg
// DO NOT EDIT!


#ifndef PEDSIM_MSGS_MESSAGE_PEDSIMAGENTSDATAFRAME_H
#define PEDSIM_MSGS_MESSAGE_PEDSIMAGENTSDATAFRAME_H


#include <string>
#include <vector>
#include <memory>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>

#include <std_msgs/Header.h>
#include <pedsim_msgs/AgentState.h>
#include <pedsim_msgs/RobotState.h>
#include <pedsim_msgs/Waypoint.h>
#include <pedsim_msgs/AgentGroup.h>
#include <pedsim_msgs/Obstacle.h>
#include <pedsim_msgs/Wall.h>

namespace pedsim_msgs
{
template <class ContainerAllocator>
struct PedsimAgentsDataframe_
{
  typedef PedsimAgentsDataframe_<ContainerAllocator> Type;

  PedsimAgentsDataframe_()
    : header()
    , agent_states()
    , robot_states()
    , simulated_waypoints()
    , simulated_groups()
    , obstacles()
    , walls()  {
    }
  PedsimAgentsDataframe_(const ContainerAllocator& _alloc)
    : header(_alloc)
    , agent_states(_alloc)
    , robot_states(_alloc)
    , simulated_waypoints(_alloc)
    , simulated_groups(_alloc)
    , obstacles(_alloc)
    , walls(_alloc)  {
  (void)_alloc;
    }



   typedef  ::std_msgs::Header_<ContainerAllocator>  _header_type;
  _header_type header;

   typedef std::vector< ::pedsim_msgs::AgentState_<ContainerAllocator> , typename std::allocator_traits<ContainerAllocator>::template rebind_alloc< ::pedsim_msgs::AgentState_<ContainerAllocator> >> _agent_states_type;
  _agent_states_type agent_states;

   typedef std::vector< ::pedsim_msgs::RobotState_<ContainerAllocator> , typename std::allocator_traits<ContainerAllocator>::template rebind_alloc< ::pedsim_msgs::RobotState_<ContainerAllocator> >> _robot_states_type;
  _robot_states_type robot_states;

   typedef std::vector< ::pedsim_msgs::Waypoint_<ContainerAllocator> , typename std::allocator_traits<ContainerAllocator>::template rebind_alloc< ::pedsim_msgs::Waypoint_<ContainerAllocator> >> _simulated_waypoints_type;
  _simulated_waypoints_type simulated_waypoints;

   typedef std::vector< ::pedsim_msgs::AgentGroup_<ContainerAllocator> , typename std::allocator_traits<ContainerAllocator>::template rebind_alloc< ::pedsim_msgs::AgentGroup_<ContainerAllocator> >> _simulated_groups_type;
  _simulated_groups_type simulated_groups;

   typedef std::vector< ::pedsim_msgs::Obstacle_<ContainerAllocator> , typename std::allocator_traits<ContainerAllocator>::template rebind_alloc< ::pedsim_msgs::Obstacle_<ContainerAllocator> >> _obstacles_type;
  _obstacles_type obstacles;

   typedef std::vector< ::pedsim_msgs::Wall_<ContainerAllocator> , typename std::allocator_traits<ContainerAllocator>::template rebind_alloc< ::pedsim_msgs::Wall_<ContainerAllocator> >> _walls_type;
  _walls_type walls;





  typedef boost::shared_ptr< ::pedsim_msgs::PedsimAgentsDataframe_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::pedsim_msgs::PedsimAgentsDataframe_<ContainerAllocator> const> ConstPtr;

}; // struct PedsimAgentsDataframe_

typedef ::pedsim_msgs::PedsimAgentsDataframe_<std::allocator<void> > PedsimAgentsDataframe;

typedef boost::shared_ptr< ::pedsim_msgs::PedsimAgentsDataframe > PedsimAgentsDataframePtr;
typedef boost::shared_ptr< ::pedsim_msgs::PedsimAgentsDataframe const> PedsimAgentsDataframeConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::pedsim_msgs::PedsimAgentsDataframe_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::pedsim_msgs::PedsimAgentsDataframe_<ContainerAllocator> >::stream(s, "", v);
return s;
}


template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator==(const ::pedsim_msgs::PedsimAgentsDataframe_<ContainerAllocator1> & lhs, const ::pedsim_msgs::PedsimAgentsDataframe_<ContainerAllocator2> & rhs)
{
  return lhs.header == rhs.header &&
    lhs.agent_states == rhs.agent_states &&
    lhs.robot_states == rhs.robot_states &&
    lhs.simulated_waypoints == rhs.simulated_waypoints &&
    lhs.simulated_groups == rhs.simulated_groups &&
    lhs.obstacles == rhs.obstacles &&
    lhs.walls == rhs.walls;
}

template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator!=(const ::pedsim_msgs::PedsimAgentsDataframe_<ContainerAllocator1> & lhs, const ::pedsim_msgs::PedsimAgentsDataframe_<ContainerAllocator2> & rhs)
{
  return !(lhs == rhs);
}


} // namespace pedsim_msgs

namespace ros
{
namespace message_traits
{





template <class ContainerAllocator>
struct IsMessage< ::pedsim_msgs::PedsimAgentsDataframe_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::pedsim_msgs::PedsimAgentsDataframe_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::pedsim_msgs::PedsimAgentsDataframe_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::pedsim_msgs::PedsimAgentsDataframe_<ContainerAllocator> const>
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::pedsim_msgs::PedsimAgentsDataframe_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::pedsim_msgs::PedsimAgentsDataframe_<ContainerAllocator> const>
  : TrueType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::pedsim_msgs::PedsimAgentsDataframe_<ContainerAllocator> >
{
  static const char* value()
  {
    return "c051cf49747f6875eb7bd2af8dc2ea06";
  }

  static const char* value(const ::pedsim_msgs::PedsimAgentsDataframe_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0xc051cf49747f6875ULL;
  static const uint64_t static_value2 = 0xeb7bd2af8dc2ea06ULL;
};

template<class ContainerAllocator>
struct DataType< ::pedsim_msgs::PedsimAgentsDataframe_<ContainerAllocator> >
{
  static const char* value()
  {
    return "pedsim_msgs/PedsimAgentsDataframe";
  }

  static const char* value(const ::pedsim_msgs::PedsimAgentsDataframe_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::pedsim_msgs::PedsimAgentsDataframe_<ContainerAllocator> >
{
  static const char* value()
  {
    return "Header header\n"
"pedsim_msgs/AgentState[] agent_states\n"
"pedsim_msgs/RobotState[] robot_states\n"
"pedsim_msgs/Waypoint[] simulated_waypoints\n"
"pedsim_msgs/AgentGroup[] simulated_groups\n"
"pedsim_msgs/Obstacle[] obstacles\n"
"pedsim_msgs/Wall[] walls\n"
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
"================================================================================\n"
"MSG: pedsim_msgs/RobotState\n"
"string name\n"
"geometry_msgs/Pose pose\n"
"geometry_msgs/Twist twist\n"
"================================================================================\n"
"MSG: pedsim_msgs/Waypoint\n"
"int8 BHV_SIMPLE = 0\n"
"int8 BHV_SOURCE = 1\n"
"int8 BHV_SINK = 2\n"
"\n"
"string name\n"
"int8 type\n"
"int8 behavior\n"
"geometry_msgs/Point position\n"
"float32 radius\n"
"float32 interaction_radius\n"
"\n"
"================================================================================\n"
"MSG: pedsim_msgs/AgentGroup\n"
"Header header\n"
"string group_id\n"
"float64 age\n"
"string[] members\n"
"geometry_msgs/Pose center_of_mass\n"
"\n"
"================================================================================\n"
"MSG: pedsim_msgs/Obstacle\n"
"string name\n"
"# type can be one of the following: \"shelf\"\n"
"string type\n"
"geometry_msgs/Pose pose\n"
"float64 interaction_radius\n"
"string yaml_path\n"
"\n"
"================================================================================\n"
"MSG: pedsim_msgs/Wall\n"
"# A line obstacle in the simulator.\n"
"\n"
"geometry_msgs/Point start\n"
"geometry_msgs/Point end\n"
"uint8 layer\n"
;
  }

  static const char* value(const ::pedsim_msgs::PedsimAgentsDataframe_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::pedsim_msgs::PedsimAgentsDataframe_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.header);
      stream.next(m.agent_states);
      stream.next(m.robot_states);
      stream.next(m.simulated_waypoints);
      stream.next(m.simulated_groups);
      stream.next(m.obstacles);
      stream.next(m.walls);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct PedsimAgentsDataframe_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::pedsim_msgs::PedsimAgentsDataframe_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::pedsim_msgs::PedsimAgentsDataframe_<ContainerAllocator>& v)
  {
    s << indent << "header: ";
    s << std::endl;
    Printer< ::std_msgs::Header_<ContainerAllocator> >::stream(s, indent + "  ", v.header);
    s << indent << "agent_states[]" << std::endl;
    for (size_t i = 0; i < v.agent_states.size(); ++i)
    {
      s << indent << "  agent_states[" << i << "]: ";
      s << std::endl;
      s << indent;
      Printer< ::pedsim_msgs::AgentState_<ContainerAllocator> >::stream(s, indent + "    ", v.agent_states[i]);
    }
    s << indent << "robot_states[]" << std::endl;
    for (size_t i = 0; i < v.robot_states.size(); ++i)
    {
      s << indent << "  robot_states[" << i << "]: ";
      s << std::endl;
      s << indent;
      Printer< ::pedsim_msgs::RobotState_<ContainerAllocator> >::stream(s, indent + "    ", v.robot_states[i]);
    }
    s << indent << "simulated_waypoints[]" << std::endl;
    for (size_t i = 0; i < v.simulated_waypoints.size(); ++i)
    {
      s << indent << "  simulated_waypoints[" << i << "]: ";
      s << std::endl;
      s << indent;
      Printer< ::pedsim_msgs::Waypoint_<ContainerAllocator> >::stream(s, indent + "    ", v.simulated_waypoints[i]);
    }
    s << indent << "simulated_groups[]" << std::endl;
    for (size_t i = 0; i < v.simulated_groups.size(); ++i)
    {
      s << indent << "  simulated_groups[" << i << "]: ";
      s << std::endl;
      s << indent;
      Printer< ::pedsim_msgs::AgentGroup_<ContainerAllocator> >::stream(s, indent + "    ", v.simulated_groups[i]);
    }
    s << indent << "obstacles[]" << std::endl;
    for (size_t i = 0; i < v.obstacles.size(); ++i)
    {
      s << indent << "  obstacles[" << i << "]: ";
      s << std::endl;
      s << indent;
      Printer< ::pedsim_msgs::Obstacle_<ContainerAllocator> >::stream(s, indent + "    ", v.obstacles[i]);
    }
    s << indent << "walls[]" << std::endl;
    for (size_t i = 0; i < v.walls.size(); ++i)
    {
      s << indent << "  walls[" << i << "]: ";
      s << std::endl;
      s << indent;
      Printer< ::pedsim_msgs::Wall_<ContainerAllocator> >::stream(s, indent + "    ", v.walls[i]);
    }
  }
};

} // namespace message_operations
} // namespace ros

#endif // PEDSIM_MSGS_MESSAGE_PEDSIMAGENTSDATAFRAME_H
