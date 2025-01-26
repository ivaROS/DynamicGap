// Generated by gencpp from file spencer_tracking_msgs/TrackedGroups.msg
// DO NOT EDIT!


#ifndef SPENCER_TRACKING_MSGS_MESSAGE_TRACKEDGROUPS_H
#define SPENCER_TRACKING_MSGS_MESSAGE_TRACKEDGROUPS_H


#include <string>
#include <vector>
#include <memory>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>

#include <std_msgs/Header.h>
#include <spencer_tracking_msgs/TrackedGroup.h>

namespace spencer_tracking_msgs
{
template <class ContainerAllocator>
struct TrackedGroups_
{
  typedef TrackedGroups_<ContainerAllocator> Type;

  TrackedGroups_()
    : header()
    , groups()  {
    }
  TrackedGroups_(const ContainerAllocator& _alloc)
    : header(_alloc)
    , groups(_alloc)  {
  (void)_alloc;
    }



   typedef  ::std_msgs::Header_<ContainerAllocator>  _header_type;
  _header_type header;

   typedef std::vector< ::spencer_tracking_msgs::TrackedGroup_<ContainerAllocator> , typename std::allocator_traits<ContainerAllocator>::template rebind_alloc< ::spencer_tracking_msgs::TrackedGroup_<ContainerAllocator> >> _groups_type;
  _groups_type groups;





  typedef boost::shared_ptr< ::spencer_tracking_msgs::TrackedGroups_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::spencer_tracking_msgs::TrackedGroups_<ContainerAllocator> const> ConstPtr;

}; // struct TrackedGroups_

typedef ::spencer_tracking_msgs::TrackedGroups_<std::allocator<void> > TrackedGroups;

typedef boost::shared_ptr< ::spencer_tracking_msgs::TrackedGroups > TrackedGroupsPtr;
typedef boost::shared_ptr< ::spencer_tracking_msgs::TrackedGroups const> TrackedGroupsConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::spencer_tracking_msgs::TrackedGroups_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::spencer_tracking_msgs::TrackedGroups_<ContainerAllocator> >::stream(s, "", v);
return s;
}


template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator==(const ::spencer_tracking_msgs::TrackedGroups_<ContainerAllocator1> & lhs, const ::spencer_tracking_msgs::TrackedGroups_<ContainerAllocator2> & rhs)
{
  return lhs.header == rhs.header &&
    lhs.groups == rhs.groups;
}

template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator!=(const ::spencer_tracking_msgs::TrackedGroups_<ContainerAllocator1> & lhs, const ::spencer_tracking_msgs::TrackedGroups_<ContainerAllocator2> & rhs)
{
  return !(lhs == rhs);
}


} // namespace spencer_tracking_msgs

namespace ros
{
namespace message_traits
{





template <class ContainerAllocator>
struct IsMessage< ::spencer_tracking_msgs::TrackedGroups_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::spencer_tracking_msgs::TrackedGroups_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::spencer_tracking_msgs::TrackedGroups_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::spencer_tracking_msgs::TrackedGroups_<ContainerAllocator> const>
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::spencer_tracking_msgs::TrackedGroups_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::spencer_tracking_msgs::TrackedGroups_<ContainerAllocator> const>
  : TrueType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::spencer_tracking_msgs::TrackedGroups_<ContainerAllocator> >
{
  static const char* value()
  {
    return "6f0c24e3b3868fbd7c8271e4fd772fda";
  }

  static const char* value(const ::spencer_tracking_msgs::TrackedGroups_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0x6f0c24e3b3868fbdULL;
  static const uint64_t static_value2 = 0x7c8271e4fd772fdaULL;
};

template<class ContainerAllocator>
struct DataType< ::spencer_tracking_msgs::TrackedGroups_<ContainerAllocator> >
{
  static const char* value()
  {
    return "spencer_tracking_msgs/TrackedGroups";
  }

  static const char* value(const ::spencer_tracking_msgs::TrackedGroups_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::spencer_tracking_msgs::TrackedGroups_<ContainerAllocator> >
{
  static const char* value()
  {
    return "# Message with all currently tracked groups \n"
"#\n"
"\n"
"Header              header      # Header containing timestamp etc. of this message\n"
"TrackedGroup[]      groups      # All groups that are currently being tracked\n"
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
"MSG: spencer_tracking_msgs/TrackedGroup\n"
"# Message defining a tracked group\n"
"#\n"
"\n"
"string      group_id        # unique identifier of the target, consistent over time\n"
"\n"
"duration    age             # age of the group\n"
"\n"
"geometry_msgs/PoseWithCovariance    centerOfGravity   # mean and covariance of the group (calculated from its person tracks)\n"
"\n"
"string[]    track_ids       # IDs of the tracked persons in this group. See srl_tracking_msgs/TrackedPersons\n"
"================================================================================\n"
"MSG: geometry_msgs/PoseWithCovariance\n"
"# This represents a pose in free space with uncertainty.\n"
"\n"
"Pose pose\n"
"\n"
"# Row-major representation of the 6x6 covariance matrix\n"
"# The orientation parameters use a fixed-axis representation.\n"
"# In order, the parameters are:\n"
"# (x, y, z, rotation about X axis, rotation about Y axis, rotation about Z axis)\n"
"float64[36] covariance\n"
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
;
  }

  static const char* value(const ::spencer_tracking_msgs::TrackedGroups_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::spencer_tracking_msgs::TrackedGroups_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.header);
      stream.next(m.groups);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct TrackedGroups_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::spencer_tracking_msgs::TrackedGroups_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::spencer_tracking_msgs::TrackedGroups_<ContainerAllocator>& v)
  {
    s << indent << "header: ";
    s << std::endl;
    Printer< ::std_msgs::Header_<ContainerAllocator> >::stream(s, indent + "  ", v.header);
    s << indent << "groups[]" << std::endl;
    for (size_t i = 0; i < v.groups.size(); ++i)
    {
      s << indent << "  groups[" << i << "]: ";
      s << std::endl;
      s << indent;
      Printer< ::spencer_tracking_msgs::TrackedGroup_<ContainerAllocator> >::stream(s, indent + "    ", v.groups[i]);
    }
  }
};

} // namespace message_operations
} // namespace ros

#endif // SPENCER_TRACKING_MSGS_MESSAGE_TRACKEDGROUPS_H
