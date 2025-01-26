// Generated by gencpp from file mbf_msgs/GetPathActionResult.msg
// DO NOT EDIT!


#ifndef MBF_MSGS_MESSAGE_GETPATHACTIONRESULT_H
#define MBF_MSGS_MESSAGE_GETPATHACTIONRESULT_H


#include <string>
#include <vector>
#include <memory>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>

#include <std_msgs/Header.h>
#include <actionlib_msgs/GoalStatus.h>
#include <mbf_msgs/GetPathResult.h>

namespace mbf_msgs
{
template <class ContainerAllocator>
struct GetPathActionResult_
{
  typedef GetPathActionResult_<ContainerAllocator> Type;

  GetPathActionResult_()
    : header()
    , status()
    , result()  {
    }
  GetPathActionResult_(const ContainerAllocator& _alloc)
    : header(_alloc)
    , status(_alloc)
    , result(_alloc)  {
  (void)_alloc;
    }



   typedef  ::std_msgs::Header_<ContainerAllocator>  _header_type;
  _header_type header;

   typedef  ::actionlib_msgs::GoalStatus_<ContainerAllocator>  _status_type;
  _status_type status;

   typedef  ::mbf_msgs::GetPathResult_<ContainerAllocator>  _result_type;
  _result_type result;





  typedef boost::shared_ptr< ::mbf_msgs::GetPathActionResult_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::mbf_msgs::GetPathActionResult_<ContainerAllocator> const> ConstPtr;

}; // struct GetPathActionResult_

typedef ::mbf_msgs::GetPathActionResult_<std::allocator<void> > GetPathActionResult;

typedef boost::shared_ptr< ::mbf_msgs::GetPathActionResult > GetPathActionResultPtr;
typedef boost::shared_ptr< ::mbf_msgs::GetPathActionResult const> GetPathActionResultConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::mbf_msgs::GetPathActionResult_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::mbf_msgs::GetPathActionResult_<ContainerAllocator> >::stream(s, "", v);
return s;
}


template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator==(const ::mbf_msgs::GetPathActionResult_<ContainerAllocator1> & lhs, const ::mbf_msgs::GetPathActionResult_<ContainerAllocator2> & rhs)
{
  return lhs.header == rhs.header &&
    lhs.status == rhs.status &&
    lhs.result == rhs.result;
}

template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator!=(const ::mbf_msgs::GetPathActionResult_<ContainerAllocator1> & lhs, const ::mbf_msgs::GetPathActionResult_<ContainerAllocator2> & rhs)
{
  return !(lhs == rhs);
}


} // namespace mbf_msgs

namespace ros
{
namespace message_traits
{





template <class ContainerAllocator>
struct IsMessage< ::mbf_msgs::GetPathActionResult_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::mbf_msgs::GetPathActionResult_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::mbf_msgs::GetPathActionResult_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::mbf_msgs::GetPathActionResult_<ContainerAllocator> const>
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::mbf_msgs::GetPathActionResult_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::mbf_msgs::GetPathActionResult_<ContainerAllocator> const>
  : TrueType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::mbf_msgs::GetPathActionResult_<ContainerAllocator> >
{
  static const char* value()
  {
    return "d67aca1f3dee54d1f090431dcebfe886";
  }

  static const char* value(const ::mbf_msgs::GetPathActionResult_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0xd67aca1f3dee54d1ULL;
  static const uint64_t static_value2 = 0xf090431dcebfe886ULL;
};

template<class ContainerAllocator>
struct DataType< ::mbf_msgs::GetPathActionResult_<ContainerAllocator> >
{
  static const char* value()
  {
    return "mbf_msgs/GetPathActionResult";
  }

  static const char* value(const ::mbf_msgs::GetPathActionResult_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::mbf_msgs::GetPathActionResult_<ContainerAllocator> >
{
  static const char* value()
  {
    return "# ====== DO NOT MODIFY! AUTOGENERATED FROM AN ACTION DEFINITION ======\n"
"\n"
"Header header\n"
"actionlib_msgs/GoalStatus status\n"
"GetPathResult result\n"
"\n"
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
"MSG: actionlib_msgs/GoalStatus\n"
"GoalID goal_id\n"
"uint8 status\n"
"uint8 PENDING         = 0   # The goal has yet to be processed by the action server\n"
"uint8 ACTIVE          = 1   # The goal is currently being processed by the action server\n"
"uint8 PREEMPTED       = 2   # The goal received a cancel request after it started executing\n"
"                            #   and has since completed its execution (Terminal State)\n"
"uint8 SUCCEEDED       = 3   # The goal was achieved successfully by the action server (Terminal State)\n"
"uint8 ABORTED         = 4   # The goal was aborted during execution by the action server due\n"
"                            #    to some failure (Terminal State)\n"
"uint8 REJECTED        = 5   # The goal was rejected by the action server without being processed,\n"
"                            #    because the goal was unattainable or invalid (Terminal State)\n"
"uint8 PREEMPTING      = 6   # The goal received a cancel request after it started executing\n"
"                            #    and has not yet completed execution\n"
"uint8 RECALLING       = 7   # The goal received a cancel request before it started executing,\n"
"                            #    but the action server has not yet confirmed that the goal is canceled\n"
"uint8 RECALLED        = 8   # The goal received a cancel request before it started executing\n"
"                            #    and was successfully cancelled (Terminal State)\n"
"uint8 LOST            = 9   # An action client can determine that a goal is LOST. This should not be\n"
"                            #    sent over the wire by an action server\n"
"\n"
"#Allow for the user to associate a string with GoalStatus for debugging\n"
"string text\n"
"\n"
"\n"
"================================================================================\n"
"MSG: actionlib_msgs/GoalID\n"
"# The stamp should store the time at which this goal was requested.\n"
"# It is used by an action server when it tries to preempt all\n"
"# goals that were requested before a certain time\n"
"time stamp\n"
"\n"
"# The id provides a way to associate feedback and\n"
"# result message with specific goal requests. The id\n"
"# specified must be unique.\n"
"string id\n"
"\n"
"\n"
"================================================================================\n"
"MSG: mbf_msgs/GetPathResult\n"
"# ====== DO NOT MODIFY! AUTOGENERATED FROM AN ACTION DEFINITION ======\n"
"\n"
"# Predefined success codes:\n"
"uint8 SUCCESS           = 0\n"
"# 1..9 are reserved as plugin specific non-error results\n"
"\n"
"# Possible error codes:\n"
"uint8 FAILURE           = 50  # Unspecified failure, only used for old, non-mfb_core based plugins\n"
"uint8 CANCELED          = 51  # The action has been canceled by a action client\n"
"uint8 INVALID_START     = 52  # The start pose is inconsistent (e.g. frame is not valid)\n"
"uint8 INVALID_GOAL      = 53  # The goal pose is inconsistent (e.g. frame is not valid)\n"
"uint8 BLOCKED_START     = 54  # The start pose is in collision\n"
"uint8 BLOCKED_GOAL      = 55  # The goal pose is in collision\n"
"uint8 NO_PATH_FOUND     = 56\n"
"uint8 PAT_EXCEEDED      = 57\n"
"uint8 EMPTY_PATH        = 58\n"
"uint8 TF_ERROR          = 59\n"
"uint8 NOT_INITIALIZED   = 60\n"
"uint8 INVALID_PLUGIN    = 61\n"
"uint8 INTERNAL_ERROR    = 62\n"
"uint8 OUT_OF_MAP        = 63  # The start and / or the goal are outside the map\n"
"uint8 MAP_ERROR         = 64  # The map is not available or not running properly\n"
"uint8 STOPPED           = 65  # The planner execution has been stopped rigorously\n"
"\n"
"uint8 ERROR_RANGE_START = 50\n"
"uint8 ERROR_RANGE_END   = 99\n"
"\n"
"# 71..99 are reserved as plugin specific errors:\n"
"uint8 PLUGIN_ERROR_RANGE_START = 71\n"
"uint8 PLUGIN_ERROR_RANGE_END   = 99\n"
"\n"
"uint32 outcome\n"
"string message\n"
"\n"
"nav_msgs/Path path\n"
"\n"
"float64 cost\n"
"\n"
"\n"
"================================================================================\n"
"MSG: nav_msgs/Path\n"
"#An array of poses that represents a Path for a robot to follow\n"
"Header header\n"
"geometry_msgs/PoseStamped[] poses\n"
"\n"
"================================================================================\n"
"MSG: geometry_msgs/PoseStamped\n"
"# A Pose with reference coordinate frame and timestamp\n"
"Header header\n"
"Pose pose\n"
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

  static const char* value(const ::mbf_msgs::GetPathActionResult_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::mbf_msgs::GetPathActionResult_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.header);
      stream.next(m.status);
      stream.next(m.result);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct GetPathActionResult_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::mbf_msgs::GetPathActionResult_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::mbf_msgs::GetPathActionResult_<ContainerAllocator>& v)
  {
    s << indent << "header: ";
    s << std::endl;
    Printer< ::std_msgs::Header_<ContainerAllocator> >::stream(s, indent + "  ", v.header);
    s << indent << "status: ";
    s << std::endl;
    Printer< ::actionlib_msgs::GoalStatus_<ContainerAllocator> >::stream(s, indent + "  ", v.status);
    s << indent << "result: ";
    s << std::endl;
    Printer< ::mbf_msgs::GetPathResult_<ContainerAllocator> >::stream(s, indent + "  ", v.result);
  }
};

} // namespace message_operations
} // namespace ros

#endif // MBF_MSGS_MESSAGE_GETPATHACTIONRESULT_H
