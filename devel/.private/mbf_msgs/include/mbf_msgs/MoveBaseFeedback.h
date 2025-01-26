// Generated by gencpp from file mbf_msgs/MoveBaseFeedback.msg
// DO NOT EDIT!


#ifndef MBF_MSGS_MESSAGE_MOVEBASEFEEDBACK_H
#define MBF_MSGS_MESSAGE_MOVEBASEFEEDBACK_H


#include <string>
#include <vector>
#include <memory>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>

#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>

namespace mbf_msgs
{
template <class ContainerAllocator>
struct MoveBaseFeedback_
{
  typedef MoveBaseFeedback_<ContainerAllocator> Type;

  MoveBaseFeedback_()
    : outcome(0)
    , message()
    , dist_to_goal(0.0)
    , angle_to_goal(0.0)
    , current_pose()
    , last_cmd_vel()  {
    }
  MoveBaseFeedback_(const ContainerAllocator& _alloc)
    : outcome(0)
    , message(_alloc)
    , dist_to_goal(0.0)
    , angle_to_goal(0.0)
    , current_pose(_alloc)
    , last_cmd_vel(_alloc)  {
  (void)_alloc;
    }



   typedef uint32_t _outcome_type;
  _outcome_type outcome;

   typedef std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>> _message_type;
  _message_type message;

   typedef float _dist_to_goal_type;
  _dist_to_goal_type dist_to_goal;

   typedef float _angle_to_goal_type;
  _angle_to_goal_type angle_to_goal;

   typedef  ::geometry_msgs::PoseStamped_<ContainerAllocator>  _current_pose_type;
  _current_pose_type current_pose;

   typedef  ::geometry_msgs::TwistStamped_<ContainerAllocator>  _last_cmd_vel_type;
  _last_cmd_vel_type last_cmd_vel;





  typedef boost::shared_ptr< ::mbf_msgs::MoveBaseFeedback_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::mbf_msgs::MoveBaseFeedback_<ContainerAllocator> const> ConstPtr;

}; // struct MoveBaseFeedback_

typedef ::mbf_msgs::MoveBaseFeedback_<std::allocator<void> > MoveBaseFeedback;

typedef boost::shared_ptr< ::mbf_msgs::MoveBaseFeedback > MoveBaseFeedbackPtr;
typedef boost::shared_ptr< ::mbf_msgs::MoveBaseFeedback const> MoveBaseFeedbackConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::mbf_msgs::MoveBaseFeedback_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::mbf_msgs::MoveBaseFeedback_<ContainerAllocator> >::stream(s, "", v);
return s;
}


template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator==(const ::mbf_msgs::MoveBaseFeedback_<ContainerAllocator1> & lhs, const ::mbf_msgs::MoveBaseFeedback_<ContainerAllocator2> & rhs)
{
  return lhs.outcome == rhs.outcome &&
    lhs.message == rhs.message &&
    lhs.dist_to_goal == rhs.dist_to_goal &&
    lhs.angle_to_goal == rhs.angle_to_goal &&
    lhs.current_pose == rhs.current_pose &&
    lhs.last_cmd_vel == rhs.last_cmd_vel;
}

template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator!=(const ::mbf_msgs::MoveBaseFeedback_<ContainerAllocator1> & lhs, const ::mbf_msgs::MoveBaseFeedback_<ContainerAllocator2> & rhs)
{
  return !(lhs == rhs);
}


} // namespace mbf_msgs

namespace ros
{
namespace message_traits
{





template <class ContainerAllocator>
struct IsMessage< ::mbf_msgs::MoveBaseFeedback_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::mbf_msgs::MoveBaseFeedback_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::mbf_msgs::MoveBaseFeedback_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::mbf_msgs::MoveBaseFeedback_<ContainerAllocator> const>
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::mbf_msgs::MoveBaseFeedback_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::mbf_msgs::MoveBaseFeedback_<ContainerAllocator> const>
  : FalseType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::mbf_msgs::MoveBaseFeedback_<ContainerAllocator> >
{
  static const char* value()
  {
    return "1b30e381361670e9521046df439847e2";
  }

  static const char* value(const ::mbf_msgs::MoveBaseFeedback_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0x1b30e381361670e9ULL;
  static const uint64_t static_value2 = 0x521046df439847e2ULL;
};

template<class ContainerAllocator>
struct DataType< ::mbf_msgs::MoveBaseFeedback_<ContainerAllocator> >
{
  static const char* value()
  {
    return "mbf_msgs/MoveBaseFeedback";
  }

  static const char* value(const ::mbf_msgs::MoveBaseFeedback_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::mbf_msgs::MoveBaseFeedback_<ContainerAllocator> >
{
  static const char* value()
  {
    return "# ====== DO NOT MODIFY! AUTOGENERATED FROM AN ACTION DEFINITION ======\n"
"\n"
"# Outcome of most recent controller cycle. Same values as in MoveBase or ExePath result\n"
"uint32 outcome\n"
"string message\n"
"\n"
"float32 dist_to_goal\n"
"float32 angle_to_goal\n"
"geometry_msgs/PoseStamped current_pose\n"
"geometry_msgs/TwistStamped last_cmd_vel  # last command calculated by the controller\n"
"\n"
"\n"
"================================================================================\n"
"MSG: geometry_msgs/PoseStamped\n"
"# A Pose with reference coordinate frame and timestamp\n"
"Header header\n"
"Pose pose\n"
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
"MSG: geometry_msgs/TwistStamped\n"
"# A twist with reference coordinate frame and timestamp\n"
"Header header\n"
"Twist twist\n"
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

  static const char* value(const ::mbf_msgs::MoveBaseFeedback_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::mbf_msgs::MoveBaseFeedback_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.outcome);
      stream.next(m.message);
      stream.next(m.dist_to_goal);
      stream.next(m.angle_to_goal);
      stream.next(m.current_pose);
      stream.next(m.last_cmd_vel);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct MoveBaseFeedback_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::mbf_msgs::MoveBaseFeedback_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::mbf_msgs::MoveBaseFeedback_<ContainerAllocator>& v)
  {
    s << indent << "outcome: ";
    Printer<uint32_t>::stream(s, indent + "  ", v.outcome);
    s << indent << "message: ";
    Printer<std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>>::stream(s, indent + "  ", v.message);
    s << indent << "dist_to_goal: ";
    Printer<float>::stream(s, indent + "  ", v.dist_to_goal);
    s << indent << "angle_to_goal: ";
    Printer<float>::stream(s, indent + "  ", v.angle_to_goal);
    s << indent << "current_pose: ";
    s << std::endl;
    Printer< ::geometry_msgs::PoseStamped_<ContainerAllocator> >::stream(s, indent + "  ", v.current_pose);
    s << indent << "last_cmd_vel: ";
    s << std::endl;
    Printer< ::geometry_msgs::TwistStamped_<ContainerAllocator> >::stream(s, indent + "  ", v.last_cmd_vel);
  }
};

} // namespace message_operations
} // namespace ros

#endif // MBF_MSGS_MESSAGE_MOVEBASEFEEDBACK_H
