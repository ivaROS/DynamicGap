// Generated by gencpp from file mbf_msgs/RecoveryAction.msg
// DO NOT EDIT!


#ifndef MBF_MSGS_MESSAGE_RECOVERYACTION_H
#define MBF_MSGS_MESSAGE_RECOVERYACTION_H


#include <string>
#include <vector>
#include <memory>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>

#include <mbf_msgs/RecoveryActionGoal.h>
#include <mbf_msgs/RecoveryActionResult.h>
#include <mbf_msgs/RecoveryActionFeedback.h>

namespace mbf_msgs
{
template <class ContainerAllocator>
struct RecoveryAction_
{
  typedef RecoveryAction_<ContainerAllocator> Type;

  RecoveryAction_()
    : action_goal()
    , action_result()
    , action_feedback()  {
    }
  RecoveryAction_(const ContainerAllocator& _alloc)
    : action_goal(_alloc)
    , action_result(_alloc)
    , action_feedback(_alloc)  {
  (void)_alloc;
    }



   typedef  ::mbf_msgs::RecoveryActionGoal_<ContainerAllocator>  _action_goal_type;
  _action_goal_type action_goal;

   typedef  ::mbf_msgs::RecoveryActionResult_<ContainerAllocator>  _action_result_type;
  _action_result_type action_result;

   typedef  ::mbf_msgs::RecoveryActionFeedback_<ContainerAllocator>  _action_feedback_type;
  _action_feedback_type action_feedback;





  typedef boost::shared_ptr< ::mbf_msgs::RecoveryAction_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::mbf_msgs::RecoveryAction_<ContainerAllocator> const> ConstPtr;

}; // struct RecoveryAction_

typedef ::mbf_msgs::RecoveryAction_<std::allocator<void> > RecoveryAction;

typedef boost::shared_ptr< ::mbf_msgs::RecoveryAction > RecoveryActionPtr;
typedef boost::shared_ptr< ::mbf_msgs::RecoveryAction const> RecoveryActionConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::mbf_msgs::RecoveryAction_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::mbf_msgs::RecoveryAction_<ContainerAllocator> >::stream(s, "", v);
return s;
}


template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator==(const ::mbf_msgs::RecoveryAction_<ContainerAllocator1> & lhs, const ::mbf_msgs::RecoveryAction_<ContainerAllocator2> & rhs)
{
  return lhs.action_goal == rhs.action_goal &&
    lhs.action_result == rhs.action_result &&
    lhs.action_feedback == rhs.action_feedback;
}

template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator!=(const ::mbf_msgs::RecoveryAction_<ContainerAllocator1> & lhs, const ::mbf_msgs::RecoveryAction_<ContainerAllocator2> & rhs)
{
  return !(lhs == rhs);
}


} // namespace mbf_msgs

namespace ros
{
namespace message_traits
{





template <class ContainerAllocator>
struct IsMessage< ::mbf_msgs::RecoveryAction_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::mbf_msgs::RecoveryAction_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::mbf_msgs::RecoveryAction_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::mbf_msgs::RecoveryAction_<ContainerAllocator> const>
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::mbf_msgs::RecoveryAction_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::mbf_msgs::RecoveryAction_<ContainerAllocator> const>
  : FalseType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::mbf_msgs::RecoveryAction_<ContainerAllocator> >
{
  static const char* value()
  {
    return "7f0b73a03d963f251558d4e79a23265a";
  }

  static const char* value(const ::mbf_msgs::RecoveryAction_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0x7f0b73a03d963f25ULL;
  static const uint64_t static_value2 = 0x1558d4e79a23265aULL;
};

template<class ContainerAllocator>
struct DataType< ::mbf_msgs::RecoveryAction_<ContainerAllocator> >
{
  static const char* value()
  {
    return "mbf_msgs/RecoveryAction";
  }

  static const char* value(const ::mbf_msgs::RecoveryAction_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::mbf_msgs::RecoveryAction_<ContainerAllocator> >
{
  static const char* value()
  {
    return "# ====== DO NOT MODIFY! AUTOGENERATED FROM AN ACTION DEFINITION ======\n"
"\n"
"RecoveryActionGoal action_goal\n"
"RecoveryActionResult action_result\n"
"RecoveryActionFeedback action_feedback\n"
"\n"
"================================================================================\n"
"MSG: mbf_msgs/RecoveryActionGoal\n"
"# ====== DO NOT MODIFY! AUTOGENERATED FROM AN ACTION DEFINITION ======\n"
"\n"
"Header header\n"
"actionlib_msgs/GoalID goal_id\n"
"RecoveryGoal goal\n"
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
"MSG: mbf_msgs/RecoveryGoal\n"
"# ====== DO NOT MODIFY! AUTOGENERATED FROM AN ACTION DEFINITION ======\n"
"# Run one of the recovery behavior listed on recovery_behaviors parameter\n"
"\n"
"string behavior\n"
"\n"
"# use different slots for concurrency\n"
"uint8 concurrency_slot\n"
"\n"
"\n"
"================================================================================\n"
"MSG: mbf_msgs/RecoveryActionResult\n"
"# ====== DO NOT MODIFY! AUTOGENERATED FROM AN ACTION DEFINITION ======\n"
"\n"
"Header header\n"
"actionlib_msgs/GoalStatus status\n"
"RecoveryResult result\n"
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
"MSG: mbf_msgs/RecoveryResult\n"
"# ====== DO NOT MODIFY! AUTOGENERATED FROM AN ACTION DEFINITION ======\n"
"\n"
"# Predefined success codes:\n"
"uint8 SUCCESS           = 0\n"
"\n"
"# Possible server codes:\n"
"uint8 FAILURE           = 150\n"
"uint8 CANCELED          = 151\n"
"uint8 PAT_EXCEEDED      = 152\n"
"uint8 TF_ERROR          = 153\n"
"uint8 NOT_INITIALIZED   = 154\n"
"uint8 INVALID_PLUGIN    = 155\n"
"uint8 INTERNAL_ERROR    = 156\n"
"uint8 STOPPED           = 157  # The recovery behaviour execution has been stopped rigorously\n"
"uint8 IMPASSABLE        = 158  # Further execution would lead to a collision\n"
"\n"
"uint8 ERROR_RANGE_START = 150\n"
"uint8 ERROR_RANGE_END   = 199\n"
"\n"
"# 171..199 are reserved as plugin specific errors:\n"
"uint8 PLUGIN_ERROR_RANGE_START = 171\n"
"uint8 PLUGIN_ERROR_RANGE_END   = 199\n"
"\n"
"uint32 outcome\n"
"string message\n"
"string used_plugin\n"
"\n"
"\n"
"================================================================================\n"
"MSG: mbf_msgs/RecoveryActionFeedback\n"
"# ====== DO NOT MODIFY! AUTOGENERATED FROM AN ACTION DEFINITION ======\n"
"\n"
"Header header\n"
"actionlib_msgs/GoalStatus status\n"
"RecoveryFeedback feedback\n"
"\n"
"================================================================================\n"
"MSG: mbf_msgs/RecoveryFeedback\n"
"# ====== DO NOT MODIFY! AUTOGENERATED FROM AN ACTION DEFINITION ======\n"
"\n"
;
  }

  static const char* value(const ::mbf_msgs::RecoveryAction_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::mbf_msgs::RecoveryAction_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.action_goal);
      stream.next(m.action_result);
      stream.next(m.action_feedback);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct RecoveryAction_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::mbf_msgs::RecoveryAction_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::mbf_msgs::RecoveryAction_<ContainerAllocator>& v)
  {
    s << indent << "action_goal: ";
    s << std::endl;
    Printer< ::mbf_msgs::RecoveryActionGoal_<ContainerAllocator> >::stream(s, indent + "  ", v.action_goal);
    s << indent << "action_result: ";
    s << std::endl;
    Printer< ::mbf_msgs::RecoveryActionResult_<ContainerAllocator> >::stream(s, indent + "  ", v.action_result);
    s << indent << "action_feedback: ";
    s << std::endl;
    Printer< ::mbf_msgs::RecoveryActionFeedback_<ContainerAllocator> >::stream(s, indent + "  ", v.action_feedback);
  }
};

} // namespace message_operations
} // namespace ros

#endif // MBF_MSGS_MESSAGE_RECOVERYACTION_H
