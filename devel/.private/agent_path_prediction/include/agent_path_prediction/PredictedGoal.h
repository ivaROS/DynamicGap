// Generated by gencpp from file agent_path_prediction/PredictedGoal.msg
// DO NOT EDIT!


#ifndef AGENT_PATH_PREDICTION_MESSAGE_PREDICTEDGOAL_H
#define AGENT_PATH_PREDICTION_MESSAGE_PREDICTEDGOAL_H


#include <string>
#include <vector>
#include <memory>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>

#include <geometry_msgs/PoseStamped.h>

namespace agent_path_prediction
{
template <class ContainerAllocator>
struct PredictedGoal_
{
  typedef PredictedGoal_<ContainerAllocator> Type;

  PredictedGoal_()
    : changed(false)
    , goal()  {
    }
  PredictedGoal_(const ContainerAllocator& _alloc)
    : changed(false)
    , goal(_alloc)  {
  (void)_alloc;
    }



   typedef uint8_t _changed_type;
  _changed_type changed;

   typedef  ::geometry_msgs::PoseStamped_<ContainerAllocator>  _goal_type;
  _goal_type goal;





  typedef boost::shared_ptr< ::agent_path_prediction::PredictedGoal_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::agent_path_prediction::PredictedGoal_<ContainerAllocator> const> ConstPtr;

}; // struct PredictedGoal_

typedef ::agent_path_prediction::PredictedGoal_<std::allocator<void> > PredictedGoal;

typedef boost::shared_ptr< ::agent_path_prediction::PredictedGoal > PredictedGoalPtr;
typedef boost::shared_ptr< ::agent_path_prediction::PredictedGoal const> PredictedGoalConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::agent_path_prediction::PredictedGoal_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::agent_path_prediction::PredictedGoal_<ContainerAllocator> >::stream(s, "", v);
return s;
}


template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator==(const ::agent_path_prediction::PredictedGoal_<ContainerAllocator1> & lhs, const ::agent_path_prediction::PredictedGoal_<ContainerAllocator2> & rhs)
{
  return lhs.changed == rhs.changed &&
    lhs.goal == rhs.goal;
}

template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator!=(const ::agent_path_prediction::PredictedGoal_<ContainerAllocator1> & lhs, const ::agent_path_prediction::PredictedGoal_<ContainerAllocator2> & rhs)
{
  return !(lhs == rhs);
}


} // namespace agent_path_prediction

namespace ros
{
namespace message_traits
{





template <class ContainerAllocator>
struct IsMessage< ::agent_path_prediction::PredictedGoal_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::agent_path_prediction::PredictedGoal_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::agent_path_prediction::PredictedGoal_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::agent_path_prediction::PredictedGoal_<ContainerAllocator> const>
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::agent_path_prediction::PredictedGoal_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::agent_path_prediction::PredictedGoal_<ContainerAllocator> const>
  : FalseType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::agent_path_prediction::PredictedGoal_<ContainerAllocator> >
{
  static const char* value()
  {
    return "b1011011ba3cf8056b5f0d10375a839d";
  }

  static const char* value(const ::agent_path_prediction::PredictedGoal_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0xb1011011ba3cf805ULL;
  static const uint64_t static_value2 = 0x6b5f0d10375a839dULL;
};

template<class ContainerAllocator>
struct DataType< ::agent_path_prediction::PredictedGoal_<ContainerAllocator> >
{
  static const char* value()
  {
    return "agent_path_prediction/PredictedGoal";
  }

  static const char* value(const ::agent_path_prediction::PredictedGoal_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::agent_path_prediction::PredictedGoal_<ContainerAllocator> >
{
  static const char* value()
  {
    return "bool                        changed\n"
"geometry_msgs/PoseStamped   goal                                                       \n"
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
;
  }

  static const char* value(const ::agent_path_prediction::PredictedGoal_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::agent_path_prediction::PredictedGoal_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.changed);
      stream.next(m.goal);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct PredictedGoal_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::agent_path_prediction::PredictedGoal_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::agent_path_prediction::PredictedGoal_<ContainerAllocator>& v)
  {
    s << indent << "changed: ";
    Printer<uint8_t>::stream(s, indent + "  ", v.changed);
    s << indent << "goal: ";
    s << std::endl;
    Printer< ::geometry_msgs::PoseStamped_<ContainerAllocator> >::stream(s, indent + "  ", v.goal);
  }
};

} // namespace message_operations
} // namespace ros

#endif // AGENT_PATH_PREDICTION_MESSAGE_PREDICTEDGOAL_H
