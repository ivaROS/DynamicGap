// Generated by gencpp from file mbf_msgs/RecoveryGoal.msg
// DO NOT EDIT!


#ifndef MBF_MSGS_MESSAGE_RECOVERYGOAL_H
#define MBF_MSGS_MESSAGE_RECOVERYGOAL_H


#include <string>
#include <vector>
#include <memory>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>


namespace mbf_msgs
{
template <class ContainerAllocator>
struct RecoveryGoal_
{
  typedef RecoveryGoal_<ContainerAllocator> Type;

  RecoveryGoal_()
    : behavior()
    , concurrency_slot(0)  {
    }
  RecoveryGoal_(const ContainerAllocator& _alloc)
    : behavior(_alloc)
    , concurrency_slot(0)  {
  (void)_alloc;
    }



   typedef std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>> _behavior_type;
  _behavior_type behavior;

   typedef uint8_t _concurrency_slot_type;
  _concurrency_slot_type concurrency_slot;





  typedef boost::shared_ptr< ::mbf_msgs::RecoveryGoal_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::mbf_msgs::RecoveryGoal_<ContainerAllocator> const> ConstPtr;

}; // struct RecoveryGoal_

typedef ::mbf_msgs::RecoveryGoal_<std::allocator<void> > RecoveryGoal;

typedef boost::shared_ptr< ::mbf_msgs::RecoveryGoal > RecoveryGoalPtr;
typedef boost::shared_ptr< ::mbf_msgs::RecoveryGoal const> RecoveryGoalConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::mbf_msgs::RecoveryGoal_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::mbf_msgs::RecoveryGoal_<ContainerAllocator> >::stream(s, "", v);
return s;
}


template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator==(const ::mbf_msgs::RecoveryGoal_<ContainerAllocator1> & lhs, const ::mbf_msgs::RecoveryGoal_<ContainerAllocator2> & rhs)
{
  return lhs.behavior == rhs.behavior &&
    lhs.concurrency_slot == rhs.concurrency_slot;
}

template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator!=(const ::mbf_msgs::RecoveryGoal_<ContainerAllocator1> & lhs, const ::mbf_msgs::RecoveryGoal_<ContainerAllocator2> & rhs)
{
  return !(lhs == rhs);
}


} // namespace mbf_msgs

namespace ros
{
namespace message_traits
{





template <class ContainerAllocator>
struct IsMessage< ::mbf_msgs::RecoveryGoal_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::mbf_msgs::RecoveryGoal_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::mbf_msgs::RecoveryGoal_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::mbf_msgs::RecoveryGoal_<ContainerAllocator> const>
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::mbf_msgs::RecoveryGoal_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::mbf_msgs::RecoveryGoal_<ContainerAllocator> const>
  : FalseType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::mbf_msgs::RecoveryGoal_<ContainerAllocator> >
{
  static const char* value()
  {
    return "ce28884316a172b85e57b78a84014451";
  }

  static const char* value(const ::mbf_msgs::RecoveryGoal_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0xce28884316a172b8ULL;
  static const uint64_t static_value2 = 0x5e57b78a84014451ULL;
};

template<class ContainerAllocator>
struct DataType< ::mbf_msgs::RecoveryGoal_<ContainerAllocator> >
{
  static const char* value()
  {
    return "mbf_msgs/RecoveryGoal";
  }

  static const char* value(const ::mbf_msgs::RecoveryGoal_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::mbf_msgs::RecoveryGoal_<ContainerAllocator> >
{
  static const char* value()
  {
    return "# ====== DO NOT MODIFY! AUTOGENERATED FROM AN ACTION DEFINITION ======\n"
"# Run one of the recovery behavior listed on recovery_behaviors parameter\n"
"\n"
"string behavior\n"
"\n"
"# use different slots for concurrency\n"
"uint8 concurrency_slot\n"
"\n"
;
  }

  static const char* value(const ::mbf_msgs::RecoveryGoal_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::mbf_msgs::RecoveryGoal_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.behavior);
      stream.next(m.concurrency_slot);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct RecoveryGoal_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::mbf_msgs::RecoveryGoal_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::mbf_msgs::RecoveryGoal_<ContainerAllocator>& v)
  {
    s << indent << "behavior: ";
    Printer<std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>>::stream(s, indent + "  ", v.behavior);
    s << indent << "concurrency_slot: ";
    Printer<uint8_t>::stream(s, indent + "  ", v.concurrency_slot);
  }
};

} // namespace message_operations
} // namespace ros

#endif // MBF_MSGS_MESSAGE_RECOVERYGOAL_H
