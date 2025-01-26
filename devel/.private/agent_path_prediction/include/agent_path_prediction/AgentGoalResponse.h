// Generated by gencpp from file agent_path_prediction/AgentGoalResponse.msg
// DO NOT EDIT!


#ifndef AGENT_PATH_PREDICTION_MESSAGE_AGENTGOALRESPONSE_H
#define AGENT_PATH_PREDICTION_MESSAGE_AGENTGOALRESPONSE_H


#include <string>
#include <vector>
#include <memory>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>


namespace agent_path_prediction
{
template <class ContainerAllocator>
struct AgentGoalResponse_
{
  typedef AgentGoalResponse_<ContainerAllocator> Type;

  AgentGoalResponse_()
    : success(false)
    , message()  {
    }
  AgentGoalResponse_(const ContainerAllocator& _alloc)
    : success(false)
    , message(_alloc)  {
  (void)_alloc;
    }



   typedef uint8_t _success_type;
  _success_type success;

   typedef std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>> _message_type;
  _message_type message;





  typedef boost::shared_ptr< ::agent_path_prediction::AgentGoalResponse_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::agent_path_prediction::AgentGoalResponse_<ContainerAllocator> const> ConstPtr;

}; // struct AgentGoalResponse_

typedef ::agent_path_prediction::AgentGoalResponse_<std::allocator<void> > AgentGoalResponse;

typedef boost::shared_ptr< ::agent_path_prediction::AgentGoalResponse > AgentGoalResponsePtr;
typedef boost::shared_ptr< ::agent_path_prediction::AgentGoalResponse const> AgentGoalResponseConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::agent_path_prediction::AgentGoalResponse_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::agent_path_prediction::AgentGoalResponse_<ContainerAllocator> >::stream(s, "", v);
return s;
}


template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator==(const ::agent_path_prediction::AgentGoalResponse_<ContainerAllocator1> & lhs, const ::agent_path_prediction::AgentGoalResponse_<ContainerAllocator2> & rhs)
{
  return lhs.success == rhs.success &&
    lhs.message == rhs.message;
}

template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator!=(const ::agent_path_prediction::AgentGoalResponse_<ContainerAllocator1> & lhs, const ::agent_path_prediction::AgentGoalResponse_<ContainerAllocator2> & rhs)
{
  return !(lhs == rhs);
}


} // namespace agent_path_prediction

namespace ros
{
namespace message_traits
{





template <class ContainerAllocator>
struct IsMessage< ::agent_path_prediction::AgentGoalResponse_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::agent_path_prediction::AgentGoalResponse_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::agent_path_prediction::AgentGoalResponse_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::agent_path_prediction::AgentGoalResponse_<ContainerAllocator> const>
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::agent_path_prediction::AgentGoalResponse_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::agent_path_prediction::AgentGoalResponse_<ContainerAllocator> const>
  : FalseType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::agent_path_prediction::AgentGoalResponse_<ContainerAllocator> >
{
  static const char* value()
  {
    return "937c9679a518e3a18d831e57125ea522";
  }

  static const char* value(const ::agent_path_prediction::AgentGoalResponse_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0x937c9679a518e3a1ULL;
  static const uint64_t static_value2 = 0x8d831e57125ea522ULL;
};

template<class ContainerAllocator>
struct DataType< ::agent_path_prediction::AgentGoalResponse_<ContainerAllocator> >
{
  static const char* value()
  {
    return "agent_path_prediction/AgentGoalResponse";
  }

  static const char* value(const ::agent_path_prediction::AgentGoalResponse_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::agent_path_prediction::AgentGoalResponse_<ContainerAllocator> >
{
  static const char* value()
  {
    return "bool                         success\n"
"string                       message\n"
"\n"
;
  }

  static const char* value(const ::agent_path_prediction::AgentGoalResponse_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::agent_path_prediction::AgentGoalResponse_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.success);
      stream.next(m.message);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct AgentGoalResponse_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::agent_path_prediction::AgentGoalResponse_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::agent_path_prediction::AgentGoalResponse_<ContainerAllocator>& v)
  {
    s << indent << "success: ";
    Printer<uint8_t>::stream(s, indent + "  ", v.success);
    s << indent << "message: ";
    Printer<std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>>::stream(s, indent + "  ", v.message);
  }
};

} // namespace message_operations
} // namespace ros

#endif // AGENT_PATH_PREDICTION_MESSAGE_AGENTGOALRESPONSE_H
