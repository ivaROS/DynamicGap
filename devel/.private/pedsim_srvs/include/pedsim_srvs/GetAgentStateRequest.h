// Generated by gencpp from file pedsim_srvs/GetAgentStateRequest.msg
// DO NOT EDIT!


#ifndef PEDSIM_SRVS_MESSAGE_GETAGENTSTATEREQUEST_H
#define PEDSIM_SRVS_MESSAGE_GETAGENTSTATEREQUEST_H


#include <string>
#include <vector>
#include <memory>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>


namespace pedsim_srvs
{
template <class ContainerAllocator>
struct GetAgentStateRequest_
{
  typedef GetAgentStateRequest_<ContainerAllocator> Type;

  GetAgentStateRequest_()
    : agent_id()  {
    }
  GetAgentStateRequest_(const ContainerAllocator& _alloc)
    : agent_id(_alloc)  {
  (void)_alloc;
    }



   typedef std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>> _agent_id_type;
  _agent_id_type agent_id;





  typedef boost::shared_ptr< ::pedsim_srvs::GetAgentStateRequest_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::pedsim_srvs::GetAgentStateRequest_<ContainerAllocator> const> ConstPtr;

}; // struct GetAgentStateRequest_

typedef ::pedsim_srvs::GetAgentStateRequest_<std::allocator<void> > GetAgentStateRequest;

typedef boost::shared_ptr< ::pedsim_srvs::GetAgentStateRequest > GetAgentStateRequestPtr;
typedef boost::shared_ptr< ::pedsim_srvs::GetAgentStateRequest const> GetAgentStateRequestConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::pedsim_srvs::GetAgentStateRequest_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::pedsim_srvs::GetAgentStateRequest_<ContainerAllocator> >::stream(s, "", v);
return s;
}


template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator==(const ::pedsim_srvs::GetAgentStateRequest_<ContainerAllocator1> & lhs, const ::pedsim_srvs::GetAgentStateRequest_<ContainerAllocator2> & rhs)
{
  return lhs.agent_id == rhs.agent_id;
}

template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator!=(const ::pedsim_srvs::GetAgentStateRequest_<ContainerAllocator1> & lhs, const ::pedsim_srvs::GetAgentStateRequest_<ContainerAllocator2> & rhs)
{
  return !(lhs == rhs);
}


} // namespace pedsim_srvs

namespace ros
{
namespace message_traits
{





template <class ContainerAllocator>
struct IsMessage< ::pedsim_srvs::GetAgentStateRequest_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::pedsim_srvs::GetAgentStateRequest_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::pedsim_srvs::GetAgentStateRequest_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::pedsim_srvs::GetAgentStateRequest_<ContainerAllocator> const>
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::pedsim_srvs::GetAgentStateRequest_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::pedsim_srvs::GetAgentStateRequest_<ContainerAllocator> const>
  : FalseType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::pedsim_srvs::GetAgentStateRequest_<ContainerAllocator> >
{
  static const char* value()
  {
    return "4c5c0d6a6da5417447cea232216f8416";
  }

  static const char* value(const ::pedsim_srvs::GetAgentStateRequest_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0x4c5c0d6a6da54174ULL;
  static const uint64_t static_value2 = 0x47cea232216f8416ULL;
};

template<class ContainerAllocator>
struct DataType< ::pedsim_srvs::GetAgentStateRequest_<ContainerAllocator> >
{
  static const char* value()
  {
    return "pedsim_srvs/GetAgentStateRequest";
  }

  static const char* value(const ::pedsim_srvs::GetAgentStateRequest_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::pedsim_srvs::GetAgentStateRequest_<ContainerAllocator> >
{
  static const char* value()
  {
    return "string agent_id\n"
;
  }

  static const char* value(const ::pedsim_srvs::GetAgentStateRequest_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::pedsim_srvs::GetAgentStateRequest_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.agent_id);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct GetAgentStateRequest_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::pedsim_srvs::GetAgentStateRequest_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::pedsim_srvs::GetAgentStateRequest_<ContainerAllocator>& v)
  {
    s << indent << "agent_id: ";
    Printer<std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>>::stream(s, indent + "  ", v.agent_id);
  }
};

} // namespace message_operations
} // namespace ros

#endif // PEDSIM_SRVS_MESSAGE_GETAGENTSTATEREQUEST_H
