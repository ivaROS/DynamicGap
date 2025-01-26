// Generated by gencpp from file pedsim_srvs/SpawnPedResponse.msg
// DO NOT EDIT!


#ifndef PEDSIM_SRVS_MESSAGE_SPAWNPEDRESPONSE_H
#define PEDSIM_SRVS_MESSAGE_SPAWNPEDRESPONSE_H


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
struct SpawnPedResponse_
{
  typedef SpawnPedResponse_<ContainerAllocator> Type;

  SpawnPedResponse_()
    : success(false)  {
    }
  SpawnPedResponse_(const ContainerAllocator& _alloc)
    : success(false)  {
  (void)_alloc;
    }



   typedef uint8_t _success_type;
  _success_type success;





  typedef boost::shared_ptr< ::pedsim_srvs::SpawnPedResponse_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::pedsim_srvs::SpawnPedResponse_<ContainerAllocator> const> ConstPtr;

}; // struct SpawnPedResponse_

typedef ::pedsim_srvs::SpawnPedResponse_<std::allocator<void> > SpawnPedResponse;

typedef boost::shared_ptr< ::pedsim_srvs::SpawnPedResponse > SpawnPedResponsePtr;
typedef boost::shared_ptr< ::pedsim_srvs::SpawnPedResponse const> SpawnPedResponseConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::pedsim_srvs::SpawnPedResponse_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::pedsim_srvs::SpawnPedResponse_<ContainerAllocator> >::stream(s, "", v);
return s;
}


template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator==(const ::pedsim_srvs::SpawnPedResponse_<ContainerAllocator1> & lhs, const ::pedsim_srvs::SpawnPedResponse_<ContainerAllocator2> & rhs)
{
  return lhs.success == rhs.success;
}

template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator!=(const ::pedsim_srvs::SpawnPedResponse_<ContainerAllocator1> & lhs, const ::pedsim_srvs::SpawnPedResponse_<ContainerAllocator2> & rhs)
{
  return !(lhs == rhs);
}


} // namespace pedsim_srvs

namespace ros
{
namespace message_traits
{





template <class ContainerAllocator>
struct IsMessage< ::pedsim_srvs::SpawnPedResponse_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::pedsim_srvs::SpawnPedResponse_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::pedsim_srvs::SpawnPedResponse_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::pedsim_srvs::SpawnPedResponse_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::pedsim_srvs::SpawnPedResponse_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::pedsim_srvs::SpawnPedResponse_<ContainerAllocator> const>
  : FalseType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::pedsim_srvs::SpawnPedResponse_<ContainerAllocator> >
{
  static const char* value()
  {
    return "358e233cde0c8a8bcfea4ce193f8fc15";
  }

  static const char* value(const ::pedsim_srvs::SpawnPedResponse_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0x358e233cde0c8a8bULL;
  static const uint64_t static_value2 = 0xcfea4ce193f8fc15ULL;
};

template<class ContainerAllocator>
struct DataType< ::pedsim_srvs::SpawnPedResponse_<ContainerAllocator> >
{
  static const char* value()
  {
    return "pedsim_srvs/SpawnPedResponse";
  }

  static const char* value(const ::pedsim_srvs::SpawnPedResponse_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::pedsim_srvs::SpawnPedResponse_<ContainerAllocator> >
{
  static const char* value()
  {
    return "bool success\n"
"\n"
;
  }

  static const char* value(const ::pedsim_srvs::SpawnPedResponse_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::pedsim_srvs::SpawnPedResponse_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.success);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct SpawnPedResponse_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::pedsim_srvs::SpawnPedResponse_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::pedsim_srvs::SpawnPedResponse_<ContainerAllocator>& v)
  {
    s << indent << "success: ";
    Printer<uint8_t>::stream(s, indent + "  ", v.success);
  }
};

} // namespace message_operations
} // namespace ros

#endif // PEDSIM_SRVS_MESSAGE_SPAWNPEDRESPONSE_H
