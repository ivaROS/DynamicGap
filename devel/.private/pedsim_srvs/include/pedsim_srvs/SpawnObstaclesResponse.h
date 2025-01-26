// Generated by gencpp from file pedsim_srvs/SpawnObstaclesResponse.msg
// DO NOT EDIT!


#ifndef PEDSIM_SRVS_MESSAGE_SPAWNOBSTACLESRESPONSE_H
#define PEDSIM_SRVS_MESSAGE_SPAWNOBSTACLESRESPONSE_H


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
struct SpawnObstaclesResponse_
{
  typedef SpawnObstaclesResponse_<ContainerAllocator> Type;

  SpawnObstaclesResponse_()
    : success(false)  {
    }
  SpawnObstaclesResponse_(const ContainerAllocator& _alloc)
    : success(false)  {
  (void)_alloc;
    }



   typedef uint8_t _success_type;
  _success_type success;





  typedef boost::shared_ptr< ::pedsim_srvs::SpawnObstaclesResponse_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::pedsim_srvs::SpawnObstaclesResponse_<ContainerAllocator> const> ConstPtr;

}; // struct SpawnObstaclesResponse_

typedef ::pedsim_srvs::SpawnObstaclesResponse_<std::allocator<void> > SpawnObstaclesResponse;

typedef boost::shared_ptr< ::pedsim_srvs::SpawnObstaclesResponse > SpawnObstaclesResponsePtr;
typedef boost::shared_ptr< ::pedsim_srvs::SpawnObstaclesResponse const> SpawnObstaclesResponseConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::pedsim_srvs::SpawnObstaclesResponse_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::pedsim_srvs::SpawnObstaclesResponse_<ContainerAllocator> >::stream(s, "", v);
return s;
}


template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator==(const ::pedsim_srvs::SpawnObstaclesResponse_<ContainerAllocator1> & lhs, const ::pedsim_srvs::SpawnObstaclesResponse_<ContainerAllocator2> & rhs)
{
  return lhs.success == rhs.success;
}

template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator!=(const ::pedsim_srvs::SpawnObstaclesResponse_<ContainerAllocator1> & lhs, const ::pedsim_srvs::SpawnObstaclesResponse_<ContainerAllocator2> & rhs)
{
  return !(lhs == rhs);
}


} // namespace pedsim_srvs

namespace ros
{
namespace message_traits
{





template <class ContainerAllocator>
struct IsMessage< ::pedsim_srvs::SpawnObstaclesResponse_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::pedsim_srvs::SpawnObstaclesResponse_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::pedsim_srvs::SpawnObstaclesResponse_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::pedsim_srvs::SpawnObstaclesResponse_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::pedsim_srvs::SpawnObstaclesResponse_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::pedsim_srvs::SpawnObstaclesResponse_<ContainerAllocator> const>
  : FalseType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::pedsim_srvs::SpawnObstaclesResponse_<ContainerAllocator> >
{
  static const char* value()
  {
    return "358e233cde0c8a8bcfea4ce193f8fc15";
  }

  static const char* value(const ::pedsim_srvs::SpawnObstaclesResponse_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0x358e233cde0c8a8bULL;
  static const uint64_t static_value2 = 0xcfea4ce193f8fc15ULL;
};

template<class ContainerAllocator>
struct DataType< ::pedsim_srvs::SpawnObstaclesResponse_<ContainerAllocator> >
{
  static const char* value()
  {
    return "pedsim_srvs/SpawnObstaclesResponse";
  }

  static const char* value(const ::pedsim_srvs::SpawnObstaclesResponse_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::pedsim_srvs::SpawnObstaclesResponse_<ContainerAllocator> >
{
  static const char* value()
  {
    return "bool success\n"
"\n"
;
  }

  static const char* value(const ::pedsim_srvs::SpawnObstaclesResponse_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::pedsim_srvs::SpawnObstaclesResponse_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.success);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct SpawnObstaclesResponse_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::pedsim_srvs::SpawnObstaclesResponse_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::pedsim_srvs::SpawnObstaclesResponse_<ContainerAllocator>& v)
  {
    s << indent << "success: ";
    Printer<uint8_t>::stream(s, indent + "  ", v.success);
  }
};

} // namespace message_operations
} // namespace ros

#endif // PEDSIM_SRVS_MESSAGE_SPAWNOBSTACLESRESPONSE_H
