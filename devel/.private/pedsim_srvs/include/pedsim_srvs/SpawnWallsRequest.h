// Generated by gencpp from file pedsim_srvs/SpawnWallsRequest.msg
// DO NOT EDIT!


#ifndef PEDSIM_SRVS_MESSAGE_SPAWNWALLSREQUEST_H
#define PEDSIM_SRVS_MESSAGE_SPAWNWALLSREQUEST_H


#include <string>
#include <vector>
#include <memory>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>

#include <pedsim_msgs/Wall.h>

namespace pedsim_srvs
{
template <class ContainerAllocator>
struct SpawnWallsRequest_
{
  typedef SpawnWallsRequest_<ContainerAllocator> Type;

  SpawnWallsRequest_()
    : walls()  {
    }
  SpawnWallsRequest_(const ContainerAllocator& _alloc)
    : walls(_alloc)  {
  (void)_alloc;
    }



   typedef std::vector< ::pedsim_msgs::Wall_<ContainerAllocator> , typename std::allocator_traits<ContainerAllocator>::template rebind_alloc< ::pedsim_msgs::Wall_<ContainerAllocator> >> _walls_type;
  _walls_type walls;





  typedef boost::shared_ptr< ::pedsim_srvs::SpawnWallsRequest_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::pedsim_srvs::SpawnWallsRequest_<ContainerAllocator> const> ConstPtr;

}; // struct SpawnWallsRequest_

typedef ::pedsim_srvs::SpawnWallsRequest_<std::allocator<void> > SpawnWallsRequest;

typedef boost::shared_ptr< ::pedsim_srvs::SpawnWallsRequest > SpawnWallsRequestPtr;
typedef boost::shared_ptr< ::pedsim_srvs::SpawnWallsRequest const> SpawnWallsRequestConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::pedsim_srvs::SpawnWallsRequest_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::pedsim_srvs::SpawnWallsRequest_<ContainerAllocator> >::stream(s, "", v);
return s;
}


template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator==(const ::pedsim_srvs::SpawnWallsRequest_<ContainerAllocator1> & lhs, const ::pedsim_srvs::SpawnWallsRequest_<ContainerAllocator2> & rhs)
{
  return lhs.walls == rhs.walls;
}

template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator!=(const ::pedsim_srvs::SpawnWallsRequest_<ContainerAllocator1> & lhs, const ::pedsim_srvs::SpawnWallsRequest_<ContainerAllocator2> & rhs)
{
  return !(lhs == rhs);
}


} // namespace pedsim_srvs

namespace ros
{
namespace message_traits
{





template <class ContainerAllocator>
struct IsMessage< ::pedsim_srvs::SpawnWallsRequest_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::pedsim_srvs::SpawnWallsRequest_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::pedsim_srvs::SpawnWallsRequest_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::pedsim_srvs::SpawnWallsRequest_<ContainerAllocator> const>
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::pedsim_srvs::SpawnWallsRequest_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::pedsim_srvs::SpawnWallsRequest_<ContainerAllocator> const>
  : FalseType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::pedsim_srvs::SpawnWallsRequest_<ContainerAllocator> >
{
  static const char* value()
  {
    return "2e5057156bb1500cfecde0bf4bbe71bc";
  }

  static const char* value(const ::pedsim_srvs::SpawnWallsRequest_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0x2e5057156bb1500cULL;
  static const uint64_t static_value2 = 0xfecde0bf4bbe71bcULL;
};

template<class ContainerAllocator>
struct DataType< ::pedsim_srvs::SpawnWallsRequest_<ContainerAllocator> >
{
  static const char* value()
  {
    return "pedsim_srvs/SpawnWallsRequest";
  }

  static const char* value(const ::pedsim_srvs::SpawnWallsRequest_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::pedsim_srvs::SpawnWallsRequest_<ContainerAllocator> >
{
  static const char* value()
  {
    return "# Junhui Li\n"
"# info the pedsim_ros the position of static obstacles\n"
"pedsim_msgs/Wall[] walls\n"
"\n"
"================================================================================\n"
"MSG: pedsim_msgs/Wall\n"
"# A line obstacle in the simulator.\n"
"\n"
"geometry_msgs/Point start\n"
"geometry_msgs/Point end\n"
"uint8 layer\n"
"================================================================================\n"
"MSG: geometry_msgs/Point\n"
"# This contains the position of a point in free space\n"
"float64 x\n"
"float64 y\n"
"float64 z\n"
;
  }

  static const char* value(const ::pedsim_srvs::SpawnWallsRequest_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::pedsim_srvs::SpawnWallsRequest_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.walls);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct SpawnWallsRequest_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::pedsim_srvs::SpawnWallsRequest_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::pedsim_srvs::SpawnWallsRequest_<ContainerAllocator>& v)
  {
    s << indent << "walls[]" << std::endl;
    for (size_t i = 0; i < v.walls.size(); ++i)
    {
      s << indent << "  walls[" << i << "]: ";
      s << std::endl;
      s << indent;
      Printer< ::pedsim_msgs::Wall_<ContainerAllocator> >::stream(s, indent + "    ", v.walls[i]);
    }
  }
};

} // namespace message_operations
} // namespace ros

#endif // PEDSIM_SRVS_MESSAGE_SPAWNWALLSREQUEST_H
