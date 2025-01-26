// Generated by gencpp from file rosnav/GetActionResponse.msg
// DO NOT EDIT!


#ifndef ROSNAV_MESSAGE_GETACTIONRESPONSE_H
#define ROSNAV_MESSAGE_GETACTIONRESPONSE_H


#include <string>
#include <vector>
#include <memory>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>


namespace rosnav
{
template <class ContainerAllocator>
struct GetActionResponse_
{
  typedef GetActionResponse_<ContainerAllocator> Type;

  GetActionResponse_()
    : action()  {
    }
  GetActionResponse_(const ContainerAllocator& _alloc)
    : action(_alloc)  {
  (void)_alloc;
    }



   typedef std::vector<float, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<float>> _action_type;
  _action_type action;





  typedef boost::shared_ptr< ::rosnav::GetActionResponse_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::rosnav::GetActionResponse_<ContainerAllocator> const> ConstPtr;

}; // struct GetActionResponse_

typedef ::rosnav::GetActionResponse_<std::allocator<void> > GetActionResponse;

typedef boost::shared_ptr< ::rosnav::GetActionResponse > GetActionResponsePtr;
typedef boost::shared_ptr< ::rosnav::GetActionResponse const> GetActionResponseConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::rosnav::GetActionResponse_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::rosnav::GetActionResponse_<ContainerAllocator> >::stream(s, "", v);
return s;
}


template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator==(const ::rosnav::GetActionResponse_<ContainerAllocator1> & lhs, const ::rosnav::GetActionResponse_<ContainerAllocator2> & rhs)
{
  return lhs.action == rhs.action;
}

template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator!=(const ::rosnav::GetActionResponse_<ContainerAllocator1> & lhs, const ::rosnav::GetActionResponse_<ContainerAllocator2> & rhs)
{
  return !(lhs == rhs);
}


} // namespace rosnav

namespace ros
{
namespace message_traits
{





template <class ContainerAllocator>
struct IsMessage< ::rosnav::GetActionResponse_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::rosnav::GetActionResponse_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::rosnav::GetActionResponse_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::rosnav::GetActionResponse_<ContainerAllocator> const>
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::rosnav::GetActionResponse_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::rosnav::GetActionResponse_<ContainerAllocator> const>
  : FalseType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::rosnav::GetActionResponse_<ContainerAllocator> >
{
  static const char* value()
  {
    return "a70a7d92e0376dcb967914076f276ea6";
  }

  static const char* value(const ::rosnav::GetActionResponse_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0xa70a7d92e0376dcbULL;
  static const uint64_t static_value2 = 0x967914076f276ea6ULL;
};

template<class ContainerAllocator>
struct DataType< ::rosnav::GetActionResponse_<ContainerAllocator> >
{
  static const char* value()
  {
    return "rosnav/GetActionResponse";
  }

  static const char* value(const ::rosnav::GetActionResponse_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::rosnav::GetActionResponse_<ContainerAllocator> >
{
  static const char* value()
  {
    return "float32[] action\n"
;
  }

  static const char* value(const ::rosnav::GetActionResponse_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::rosnav::GetActionResponse_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.action);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct GetActionResponse_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::rosnav::GetActionResponse_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::rosnav::GetActionResponse_<ContainerAllocator>& v)
  {
    s << indent << "action[]" << std::endl;
    for (size_t i = 0; i < v.action.size(); ++i)
    {
      s << indent << "  action[" << i << "]: ";
      Printer<float>::stream(s, indent + "  ", v.action[i]);
    }
  }
};

} // namespace message_operations
} // namespace ros

#endif // ROSNAV_MESSAGE_GETACTIONRESPONSE_H
