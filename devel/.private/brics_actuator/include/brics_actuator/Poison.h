// Generated by gencpp from file brics_actuator/Poison.msg
// DO NOT EDIT!


#ifndef BRICS_ACTUATOR_MESSAGE_POISON_H
#define BRICS_ACTUATOR_MESSAGE_POISON_H


#include <string>
#include <vector>
#include <memory>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>


namespace brics_actuator
{
template <class ContainerAllocator>
struct Poison_
{
  typedef Poison_<ContainerAllocator> Type;

  Poison_()
    : originator()
    , description()
    , qos(0.0)  {
    }
  Poison_(const ContainerAllocator& _alloc)
    : originator(_alloc)
    , description(_alloc)
    , qos(0.0)  {
  (void)_alloc;
    }



   typedef std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>> _originator_type;
  _originator_type originator;

   typedef std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>> _description_type;
  _description_type description;

   typedef float _qos_type;
  _qos_type qos;





  typedef boost::shared_ptr< ::brics_actuator::Poison_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::brics_actuator::Poison_<ContainerAllocator> const> ConstPtr;

}; // struct Poison_

typedef ::brics_actuator::Poison_<std::allocator<void> > Poison;

typedef boost::shared_ptr< ::brics_actuator::Poison > PoisonPtr;
typedef boost::shared_ptr< ::brics_actuator::Poison const> PoisonConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::brics_actuator::Poison_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::brics_actuator::Poison_<ContainerAllocator> >::stream(s, "", v);
return s;
}


template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator==(const ::brics_actuator::Poison_<ContainerAllocator1> & lhs, const ::brics_actuator::Poison_<ContainerAllocator2> & rhs)
{
  return lhs.originator == rhs.originator &&
    lhs.description == rhs.description &&
    lhs.qos == rhs.qos;
}

template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator!=(const ::brics_actuator::Poison_<ContainerAllocator1> & lhs, const ::brics_actuator::Poison_<ContainerAllocator2> & rhs)
{
  return !(lhs == rhs);
}


} // namespace brics_actuator

namespace ros
{
namespace message_traits
{





template <class ContainerAllocator>
struct IsMessage< ::brics_actuator::Poison_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::brics_actuator::Poison_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::brics_actuator::Poison_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::brics_actuator::Poison_<ContainerAllocator> const>
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::brics_actuator::Poison_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::brics_actuator::Poison_<ContainerAllocator> const>
  : FalseType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::brics_actuator::Poison_<ContainerAllocator> >
{
  static const char* value()
  {
    return "b16420a6fd4cc18f64b776ee10e98bb0";
  }

  static const char* value(const ::brics_actuator::Poison_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0xb16420a6fd4cc18fULL;
  static const uint64_t static_value2 = 0x64b776ee10e98bb0ULL;
};

template<class ContainerAllocator>
struct DataType< ::brics_actuator::Poison_<ContainerAllocator> >
{
  static const char* value()
  {
    return "brics_actuator/Poison";
  }

  static const char* value(const ::brics_actuator::Poison_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::brics_actuator::Poison_<ContainerAllocator> >
{
  static const char* value()
  {
    return "string originator 		# node id\n"
"string description 		# encoding still an issue\n"
"float32 qos			# reliability of the channel\n"
"				# 0..1 where 1 means healthy\n"
;
  }

  static const char* value(const ::brics_actuator::Poison_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::brics_actuator::Poison_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.originator);
      stream.next(m.description);
      stream.next(m.qos);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct Poison_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::brics_actuator::Poison_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::brics_actuator::Poison_<ContainerAllocator>& v)
  {
    s << indent << "originator: ";
    Printer<std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>>::stream(s, indent + "  ", v.originator);
    s << indent << "description: ";
    Printer<std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>>::stream(s, indent + "  ", v.description);
    s << indent << "qos: ";
    Printer<float>::stream(s, indent + "  ", v.qos);
  }
};

} // namespace message_operations
} // namespace ros

#endif // BRICS_ACTUATOR_MESSAGE_POISON_H
