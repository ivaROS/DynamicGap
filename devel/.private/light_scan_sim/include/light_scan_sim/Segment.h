// Generated by gencpp from file light_scan_sim/Segment.msg
// DO NOT EDIT!


#ifndef LIGHT_SCAN_SIM_MESSAGE_SEGMENT_H
#define LIGHT_SCAN_SIM_MESSAGE_SEGMENT_H


#include <string>
#include <vector>
#include <memory>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>


namespace light_scan_sim
{
template <class ContainerAllocator>
struct Segment_
{
  typedef Segment_<ContainerAllocator> Type;

  Segment_()
    : type(0)
    , start()
    , end()  {
      start.assign(0.0);

      end.assign(0.0);
  }
  Segment_(const ContainerAllocator& _alloc)
    : type(0)
    , start()
    , end()  {
  (void)_alloc;
      start.assign(0.0);

      end.assign(0.0);
  }



   typedef uint8_t _type_type;
  _type_type type;

   typedef boost::array<float, 2>  _start_type;
  _start_type start;

   typedef boost::array<float, 2>  _end_type;
  _end_type end;





  typedef boost::shared_ptr< ::light_scan_sim::Segment_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::light_scan_sim::Segment_<ContainerAllocator> const> ConstPtr;

}; // struct Segment_

typedef ::light_scan_sim::Segment_<std::allocator<void> > Segment;

typedef boost::shared_ptr< ::light_scan_sim::Segment > SegmentPtr;
typedef boost::shared_ptr< ::light_scan_sim::Segment const> SegmentConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::light_scan_sim::Segment_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::light_scan_sim::Segment_<ContainerAllocator> >::stream(s, "", v);
return s;
}


template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator==(const ::light_scan_sim::Segment_<ContainerAllocator1> & lhs, const ::light_scan_sim::Segment_<ContainerAllocator2> & rhs)
{
  return lhs.type == rhs.type &&
    lhs.start == rhs.start &&
    lhs.end == rhs.end;
}

template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator!=(const ::light_scan_sim::Segment_<ContainerAllocator1> & lhs, const ::light_scan_sim::Segment_<ContainerAllocator2> & rhs)
{
  return !(lhs == rhs);
}


} // namespace light_scan_sim

namespace ros
{
namespace message_traits
{





template <class ContainerAllocator>
struct IsMessage< ::light_scan_sim::Segment_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::light_scan_sim::Segment_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::light_scan_sim::Segment_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::light_scan_sim::Segment_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::light_scan_sim::Segment_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::light_scan_sim::Segment_<ContainerAllocator> const>
  : FalseType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::light_scan_sim::Segment_<ContainerAllocator> >
{
  static const char* value()
  {
    return "a729512fcf7d3d835538035b772a6e15";
  }

  static const char* value(const ::light_scan_sim::Segment_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0xa729512fcf7d3d83ULL;
  static const uint64_t static_value2 = 0x5538035b772a6e15ULL;
};

template<class ContainerAllocator>
struct DataType< ::light_scan_sim::Segment_<ContainerAllocator> >
{
  static const char* value()
  {
    return "light_scan_sim/Segment";
  }

  static const char* value(const ::light_scan_sim::Segment_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::light_scan_sim::Segment_<ContainerAllocator> >
{
  static const char* value()
  {
    return "uint8 type\n"
"float32[2] start\n"
"float32[2] end\n"
;
  }

  static const char* value(const ::light_scan_sim::Segment_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::light_scan_sim::Segment_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.type);
      stream.next(m.start);
      stream.next(m.end);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct Segment_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::light_scan_sim::Segment_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::light_scan_sim::Segment_<ContainerAllocator>& v)
  {
    s << indent << "type: ";
    Printer<uint8_t>::stream(s, indent + "  ", v.type);
    s << indent << "start[]" << std::endl;
    for (size_t i = 0; i < v.start.size(); ++i)
    {
      s << indent << "  start[" << i << "]: ";
      Printer<float>::stream(s, indent + "  ", v.start[i]);
    }
    s << indent << "end[]" << std::endl;
    for (size_t i = 0; i < v.end.size(); ++i)
    {
      s << indent << "  end[" << i << "]: ";
      Printer<float>::stream(s, indent + "  ", v.end[i]);
    }
  }
};

} // namespace message_operations
} // namespace ros

#endif // LIGHT_SCAN_SIM_MESSAGE_SEGMENT_H
