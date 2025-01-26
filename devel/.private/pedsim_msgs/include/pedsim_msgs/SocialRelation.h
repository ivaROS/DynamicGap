// Generated by gencpp from file pedsim_msgs/SocialRelation.msg
// DO NOT EDIT!


#ifndef PEDSIM_MSGS_MESSAGE_SOCIALRELATION_H
#define PEDSIM_MSGS_MESSAGE_SOCIALRELATION_H


#include <string>
#include <vector>
#include <memory>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>


namespace pedsim_msgs
{
template <class ContainerAllocator>
struct SocialRelation_
{
  typedef SocialRelation_<ContainerAllocator> Type;

  SocialRelation_()
    : type()
    , strength(0.0)
    , track1_id()
    , track2_id()  {
    }
  SocialRelation_(const ContainerAllocator& _alloc)
    : type(_alloc)
    , strength(0.0)
    , track1_id(_alloc)
    , track2_id(_alloc)  {
  (void)_alloc;
    }



   typedef std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>> _type_type;
  _type_type type;

   typedef float _strength_type;
  _strength_type strength;

   typedef std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>> _track1_id_type;
  _track1_id_type track1_id;

   typedef std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>> _track2_id_type;
  _track2_id_type track2_id;



// reducing the odds to have name collisions with Windows.h 
#if defined(_WIN32) && defined(TYPE_SPATIAL)
  #undef TYPE_SPATIAL
#endif
#if defined(_WIN32) && defined(TYPE_ROMANTIC)
  #undef TYPE_ROMANTIC
#endif
#if defined(_WIN32) && defined(TYPE_PARENT_CHILD)
  #undef TYPE_PARENT_CHILD
#endif
#if defined(_WIN32) && defined(TYPE_FRIENDSHIP)
  #undef TYPE_FRIENDSHIP
#endif


  static const std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>> TYPE_SPATIAL;
  static const std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>> TYPE_ROMANTIC;
  static const std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>> TYPE_PARENT_CHILD;
  static const std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>> TYPE_FRIENDSHIP;

  typedef boost::shared_ptr< ::pedsim_msgs::SocialRelation_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::pedsim_msgs::SocialRelation_<ContainerAllocator> const> ConstPtr;

}; // struct SocialRelation_

typedef ::pedsim_msgs::SocialRelation_<std::allocator<void> > SocialRelation;

typedef boost::shared_ptr< ::pedsim_msgs::SocialRelation > SocialRelationPtr;
typedef boost::shared_ptr< ::pedsim_msgs::SocialRelation const> SocialRelationConstPtr;

// constants requiring out of line definition

   
   template<typename ContainerAllocator> const std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>
      SocialRelation_<ContainerAllocator>::TYPE_SPATIAL =
        
          "\"spatial\""
        
        ;
   

   
   template<typename ContainerAllocator> const std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>
      SocialRelation_<ContainerAllocator>::TYPE_ROMANTIC =
        
          "\"romantic\""
        
        ;
   

   
   template<typename ContainerAllocator> const std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>
      SocialRelation_<ContainerAllocator>::TYPE_PARENT_CHILD =
        
          "\"parent_child\""
        
        ;
   

   
   template<typename ContainerAllocator> const std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>
      SocialRelation_<ContainerAllocator>::TYPE_FRIENDSHIP =
        
          "\"friendship\""
        
        ;
   



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::pedsim_msgs::SocialRelation_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::pedsim_msgs::SocialRelation_<ContainerAllocator> >::stream(s, "", v);
return s;
}


template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator==(const ::pedsim_msgs::SocialRelation_<ContainerAllocator1> & lhs, const ::pedsim_msgs::SocialRelation_<ContainerAllocator2> & rhs)
{
  return lhs.type == rhs.type &&
    lhs.strength == rhs.strength &&
    lhs.track1_id == rhs.track1_id &&
    lhs.track2_id == rhs.track2_id;
}

template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator!=(const ::pedsim_msgs::SocialRelation_<ContainerAllocator1> & lhs, const ::pedsim_msgs::SocialRelation_<ContainerAllocator2> & rhs)
{
  return !(lhs == rhs);
}


} // namespace pedsim_msgs

namespace ros
{
namespace message_traits
{





template <class ContainerAllocator>
struct IsMessage< ::pedsim_msgs::SocialRelation_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::pedsim_msgs::SocialRelation_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::pedsim_msgs::SocialRelation_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::pedsim_msgs::SocialRelation_<ContainerAllocator> const>
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::pedsim_msgs::SocialRelation_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::pedsim_msgs::SocialRelation_<ContainerAllocator> const>
  : FalseType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::pedsim_msgs::SocialRelation_<ContainerAllocator> >
{
  static const char* value()
  {
    return "d1649d28f150a194ade778eaadf431e1";
  }

  static const char* value(const ::pedsim_msgs::SocialRelation_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0xd1649d28f150a194ULL;
  static const uint64_t static_value2 = 0xade778eaadf431e1ULL;
};

template<class ContainerAllocator>
struct DataType< ::pedsim_msgs::SocialRelation_<ContainerAllocator> >
{
  static const char* value()
  {
    return "pedsim_msgs/SocialRelation";
  }

  static const char* value(const ::pedsim_msgs::SocialRelation_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::pedsim_msgs::SocialRelation_<ContainerAllocator> >
{
  static const char* value()
  {
    return "string      type        # e.g. mother-son relationship, romantic relationship, etc.\n"
"float32     strength    # relationship strength between 0.0 and 1.0\n"
"\n"
"string      track1_id\n"
"string      track2_id\n"
"\n"
"\n"
"# Constants for type (just examples at the moment)\n"
"string      TYPE_SPATIAL  = \"spatial\"\n"
"string      TYPE_ROMANTIC = \"romantic\"\n"
"string      TYPE_PARENT_CHILD = \"parent_child\"\n"
"string      TYPE_FRIENDSHIP = \"friendship\"\n"
;
  }

  static const char* value(const ::pedsim_msgs::SocialRelation_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::pedsim_msgs::SocialRelation_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.type);
      stream.next(m.strength);
      stream.next(m.track1_id);
      stream.next(m.track2_id);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct SocialRelation_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::pedsim_msgs::SocialRelation_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::pedsim_msgs::SocialRelation_<ContainerAllocator>& v)
  {
    s << indent << "type: ";
    Printer<std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>>::stream(s, indent + "  ", v.type);
    s << indent << "strength: ";
    Printer<float>::stream(s, indent + "  ", v.strength);
    s << indent << "track1_id: ";
    Printer<std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>>::stream(s, indent + "  ", v.track1_id);
    s << indent << "track2_id: ";
    Printer<std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>>::stream(s, indent + "  ", v.track2_id);
  }
};

} // namespace message_operations
} // namespace ros

#endif // PEDSIM_MSGS_MESSAGE_SOCIALRELATION_H
