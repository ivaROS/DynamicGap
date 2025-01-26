// Generated by gencpp from file spencer_social_relation_msgs/SocialActivity.msg
// DO NOT EDIT!


#ifndef SPENCER_SOCIAL_RELATION_MSGS_MESSAGE_SOCIALACTIVITY_H
#define SPENCER_SOCIAL_RELATION_MSGS_MESSAGE_SOCIALACTIVITY_H


#include <string>
#include <vector>
#include <memory>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>


namespace spencer_social_relation_msgs
{
template <class ContainerAllocator>
struct SocialActivity_
{
  typedef SocialActivity_<ContainerAllocator> Type;

  SocialActivity_()
    : type()
    , confidence(0.0)
    , track_ids()  {
    }
  SocialActivity_(const ContainerAllocator& _alloc)
    : type(_alloc)
    , confidence(0.0)
    , track_ids(_alloc)  {
  (void)_alloc;
    }



   typedef std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>> _type_type;
  _type_type type;

   typedef float _confidence_type;
  _confidence_type confidence;

   typedef std::vector<std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>>> _track_ids_type;
  _track_ids_type track_ids;



// reducing the odds to have name collisions with Windows.h 
#if defined(_WIN32) && defined(TYPE_SHOPPING)
  #undef TYPE_SHOPPING
#endif
#if defined(_WIN32) && defined(TYPE_STANDING)
  #undef TYPE_STANDING
#endif
#if defined(_WIN32) && defined(TYPE_INDIVIDUAL_MOVING)
  #undef TYPE_INDIVIDUAL_MOVING
#endif
#if defined(_WIN32) && defined(TYPE_WAITING_IN_QUEUE)
  #undef TYPE_WAITING_IN_QUEUE
#endif
#if defined(_WIN32) && defined(TYPE_LOOKING_AT_INFORMATION_SCREEN)
  #undef TYPE_LOOKING_AT_INFORMATION_SCREEN
#endif
#if defined(_WIN32) && defined(TYPE_LOOKING_AT_KIOSK)
  #undef TYPE_LOOKING_AT_KIOSK
#endif
#if defined(_WIN32) && defined(TYPE_GROUP_ASSEMBLING)
  #undef TYPE_GROUP_ASSEMBLING
#endif
#if defined(_WIN32) && defined(TYPE_GROUP_MOVING)
  #undef TYPE_GROUP_MOVING
#endif
#if defined(_WIN32) && defined(TYPE_FLOW_WITH_ROBOT)
  #undef TYPE_FLOW_WITH_ROBOT
#endif
#if defined(_WIN32) && defined(TYPE_ANTIFLOW_AGAINST_ROBOT)
  #undef TYPE_ANTIFLOW_AGAINST_ROBOT
#endif
#if defined(_WIN32) && defined(TYPE_WAITING_FOR_OTHERS)
  #undef TYPE_WAITING_FOR_OTHERS
#endif
#if defined(_WIN32) && defined(TYPE_LOOKING_FOR_HELP)
  #undef TYPE_LOOKING_FOR_HELP
#endif


  static const std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>> TYPE_SHOPPING;
  static const std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>> TYPE_STANDING;
  static const std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>> TYPE_INDIVIDUAL_MOVING;
  static const std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>> TYPE_WAITING_IN_QUEUE;
  static const std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>> TYPE_LOOKING_AT_INFORMATION_SCREEN;
  static const std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>> TYPE_LOOKING_AT_KIOSK;
  static const std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>> TYPE_GROUP_ASSEMBLING;
  static const std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>> TYPE_GROUP_MOVING;
  static const std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>> TYPE_FLOW_WITH_ROBOT;
  static const std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>> TYPE_ANTIFLOW_AGAINST_ROBOT;
  static const std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>> TYPE_WAITING_FOR_OTHERS;
  static const std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>> TYPE_LOOKING_FOR_HELP;

  typedef boost::shared_ptr< ::spencer_social_relation_msgs::SocialActivity_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::spencer_social_relation_msgs::SocialActivity_<ContainerAllocator> const> ConstPtr;

}; // struct SocialActivity_

typedef ::spencer_social_relation_msgs::SocialActivity_<std::allocator<void> > SocialActivity;

typedef boost::shared_ptr< ::spencer_social_relation_msgs::SocialActivity > SocialActivityPtr;
typedef boost::shared_ptr< ::spencer_social_relation_msgs::SocialActivity const> SocialActivityConstPtr;

// constants requiring out of line definition

   
   template<typename ContainerAllocator> const std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>
      SocialActivity_<ContainerAllocator>::TYPE_SHOPPING =
        
          "shopping"
        
        ;
   

   
   template<typename ContainerAllocator> const std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>
      SocialActivity_<ContainerAllocator>::TYPE_STANDING =
        
          "standing"
        
        ;
   

   
   template<typename ContainerAllocator> const std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>
      SocialActivity_<ContainerAllocator>::TYPE_INDIVIDUAL_MOVING =
        
          "individual_moving"
        
        ;
   

   
   template<typename ContainerAllocator> const std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>
      SocialActivity_<ContainerAllocator>::TYPE_WAITING_IN_QUEUE =
        
          "waiting_in_queue"
        
        ;
   

   
   template<typename ContainerAllocator> const std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>
      SocialActivity_<ContainerAllocator>::TYPE_LOOKING_AT_INFORMATION_SCREEN =
        
          "looking_at_information_screen"
        
        ;
   

   
   template<typename ContainerAllocator> const std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>
      SocialActivity_<ContainerAllocator>::TYPE_LOOKING_AT_KIOSK =
        
          "looking_at_kiosk"
        
        ;
   

   
   template<typename ContainerAllocator> const std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>
      SocialActivity_<ContainerAllocator>::TYPE_GROUP_ASSEMBLING =
        
          "group_assembling"
        
        ;
   

   
   template<typename ContainerAllocator> const std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>
      SocialActivity_<ContainerAllocator>::TYPE_GROUP_MOVING =
        
          "group_moving"
        
        ;
   

   
   template<typename ContainerAllocator> const std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>
      SocialActivity_<ContainerAllocator>::TYPE_FLOW_WITH_ROBOT =
        
          "flow"
        
        ;
   

   
   template<typename ContainerAllocator> const std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>
      SocialActivity_<ContainerAllocator>::TYPE_ANTIFLOW_AGAINST_ROBOT =
        
          "antiflow"
        
        ;
   

   
   template<typename ContainerAllocator> const std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>
      SocialActivity_<ContainerAllocator>::TYPE_WAITING_FOR_OTHERS =
        
          "waiting_for_others"
        
        ;
   

   
   template<typename ContainerAllocator> const std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>
      SocialActivity_<ContainerAllocator>::TYPE_LOOKING_FOR_HELP =
        
          "looking_for_help"
        
        ;
   



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::spencer_social_relation_msgs::SocialActivity_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::spencer_social_relation_msgs::SocialActivity_<ContainerAllocator> >::stream(s, "", v);
return s;
}


template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator==(const ::spencer_social_relation_msgs::SocialActivity_<ContainerAllocator1> & lhs, const ::spencer_social_relation_msgs::SocialActivity_<ContainerAllocator2> & rhs)
{
  return lhs.type == rhs.type &&
    lhs.confidence == rhs.confidence &&
    lhs.track_ids == rhs.track_ids;
}

template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator!=(const ::spencer_social_relation_msgs::SocialActivity_<ContainerAllocator1> & lhs, const ::spencer_social_relation_msgs::SocialActivity_<ContainerAllocator2> & rhs)
{
  return !(lhs == rhs);
}


} // namespace spencer_social_relation_msgs

namespace ros
{
namespace message_traits
{





template <class ContainerAllocator>
struct IsMessage< ::spencer_social_relation_msgs::SocialActivity_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::spencer_social_relation_msgs::SocialActivity_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::spencer_social_relation_msgs::SocialActivity_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::spencer_social_relation_msgs::SocialActivity_<ContainerAllocator> const>
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::spencer_social_relation_msgs::SocialActivity_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::spencer_social_relation_msgs::SocialActivity_<ContainerAllocator> const>
  : FalseType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::spencer_social_relation_msgs::SocialActivity_<ContainerAllocator> >
{
  static const char* value()
  {
    return "da62314a942310457724e5a4a9e960d0";
  }

  static const char* value(const ::spencer_social_relation_msgs::SocialActivity_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0xda62314a94231045ULL;
  static const uint64_t static_value2 = 0x7724e5a4a9e960d0ULL;
};

template<class ContainerAllocator>
struct DataType< ::spencer_social_relation_msgs::SocialActivity_<ContainerAllocator> >
{
  static const char* value()
  {
    return "spencer_social_relation_msgs/SocialActivity";
  }

  static const char* value(const ::spencer_social_relation_msgs::SocialActivity_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::spencer_social_relation_msgs::SocialActivity_<ContainerAllocator> >
{
  static const char* value()
  {
    return "string      type        # see constants below\n"
"float32     confidence  # detection confidence\n"
"\n"
"string[]      track_ids   # IDs of all person tracks involved in the activity, might be one or multiple\n"
"\n"
"\n"
"# Constants for social activity type (just examples at the moment)\n"
"string      TYPE_SHOPPING = shopping\n"
"string      TYPE_STANDING = standing\n"
"string      TYPE_INDIVIDUAL_MOVING = individual_moving\n"
"string      TYPE_WAITING_IN_QUEUE = waiting_in_queue\n"
"string      TYPE_LOOKING_AT_INFORMATION_SCREEN = looking_at_information_screen\n"
"string      TYPE_LOOKING_AT_KIOSK = looking_at_kiosk\n"
"string      TYPE_GROUP_ASSEMBLING = group_assembling\n"
"string      TYPE_GROUP_MOVING = group_moving\n"
"string      TYPE_FLOW_WITH_ROBOT = flow\n"
"string      TYPE_ANTIFLOW_AGAINST_ROBOT = antiflow\n"
"string      TYPE_WAITING_FOR_OTHERS = waiting_for_others\n"
"string      TYPE_LOOKING_FOR_HELP = looking_for_help\n"
"\n"
"\n"
"#string      TYPE_COMMUNICATING = communicating\n"
"#string      TYPE_TAKING_PHOTOGRAPH = taking_photograph\n"
"#string      TYPE_TALKING_ON_PHONE = talking_on_phone\n"
;
  }

  static const char* value(const ::spencer_social_relation_msgs::SocialActivity_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::spencer_social_relation_msgs::SocialActivity_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.type);
      stream.next(m.confidence);
      stream.next(m.track_ids);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct SocialActivity_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::spencer_social_relation_msgs::SocialActivity_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::spencer_social_relation_msgs::SocialActivity_<ContainerAllocator>& v)
  {
    s << indent << "type: ";
    Printer<std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>>::stream(s, indent + "  ", v.type);
    s << indent << "confidence: ";
    Printer<float>::stream(s, indent + "  ", v.confidence);
    s << indent << "track_ids[]" << std::endl;
    for (size_t i = 0; i < v.track_ids.size(); ++i)
    {
      s << indent << "  track_ids[" << i << "]: ";
      Printer<std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>>::stream(s, indent + "  ", v.track_ids[i]);
    }
  }
};

} // namespace message_operations
} // namespace ros

#endif // SPENCER_SOCIAL_RELATION_MSGS_MESSAGE_SOCIALACTIVITY_H
