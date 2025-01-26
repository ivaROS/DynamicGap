// Generated by gencpp from file cob_control_msgs/GetObstacleDistance.msg
// DO NOT EDIT!


#ifndef COB_CONTROL_MSGS_MESSAGE_GETOBSTACLEDISTANCE_H
#define COB_CONTROL_MSGS_MESSAGE_GETOBSTACLEDISTANCE_H

#include <ros/service_traits.h>


#include <cob_control_msgs/GetObstacleDistanceRequest.h>
#include <cob_control_msgs/GetObstacleDistanceResponse.h>


namespace cob_control_msgs
{

struct GetObstacleDistance
{

typedef GetObstacleDistanceRequest Request;
typedef GetObstacleDistanceResponse Response;
Request request;
Response response;

typedef Request RequestType;
typedef Response ResponseType;

}; // struct GetObstacleDistance
} // namespace cob_control_msgs


namespace ros
{
namespace service_traits
{


template<>
struct MD5Sum< ::cob_control_msgs::GetObstacleDistance > {
  static const char* value()
  {
    return "f7fa899b8b5bba2d5f4beecae1d4101b";
  }

  static const char* value(const ::cob_control_msgs::GetObstacleDistance&) { return value(); }
};

template<>
struct DataType< ::cob_control_msgs::GetObstacleDistance > {
  static const char* value()
  {
    return "cob_control_msgs/GetObstacleDistance";
  }

  static const char* value(const ::cob_control_msgs::GetObstacleDistance&) { return value(); }
};


// service_traits::MD5Sum< ::cob_control_msgs::GetObstacleDistanceRequest> should match
// service_traits::MD5Sum< ::cob_control_msgs::GetObstacleDistance >
template<>
struct MD5Sum< ::cob_control_msgs::GetObstacleDistanceRequest>
{
  static const char* value()
  {
    return MD5Sum< ::cob_control_msgs::GetObstacleDistance >::value();
  }
  static const char* value(const ::cob_control_msgs::GetObstacleDistanceRequest&)
  {
    return value();
  }
};

// service_traits::DataType< ::cob_control_msgs::GetObstacleDistanceRequest> should match
// service_traits::DataType< ::cob_control_msgs::GetObstacleDistance >
template<>
struct DataType< ::cob_control_msgs::GetObstacleDistanceRequest>
{
  static const char* value()
  {
    return DataType< ::cob_control_msgs::GetObstacleDistance >::value();
  }
  static const char* value(const ::cob_control_msgs::GetObstacleDistanceRequest&)
  {
    return value();
  }
};

// service_traits::MD5Sum< ::cob_control_msgs::GetObstacleDistanceResponse> should match
// service_traits::MD5Sum< ::cob_control_msgs::GetObstacleDistance >
template<>
struct MD5Sum< ::cob_control_msgs::GetObstacleDistanceResponse>
{
  static const char* value()
  {
    return MD5Sum< ::cob_control_msgs::GetObstacleDistance >::value();
  }
  static const char* value(const ::cob_control_msgs::GetObstacleDistanceResponse&)
  {
    return value();
  }
};

// service_traits::DataType< ::cob_control_msgs::GetObstacleDistanceResponse> should match
// service_traits::DataType< ::cob_control_msgs::GetObstacleDistance >
template<>
struct DataType< ::cob_control_msgs::GetObstacleDistanceResponse>
{
  static const char* value()
  {
    return DataType< ::cob_control_msgs::GetObstacleDistance >::value();
  }
  static const char* value(const ::cob_control_msgs::GetObstacleDistanceResponse&)
  {
    return value();
  }
};

} // namespace service_traits
} // namespace ros

#endif // COB_CONTROL_MSGS_MESSAGE_GETOBSTACLEDISTANCE_H
