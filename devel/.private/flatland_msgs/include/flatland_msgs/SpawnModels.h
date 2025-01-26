// Generated by gencpp from file flatland_msgs/SpawnModels.msg
// DO NOT EDIT!


#ifndef FLATLAND_MSGS_MESSAGE_SPAWNMODELS_H
#define FLATLAND_MSGS_MESSAGE_SPAWNMODELS_H

#include <ros/service_traits.h>


#include <flatland_msgs/SpawnModelsRequest.h>
#include <flatland_msgs/SpawnModelsResponse.h>


namespace flatland_msgs
{

struct SpawnModels
{

typedef SpawnModelsRequest Request;
typedef SpawnModelsResponse Response;
Request request;
Response response;

typedef Request RequestType;
typedef Response ResponseType;

}; // struct SpawnModels
} // namespace flatland_msgs


namespace ros
{
namespace service_traits
{


template<>
struct MD5Sum< ::flatland_msgs::SpawnModels > {
  static const char* value()
  {
    return "25781a684ae71b6364dc4642097d5ed2";
  }

  static const char* value(const ::flatland_msgs::SpawnModels&) { return value(); }
};

template<>
struct DataType< ::flatland_msgs::SpawnModels > {
  static const char* value()
  {
    return "flatland_msgs/SpawnModels";
  }

  static const char* value(const ::flatland_msgs::SpawnModels&) { return value(); }
};


// service_traits::MD5Sum< ::flatland_msgs::SpawnModelsRequest> should match
// service_traits::MD5Sum< ::flatland_msgs::SpawnModels >
template<>
struct MD5Sum< ::flatland_msgs::SpawnModelsRequest>
{
  static const char* value()
  {
    return MD5Sum< ::flatland_msgs::SpawnModels >::value();
  }
  static const char* value(const ::flatland_msgs::SpawnModelsRequest&)
  {
    return value();
  }
};

// service_traits::DataType< ::flatland_msgs::SpawnModelsRequest> should match
// service_traits::DataType< ::flatland_msgs::SpawnModels >
template<>
struct DataType< ::flatland_msgs::SpawnModelsRequest>
{
  static const char* value()
  {
    return DataType< ::flatland_msgs::SpawnModels >::value();
  }
  static const char* value(const ::flatland_msgs::SpawnModelsRequest&)
  {
    return value();
  }
};

// service_traits::MD5Sum< ::flatland_msgs::SpawnModelsResponse> should match
// service_traits::MD5Sum< ::flatland_msgs::SpawnModels >
template<>
struct MD5Sum< ::flatland_msgs::SpawnModelsResponse>
{
  static const char* value()
  {
    return MD5Sum< ::flatland_msgs::SpawnModels >::value();
  }
  static const char* value(const ::flatland_msgs::SpawnModelsResponse&)
  {
    return value();
  }
};

// service_traits::DataType< ::flatland_msgs::SpawnModelsResponse> should match
// service_traits::DataType< ::flatland_msgs::SpawnModels >
template<>
struct DataType< ::flatland_msgs::SpawnModelsResponse>
{
  static const char* value()
  {
    return DataType< ::flatland_msgs::SpawnModels >::value();
  }
  static const char* value(const ::flatland_msgs::SpawnModelsResponse&)
  {
    return value();
  }
};

} // namespace service_traits
} // namespace ros

#endif // FLATLAND_MSGS_MESSAGE_SPAWNMODELS_H
