// Generated by gencpp from file pedsim_srvs/SpawnObstacles.msg
// DO NOT EDIT!


#ifndef PEDSIM_SRVS_MESSAGE_SPAWNOBSTACLES_H
#define PEDSIM_SRVS_MESSAGE_SPAWNOBSTACLES_H

#include <ros/service_traits.h>


#include <pedsim_srvs/SpawnObstaclesRequest.h>
#include <pedsim_srvs/SpawnObstaclesResponse.h>


namespace pedsim_srvs
{

struct SpawnObstacles
{

typedef SpawnObstaclesRequest Request;
typedef SpawnObstaclesResponse Response;
Request request;
Response response;

typedef Request RequestType;
typedef Response ResponseType;

}; // struct SpawnObstacles
} // namespace pedsim_srvs


namespace ros
{
namespace service_traits
{


template<>
struct MD5Sum< ::pedsim_srvs::SpawnObstacles > {
  static const char* value()
  {
    return "70ba2207d643e7ef873b86272b010369";
  }

  static const char* value(const ::pedsim_srvs::SpawnObstacles&) { return value(); }
};

template<>
struct DataType< ::pedsim_srvs::SpawnObstacles > {
  static const char* value()
  {
    return "pedsim_srvs/SpawnObstacles";
  }

  static const char* value(const ::pedsim_srvs::SpawnObstacles&) { return value(); }
};


// service_traits::MD5Sum< ::pedsim_srvs::SpawnObstaclesRequest> should match
// service_traits::MD5Sum< ::pedsim_srvs::SpawnObstacles >
template<>
struct MD5Sum< ::pedsim_srvs::SpawnObstaclesRequest>
{
  static const char* value()
  {
    return MD5Sum< ::pedsim_srvs::SpawnObstacles >::value();
  }
  static const char* value(const ::pedsim_srvs::SpawnObstaclesRequest&)
  {
    return value();
  }
};

// service_traits::DataType< ::pedsim_srvs::SpawnObstaclesRequest> should match
// service_traits::DataType< ::pedsim_srvs::SpawnObstacles >
template<>
struct DataType< ::pedsim_srvs::SpawnObstaclesRequest>
{
  static const char* value()
  {
    return DataType< ::pedsim_srvs::SpawnObstacles >::value();
  }
  static const char* value(const ::pedsim_srvs::SpawnObstaclesRequest&)
  {
    return value();
  }
};

// service_traits::MD5Sum< ::pedsim_srvs::SpawnObstaclesResponse> should match
// service_traits::MD5Sum< ::pedsim_srvs::SpawnObstacles >
template<>
struct MD5Sum< ::pedsim_srvs::SpawnObstaclesResponse>
{
  static const char* value()
  {
    return MD5Sum< ::pedsim_srvs::SpawnObstacles >::value();
  }
  static const char* value(const ::pedsim_srvs::SpawnObstaclesResponse&)
  {
    return value();
  }
};

// service_traits::DataType< ::pedsim_srvs::SpawnObstaclesResponse> should match
// service_traits::DataType< ::pedsim_srvs::SpawnObstacles >
template<>
struct DataType< ::pedsim_srvs::SpawnObstaclesResponse>
{
  static const char* value()
  {
    return DataType< ::pedsim_srvs::SpawnObstacles >::value();
  }
  static const char* value(const ::pedsim_srvs::SpawnObstaclesResponse&)
  {
    return value();
  }
};

} // namespace service_traits
} // namespace ros

#endif // PEDSIM_SRVS_MESSAGE_SPAWNOBSTACLES_H
