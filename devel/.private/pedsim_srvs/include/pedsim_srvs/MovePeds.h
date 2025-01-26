// Generated by gencpp from file pedsim_srvs/MovePeds.msg
// DO NOT EDIT!


#ifndef PEDSIM_SRVS_MESSAGE_MOVEPEDS_H
#define PEDSIM_SRVS_MESSAGE_MOVEPEDS_H

#include <ros/service_traits.h>


#include <pedsim_srvs/MovePedsRequest.h>
#include <pedsim_srvs/MovePedsResponse.h>


namespace pedsim_srvs
{

struct MovePeds
{

typedef MovePedsRequest Request;
typedef MovePedsResponse Response;
Request request;
Response response;

typedef Request RequestType;
typedef Response ResponseType;

}; // struct MovePeds
} // namespace pedsim_srvs


namespace ros
{
namespace service_traits
{


template<>
struct MD5Sum< ::pedsim_srvs::MovePeds > {
  static const char* value()
  {
    return "7e70f76cdefe8786e8c6f0a9a54a958a";
  }

  static const char* value(const ::pedsim_srvs::MovePeds&) { return value(); }
};

template<>
struct DataType< ::pedsim_srvs::MovePeds > {
  static const char* value()
  {
    return "pedsim_srvs/MovePeds";
  }

  static const char* value(const ::pedsim_srvs::MovePeds&) { return value(); }
};


// service_traits::MD5Sum< ::pedsim_srvs::MovePedsRequest> should match
// service_traits::MD5Sum< ::pedsim_srvs::MovePeds >
template<>
struct MD5Sum< ::pedsim_srvs::MovePedsRequest>
{
  static const char* value()
  {
    return MD5Sum< ::pedsim_srvs::MovePeds >::value();
  }
  static const char* value(const ::pedsim_srvs::MovePedsRequest&)
  {
    return value();
  }
};

// service_traits::DataType< ::pedsim_srvs::MovePedsRequest> should match
// service_traits::DataType< ::pedsim_srvs::MovePeds >
template<>
struct DataType< ::pedsim_srvs::MovePedsRequest>
{
  static const char* value()
  {
    return DataType< ::pedsim_srvs::MovePeds >::value();
  }
  static const char* value(const ::pedsim_srvs::MovePedsRequest&)
  {
    return value();
  }
};

// service_traits::MD5Sum< ::pedsim_srvs::MovePedsResponse> should match
// service_traits::MD5Sum< ::pedsim_srvs::MovePeds >
template<>
struct MD5Sum< ::pedsim_srvs::MovePedsResponse>
{
  static const char* value()
  {
    return MD5Sum< ::pedsim_srvs::MovePeds >::value();
  }
  static const char* value(const ::pedsim_srvs::MovePedsResponse&)
  {
    return value();
  }
};

// service_traits::DataType< ::pedsim_srvs::MovePedsResponse> should match
// service_traits::DataType< ::pedsim_srvs::MovePeds >
template<>
struct DataType< ::pedsim_srvs::MovePedsResponse>
{
  static const char* value()
  {
    return DataType< ::pedsim_srvs::MovePeds >::value();
  }
  static const char* value(const ::pedsim_srvs::MovePedsResponse&)
  {
    return value();
  }
};

} // namespace service_traits
} // namespace ros

#endif // PEDSIM_SRVS_MESSAGE_MOVEPEDS_H
