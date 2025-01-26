// Generated by gencpp from file all_in_one_global_planner_interface/MakeNewPlan.msg
// DO NOT EDIT!


#ifndef ALL_IN_ONE_GLOBAL_PLANNER_INTERFACE_MESSAGE_MAKENEWPLAN_H
#define ALL_IN_ONE_GLOBAL_PLANNER_INTERFACE_MESSAGE_MAKENEWPLAN_H

#include <ros/service_traits.h>


#include <all_in_one_global_planner_interface/MakeNewPlanRequest.h>
#include <all_in_one_global_planner_interface/MakeNewPlanResponse.h>


namespace all_in_one_global_planner_interface
{

struct MakeNewPlan
{

typedef MakeNewPlanRequest Request;
typedef MakeNewPlanResponse Response;
Request request;
Response response;

typedef Request RequestType;
typedef Response ResponseType;

}; // struct MakeNewPlan
} // namespace all_in_one_global_planner_interface


namespace ros
{
namespace service_traits
{


template<>
struct MD5Sum< ::all_in_one_global_planner_interface::MakeNewPlan > {
  static const char* value()
  {
    return "b0da0fe6b6750964de6877ef99c1e149";
  }

  static const char* value(const ::all_in_one_global_planner_interface::MakeNewPlan&) { return value(); }
};

template<>
struct DataType< ::all_in_one_global_planner_interface::MakeNewPlan > {
  static const char* value()
  {
    return "all_in_one_global_planner_interface/MakeNewPlan";
  }

  static const char* value(const ::all_in_one_global_planner_interface::MakeNewPlan&) { return value(); }
};


// service_traits::MD5Sum< ::all_in_one_global_planner_interface::MakeNewPlanRequest> should match
// service_traits::MD5Sum< ::all_in_one_global_planner_interface::MakeNewPlan >
template<>
struct MD5Sum< ::all_in_one_global_planner_interface::MakeNewPlanRequest>
{
  static const char* value()
  {
    return MD5Sum< ::all_in_one_global_planner_interface::MakeNewPlan >::value();
  }
  static const char* value(const ::all_in_one_global_planner_interface::MakeNewPlanRequest&)
  {
    return value();
  }
};

// service_traits::DataType< ::all_in_one_global_planner_interface::MakeNewPlanRequest> should match
// service_traits::DataType< ::all_in_one_global_planner_interface::MakeNewPlan >
template<>
struct DataType< ::all_in_one_global_planner_interface::MakeNewPlanRequest>
{
  static const char* value()
  {
    return DataType< ::all_in_one_global_planner_interface::MakeNewPlan >::value();
  }
  static const char* value(const ::all_in_one_global_planner_interface::MakeNewPlanRequest&)
  {
    return value();
  }
};

// service_traits::MD5Sum< ::all_in_one_global_planner_interface::MakeNewPlanResponse> should match
// service_traits::MD5Sum< ::all_in_one_global_planner_interface::MakeNewPlan >
template<>
struct MD5Sum< ::all_in_one_global_planner_interface::MakeNewPlanResponse>
{
  static const char* value()
  {
    return MD5Sum< ::all_in_one_global_planner_interface::MakeNewPlan >::value();
  }
  static const char* value(const ::all_in_one_global_planner_interface::MakeNewPlanResponse&)
  {
    return value();
  }
};

// service_traits::DataType< ::all_in_one_global_planner_interface::MakeNewPlanResponse> should match
// service_traits::DataType< ::all_in_one_global_planner_interface::MakeNewPlan >
template<>
struct DataType< ::all_in_one_global_planner_interface::MakeNewPlanResponse>
{
  static const char* value()
  {
    return DataType< ::all_in_one_global_planner_interface::MakeNewPlan >::value();
  }
  static const char* value(const ::all_in_one_global_planner_interface::MakeNewPlanResponse&)
  {
    return value();
  }
};

} // namespace service_traits
} // namespace ros

#endif // ALL_IN_ONE_GLOBAL_PLANNER_INTERFACE_MESSAGE_MAKENEWPLAN_H
