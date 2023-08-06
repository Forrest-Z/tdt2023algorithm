/*
 * @Description: 
 */
#ifndef TIMESTAMP_TRANS_HPP_
#define TIMESTAMP_TRANS_HPP_
#pragma once

#include <ctime>
#include <rclcpp/rclcpp.hpp>

  inline double time_to_double(builtin_interfaces::msg::Time ros2_time)
  {
    return (ros2_time.sec + (ros2_time.nanosec * 1e-9));
  }

  inline builtin_interfaces::msg::Time double_to_time(double time)
  {
      builtin_interfaces::msg::Time ros2_time;
      int32_t trans_sec = (int32_t)time;
      uint32_t trans_nanosec = (time-trans_sec)* 1e+9;    
      ros2_time.sec = trans_sec;ros2_time.nanosec = trans_nanosec;
    return ros2_time;
  }


#endif
