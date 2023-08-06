/*
 * @Description: 用于致命或重要的条件判断，并结束进程,说点实例
 * @example: 像为了避免某个地方的一个队列在使用前size为0,judge::fatal(cloud_.empty());
 */

#ifndef CONDITION_JUDGEMENT_HPP
#define CONDITION_JUDGEMENT_HPP

// c++
#include <string>
#include "rclcpp/rclcpp.hpp"
#include "color_terminal.hpp"
class judge
  {
  public:
    judge(){;}

    static void fatal(bool check,const std::string info)
    {
      if(check)
        {
            ColorTerminal::red(info+"的fatal情况出现");
            rclcpp::shutdown();
            exit(0);
        }
    }


  };

#endif