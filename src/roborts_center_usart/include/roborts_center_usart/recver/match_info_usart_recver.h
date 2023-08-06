#ifndef __MATCH_INFO_USART_RECVER_H
#define __MATCH_INFO_USART_RECVER_H

#include <errno.h>
#include <roborts_utils/base_blackboard.h>
#include <roborts_utils/base_class.h>
#include <roborts_utils/base_toolkit.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>

#include <chrono>
#include <deque>
#include <mutex>
#include <rclcpp/client.hpp>
#include <std_srvs/srv/set_bool.hpp>
#include <thread>
#include <vector>

#include "base_usart.h"
#include "navigation_interface/msg/nav_decision_command.hpp"
#include "roborts_utils/roborts_utils.h"
#include "usart_black_board.h"

namespace tdtusart {
class MatchInfoUsartRecver : public BaseUsartRecver {
 public:
// clang-format off
#pragma pack(1)
  struct MatchInfoData { // 1s 一次
    uint8_t header = 0xA5;
    uint8_t type = 4;
    
    uint8_t self_color: 1 = 0; // 0 蓝 1 红
    int16_t match_time = -200; // 比赛时间，若尚未开始则发送比赛开始倒计时时间的负数，若比赛结束发送-100 若未连接至裁判系统发送-200
    uint8_t robot_hp[14]; // 0-6 1-7号敌方机器人血量 7-13 1-7号己方机器人血量
    uint8_t supper_electric_capacity_status = -1; // 超级电容状态 0-100 可用度
    uint16_t enemy_X[6]; // 1-6号敌方机器人x坐标
    uint16_t enemy_Y[6]; // 1-6号敌方机器人y坐标
    uint16_t friend_X[6]; // 1-6号己方机器人x坐标
    uint16_t friend_Y[6]; // 1-6号己方机器人y坐标

    float time_stamp;
    int16_t frame_id;
    uint16_t CRC16CheckSum;
  };
// clang-format on
#pragma pack()

  int GetStructLength() override { return sizeof(MatchInfoData); }

  void ParseData(void *message) override {
    MatchInfoData *commander_data = (MatchInfoData *)message;
    auto info = tdttoolkit::MatchInfo(
        commander_data->self_color, commander_data->match_time,
        commander_data->robot_hp,
        commander_data->supper_electric_capacity_status);
    tdttoolkit::BaseBlackboard::Set("match_info", info);

    TDT_DEBUG(
        "recv match_info color=%d, match_time=%d, "
        "supper_electric_capacity_status=%d, time_stamp=%f",
        commander_data->self_color, commander_data->match_time,
        commander_data->supper_electric_capacity_status,
        commander_data->time_stamp);
    if (!localization_start_) {
      if (commander_data->match_time > -5) {
        TDT_WARNING("launch localization");
        std::unique_lock<std::mutex> lock(localization_start_mutex_);
        localization_start_condition_.notify_all();
        lock.unlock();
      }
    }

    if (localization_start_ && commander_data->match_time == -100) {
      TDT_WARNING("Reset localization launch flag");
      localization_start_ = false;
      usartblackboard::localization_start_ = false;
      localization_start_thread_ =
          std::thread(&MatchInfoUsartRecver::localization_start_thread, this);
      localization_start_thread_.detach();
    }

    auto nav_status =
        tdttoolkit::BaseBlackboard::Get<tdttoolkit::NavStatus>("nav_status");
    auto cnt_time = tdttoolkit::Time::GetTimeNow() / 1e6;
    if (on_recovery && nav_status.specific_target !=
                           tdttoolkit::NavStatus::SpecificTarget::SUPPLY) {
      on_recovery = false;
    }
    TDT_DEBUG("Self sentry hp is %d", info.robot_hp[13]);
    // std::cout<<info.robot_hp[11] <<  info.robot_hp[11] < 200 <<
    // info.remaining_recoverable_health > 0 << info.match_time <<
    // (!on_recovery) <<  nav_status.specific_target !=
    //         tdttoolkit::NavStatus::SpecificTarget::SUPPLY <<  (!recovered) <<
    //         std::endl;
    //
    // if (info.robot_hp[11] > 0 && info.robot_hp[11] < 200 &&
    //     info.remaining_recoverable_health > 0 && info.match_time < 240 &&
    //     (!on_recovery) &&
    //     nav_status.specific_target !=
    //         tdttoolkit::NavStatus::SpecificTarget::SUPPLY &&
    //     (!recovered)) {
    //   TDT_WARNING("哨兵前往补给区");
    //   on_recovery = true;
    //   if (nav_status.nav_target.x == -1 && nav_status.nav_target.y == -1) {
    //     marked_last_nav_target = nav_status.nav_pos;
    //   } else {
    //     marked_last_nav_target = nav_status.nav_target;
    //   }
    //   navigation_interface::msg::NavDecisionCommand nav_command;
    //   nav_command.specific_point_id =
    //       tdttoolkit::NavStatus::SpecificTarget::SUPPLY;  // 补给区
    //   nav_command.point_x = -1;
    //   nav_command.point_y = -1;
    //   nav_command.header.stamp = rclcpp::Clock().now();
    //   nav_command_publisher_->publish(nav_command);
    //   nav_status.on_arrive = false;
    //   tdttoolkit::BaseBlackboard::Set("nav_status", nav_status);
    // }
    // if (on_recovery && nav_status.on_arrive && supply_arrive_time < 0) {
    //   supply_arrive_time = cnt_time;
    // }
    // if (on_recovery &&
    //     ((info.robot_hp[11] > 580) ||
    //      (info.remaining_recoverable_health < 10) ||
    //      (cnt_time - supply_arrive_time > 6 && supply_arrive_time > 0))) {
    //   TDT_WARNING("哨兵从补给区回到原位置");
    //   on_recovery = false;
    //   navigation_interface::msg::NavDecisionCommand nav_command;
    //   nav_command.specific_point_id =
    //       tdttoolkit::NavStatus::SpecificTarget::NONE;
    //   nav_command.point_x = marked_last_nav_target.x;
    //   nav_command.point_y = marked_last_nav_target.y;
    //   nav_command.header.stamp = rclcpp::Clock().now();
    //   nav_command_publisher_->publish(nav_command);
    //   recovered = true;
    // }
  }

  void init_communicator(std::shared_ptr<rclcpp::Node> &node) override {
    localization_start_client_ =
        node->create_client<std_srvs::srv::SetBool>("localization_start");
    localization_start_thread_ =
        std::thread(&MatchInfoUsartRecver::localization_start_thread, this);
    localization_start_thread_.detach();

    nav_command_publisher_ =
        node->create_publisher<navigation_interface::msg::NavDecisionCommand>(
            "nav_decision_command", 10);
  }

  void localization_start_thread() {
    while (!localization_start_) {
      std::unique_lock<std::mutex> lock(localization_start_mutex_);
      localization_start_condition_.wait(lock);
      lock.unlock();
      auto request = std::make_shared<std_srvs::srv::SetBool::Request>();
      request->data = true;
      while (!localization_start_client_->wait_for_service(
          std::chrono::milliseconds(500))) {
        TDT_INFO("localization start service not available, waiting again...");
        if (!rclcpp::ok()) {
          TDT_FATAL("Interrupted while waiting for the service. Exiting.");
          return;
        }
      }
      auto response_received_callback =
          [this](rclcpp::Client<std_srvs::srv::SetBool>::SharedFuture result) {
            auto msg = result.get();
            if (msg->success) {
              localization_start_ = true;
              TDT_INFO(msg->message.c_str());
              usartblackboard::localization_start_ = true;
            } else {
              TDT_ERROR("localization start failed");
              TDT_ERROR(msg->message.c_str());
            }
          };
      auto future_result = localization_start_client_->async_send_request(
          request, response_received_callback);
      std::this_thread::sleep_for(std::chrono::milliseconds(300));
    }
  }

 private:
  bool localization_start_ = false;
  std::mutex localization_start_mutex_;
  std::condition_variable localization_start_condition_;
  rclcpp::Client<std_srvs::srv::SetBool>::SharedPtr localization_start_client_;

  std::thread localization_start_thread_;

  tdttoolkit::Vec2d marked_last_nav_target;
  bool on_recovery = false;
  bool recovered = false;
  double supply_arrive_time = -1;
  rclcpp::Publisher<navigation_interface::msg::NavDecisionCommand>::SharedPtr
      nav_command_publisher_;
};
}  // namespace tdtusart

#endif