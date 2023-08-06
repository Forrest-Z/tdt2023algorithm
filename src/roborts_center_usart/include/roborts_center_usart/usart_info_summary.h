#ifndef __USART_INFO_SUMMARY_H
#define __USART_INFO_SUMMARY_H

#include <errno.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>

#include <boost/asio.hpp>
#include <deque>
#include <mutex>
#include <vector>

#include "recver/commander_usart_recver.h"
#include "recver/gyroscope_usart_recver.h"
#include "recver/match_info_usart_recver.h"
#include "recver/vision_usart_recever.h"
#include "recver/speed_usart_recver.h"
#include "sender/command_reply_usart_sender.h"
#include "sender/navigation_usart_sender.h"
#include "sender/vision_usart_sender.h"

namespace tdtusart {
namespace shared_data {
static const int DataRecverNum = 5;  // gyroscope ,  command , vision, matchInfo speedUsart
static BaseUsartRecver *DataRecver[DataRecverNum + 1] = {
    nullptr, (BaseUsartRecver *)(new GyroscopeUsartRecver()),
    (BaseUsartRecver *)(new CommanderUsartRecver()),
    (BaseUsartRecver *)(new VisionUsartRecver()),
    (BaseUsartRecver *)(new MatchInfoUsartRecver()),
    (BaseUsartRecver *)(new SpaeedUsartRecver())};

static const int DataSenderNum = 3;  // naviGation , vision, commandReply
static BaseUsartSender *DataSender[DataSenderNum + 1] = {
    nullptr, (BaseUsartSender *)(new NavigationUsartSender()),
    (BaseUsartSender *)(new VisionUsartSender()),
    (BaseUsartSender *)(new CommandReplyUsartSender())};

}  // namespace shared_data

}  // namespace tdtusart
#endif