#include "node_debug/node_debug.h"
#include <cstddef>
#include <iterator>
#include <string>


namespace node_debug {

NodeDebug::NodeDebug(rclcpp::Node::SharedPtr &debug_node){
  
  debug_node_=debug_node; 

  RCLCPP_INFO(debug_node_->get_logger(),
              "VISION_DEUBG_NODE has created !");

    debug_pub_ = debug_node_->create_publisher<vision_interface::msg::VisiondebugInfo>(
      "vision_debug_info", 1);

    image_sub_ = debug_node_->create_subscription<vision_interface::msg::ImageCompressed>(
      "image_compressed", 1,
      std::bind(&NodeDebug::ImageCallback, this, std::placeholders::_1));
  
    group_sub_ = debug_node_->create_subscription<vision_interface::msg::GroupMsgs>(
        "group_msgs", 1,
        std::bind(&NodeDebug::GroupCallback, this, std::placeholders::_1));

    param_request = debug_node_->create_service<vision_interface::srv::RequestVisionParam>(
        "request_vision_param",
        std::bind(&NodeDebug::ParamRequestCallback, this, std::placeholders::_1, std::placeholders::_2));

    param_apply = debug_node_->create_service<vision_interface::srv::ApplyVisionParam>(
        "apply_vision_param",
        std::bind(&NodeDebug::ParamApplyCallback, this, std::placeholders::_1, std::placeholders::_2));     

}

void NodeDebug::ImageCallback(const vision_interface::msg::ImageCompressed::SharedPtr msg){
    image_deque_.push_back(*msg);

    if(image_deque_.size()>image_max_deque_size){
        image_deque_.pop_front();
    }
}

void NodeDebug::GroupCallback(const vision_interface::msg::GroupMsgs::SharedPtr msg){
    group_msgs_deque_.push_back(*msg);

    if(group_msgs_deque_.size()>group_max_deque_size){
        group_msgs_deque_.pop_front();
    }
}

void NodeDebug::ParamRequestCallback(const vision_interface::srv::RequestVisionParam::Request::SharedPtr request,
                                  const vision_interface::srv::RequestVisionParam::Response::SharedPtr response){
                                  
  if(request->request_param==true){

    tdttoolkit::Debug_param::sendParamList("./config/debug_param.yaml",response);
    }
}

void NodeDebug::ParamApplyCallback(const vision_interface::srv::ApplyVisionParam::Request::SharedPtr request,
                                const vision_interface::srv::ApplyVisionParam::Response::SharedPtr response){
                                
                                

}

void NodeDebug::matching_publish(int delay){
  
    if(image_deque_.empty()){
       return; 
    }

    else{
        
        while(image_deque_.back().seq -image_deque_.front().seq >= delay){
                
                vision_interface::msg::VisiondebugInfo debug_info;
                debug_info.frame_seq = image_deque_.front().seq;
                debug_info.image = image_deque_.front().image;
                image_deque_.pop_front();

                // vision_interface::msg::GroupMsgs group_msgs;

                //此处需要进行匹配,并对配对成功的deque进行pop操作（小于该帧序号的舍弃，匹配相同帧序号）
                
                while(group_msgs_deque_.size()&&group_msgs_deque_.front().frame_seq  <= debug_info.frame_seq){
                   
                    if(group_msgs_deque_.front().frame_seq < debug_info.frame_seq){
                        // std::cout<<"group_msgs_deque_seq:"<<group_msgs_deque_.front().frame_seq<<"  debug_info_seq:"<<debug_info.frame_seq<<std::endl;
                        group_msgs_deque_.pop_front();
                    }
                    else{//从此处进行特征的匹配

                        if(group_msgs_deque_.front().rects.size()){
                            debug_info.group_msgs.rects.insert(debug_info.group_msgs.rects.end(),group_msgs_deque_.front().rects.begin(),group_msgs_deque_.front().rects.end());
                        }

                        if(group_msgs_deque_.front().circles.size()){
                            debug_info.group_msgs.circles.insert(debug_info.group_msgs.circles.end(),group_msgs_deque_.front().circles.begin(),group_msgs_deque_.front().circles.end());
                        }

                        if(group_msgs_deque_.front().lines.size()){
                            debug_info.group_msgs.lines.insert(debug_info.group_msgs.lines.end(),group_msgs_deque_.front().lines.begin(),group_msgs_deque_.front().lines.end());
                        }
                        
                        if(group_msgs_deque_.front().texts.size()){
                            debug_info.group_msgs.texts.insert(debug_info.group_msgs.texts.end(),group_msgs_deque_.front().texts.begin(),group_msgs_deque_.front().texts.end());
                        }

                        if(group_msgs_deque_.front().params.size()){
                            debug_info.group_msgs.params.insert(debug_info.group_msgs.params.end(),group_msgs_deque_.front().params.begin(),group_msgs_deque_.front().params.end());
                        }

                        // RCLCPP_INFO(debug_node_->get_logger(),"group_msgs:%d",group_msgs_deque_.front().circles.size());
                        group_msgs_deque_.pop_front();
                    }

                }

                // debug_info.group_msgs = group_msgs;

                debug_pub_->publish(debug_info);

                RCLCPP_INFO(debug_node_->get_logger(),"image:%d",debug_info.frame_seq);
                std::cout<<std::endl;
        }

      




    }

}

}