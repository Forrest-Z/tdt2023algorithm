#include "armor_detect/armor_detect.h"

namespace armor_detect {

// balance
int ArmorDetector::balance_on = -1;
int ArmorDetector::balance_[3]{0, 0, 0};  // static
int ArmorDetector::balance_counter[3][2]{{0, 0}, {0, 0}, {0, 0}};

int ifdebug = 0;
int sdebug = 0;
ReceiveMessage receiveMessage;

ArmorDetector::ArmorDetector(rclcpp::Node::SharedPtr &detect_node) {
  detect_node_ = detect_node;
  RCLCPP_INFO(detect_node_->get_logger(), "VISION_ARMOR_DETECT has created !");
  init();

  //   image_sub =
  //   detect_node_->create_subscription<vision_interface::msg::ImagePub>(
  //       "image_raw", 1,
  //       std::bind(&ArmorDetector::imageCallback, this,
  //       std::placeholders::_1));

  usart_Pub =
      detect_node_->create_subscription<vision_interface::msg::UsartPub>(
          "visionUsartPub", 1,
          std::bind(&ArmorDetector::usartCallback, this,
                    std::placeholders::_1));

  armor_data_pub_ =
      detect_node_->create_publisher<vision_interface::msg::DetectPub>(
          "armor_detection", 1);

  qtDebug = detect_node_->create_publisher<vision_interface::msg::GroupMsgs>(
      "group_msgs", 1);

  paramFeedBack_sub =
      detect_node_->create_subscription<base_interface::msg::FeedBackParam>(
          "paramFeedBack", 1,
          std::bind(&ArmorDetector::paramFeedBackCallback, this,
                    std::placeholders::_1));
  compressed_image_pub =
      detect_node_->create_publisher<vision_interface::msg::ImageCompressed>(
          "image_compressed", 1);
}

void ArmorDetector::init() {
  std::vector<ArmorDetectInfo>().swap(this->last_armors_info_);
  initParam();
  this->armor_detect_roi_ =
      cv::Rect2i(cv::Point2i(0, 0), cv::Size2i(src_width, src_height));
  this->src_area_ =
      cv::Rect2i(cv::Point2i(0, 0), cv::Size2i(src_width, src_height));
  this->element_ = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(2, 1));
  this->element2_ = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(3, 3));
#ifdef use_openvino
  if (this->deeplearning_mode == 0) {
    if (this->numberDetector == nullptr) {
      this->numberDetector = new tdtml::NumberDetector();
      std::string model_path;
      LoadParam::ReadParam("detect", "NumberDetectPath", model_path);
      this->numberDetector->init(model_path, 0);
    }
  } else if (this->deeplearning_mode == 1) {
    if (this->yolo_detector == nullptr) {
      /**
       * 初始化yolo
       */
      this->yolo_detector = new tdtml::YoloDetector;
      // TODO 有待改进，这个地方的参数最好还是写在yaml里面

      std::string model_path;
      LoadParam::ReadParam("detect", "DeepLearningModePath", model_path);

      this->yolo_detector->init(model_path, dp_cof_thre, dp_nms_area_thre,
                                this->src_width, this->src_height,
                                this->gpu_mode);
    }
    if (this->openvino_number_detector == nullptr) {
      this->openvino_number_detector = new tdtml::openvino_number_detector();
      std::string model_path;
      LoadParam::ReadParam("detect", "YoloNumberDetectPath", model_path);
      this->openvino_number_detector->init(model_path, 0);
    }
  }
#endif

#ifdef use_tensorrt
  int srcH;
  int srcW;
  bool gen_engine;
  std::string yolo_engine_name;
  std::string yolo_onnx_name;
  float kNmsThresh;
  float kConfThresh;
  int kBatchSize;
  LoadParam::ReadParam("detect", "srcH", srcH);
  LoadParam::ReadParam("detect", "srcW", srcW);
  LoadParam::ReadParam("detect", "gen_engine", gen_engine);
  LoadParam::ReadParam("detect", "yolo_engine_name", yolo_engine_name);
  LoadParam::ReadParam("detect", "yolo_onnx_name", yolo_onnx_name);
  LoadParam::ReadParam("detect", "kNmsThresh", kNmsThresh);
  LoadParam::ReadParam("detect", "kConfThresh", kConfThresh);
  LoadParam::ReadParam("detect", "kBatchSize", kBatchSize);
  if (this->trt_yolo_detector == nullptr) {
    /**
     * 初始化yolo
     */
    this->trt_yolo_detector = new trt_detector::trt_yolo_detector;
    this->trt_yolo_detector->init(srcH, srcW, gen_engine, yolo_engine_name,
                                  yolo_onnx_name, kNmsThresh, kConfThresh,
                                  kBatchSize);
  }

  std::string number_engine_name;
  std::string number_onnx_name;
  LoadParam::ReadParam("detect", "number_engine_name", number_engine_name);
  LoadParam::ReadParam("detect", "number_onnx_name", number_onnx_name);

  if (this->trt_number_detector == nullptr) {
    this->trt_number_detector = new trt_detector::trt_number_detector;
    this->trt_number_detector->init(number_engine_name, number_onnx_name,
                                    gen_engine);
  }
#endif
}

void ArmorDetector::initParam() {
  LoadParam::ReadParam(param_name, "EnemyColor", this->enemy_color_);
  std::cout << "enemy_color_:" << enemy_color_ << std::endl;
  LoadParam::ReadParam(param_name, "ArmorDetect_lightbarthre",
                       this->threshold_);
  LoadParam::ReadParam(param_name, "Baitian", this->baitian_);
  LoadParam::ReadParam(param_name, "Lock", this->lock_);
  LoadParam::ReadParam(param_name, "DeepLearningMode", this->deeplearning_mode);
  LoadParam::ReadParam(param_name, "GpuMode", this->gpu_mode);
  LoadParam::ReadParam(param_name, "SrcHeight", this->src_height);
  LoadParam::ReadParam(param_name, "SrcWidth", this->src_width);
  LoadParam::ReadParam(param_name, "Balance_on", this->balance_on);
  LoadParam::ReadParam(param_name, "DeepLearning_cof_thre", this->dp_cof_thre);
  LoadParam::ReadParam(param_name, "DeepLearning_nms_area_thre",
                       this->dp_nms_area_thre);
}
void ArmorDetector::detect(const vision_interface::msg::ImagePub &img_msg) {
  auto image_ptr = std::make_shared<sensor_msgs::msg::Image>(img_msg.image);
  cv::Mat img = cv_bridge::toCvShare(image_ptr, "bgr8")->image;
  Armors = this->Get(img);

  if (tdtconfig::VIDEODEBUG) {
    vision_interface::msg::ImageCompressed compressed_image_msg;
    compressed_image_msg.image.header = img_msg.image.header;

    compressed_image_msg.seq = img_msg.seq;

    compressed_image_msg.image.format = "jpeg";
    // compressed_image_msg.image.data.size = image_msg.image.data.size() / 3;

    cv::imencode(".jpg", cv::Mat(img.rows, img.cols, CV_8UC3, img.data),

                 compressed_image_msg.image.data);

    compressed_image_pub->publish(compressed_image_msg);
  }

  publish2resolve(img_msg.seq, img_msg.yaw, img_msg.pitch);
}

void ArmorDetector::usartCallback(
    const vision_interface::msg::UsartPub &usart_msg) {
  recvMsg = usart_msg;
}

void ArmorDetector::paramFeedBackCallback(
    const base_interface::msg::FeedBackParam &param_msg) {
  //更新参数
  for (auto &param : param_msg.params) {
    LoadParam::WriteParamString(param_name, param.item.name, param.item.type,
                                param.item.reference);
  }
  initParam();
  if (param_msg.ifsave) {
    LoadParam::OutPutParam(param_name);
  }
}

void ArmorDetector::publish2resolve(
    uint32_t seq, float platForm_yaw,  // platForm_yaw为相机取帧时刻的yaw ,
                                       // recvMsg.yaw为识别节点回调接收到的yaw
    float platForm_pitch) {
  // RCLCPP_INFO(detect_node_->get_logger(), "%ld", Armors.size());

  //   predictPub->publish(sendMsg); //no_obj 默认为 1
  //   return;
  // }

  // RCLCPP_INFO(detect_node_->get_logger(), "%ld", Armors.size());

  auto message = vision_interface::msg::DetectPub();

  message.seq = seq;
  message.header.stamp = detect_node_->now();
  message.header.frame_id = "armor_detect";
  message.num = Armors.size();
  // std::cout << "armorNum= " << (int)message.num << std::endl;

  if (Armors.size()) {
    std::vector<vision_interface::msg::DetectArmor> armors(Armors.size());
    auto debug_msg = vision_interface::msg::GroupMsgs();

    debug_msg.frame_seq = seq;

    for (int i = 0; i < Armors.size(); i++) {
      armors[i].armortype = (int)Armors[i].GetRobotType();
      std::cout << "type= " << (int)Armors[i].GetRobotType() << std::endl;
      Debug::DisplayParam(debug_msg, "Detect_type", "armor_type",
                          std::to_string((int)Armors[i].GetRobotType()));
      Debug::drawTxt(debug_msg, "Num",
                     cv::Point2f(Armors[i].GetImagePointsList()[1].x + 5,
                                 Armors[i].GetImagePointsList()[1].y),
                     std::to_string(armors[i].armortype), 20, 3);

      int size = Armors[i].GetImagePointsList().size();
      for (int j = 0; j < size; j++) {
        armors[i].armor_point_image[j].x = Armors[i].GetImagePointsList()[j].x;
        armors[i].armor_point_image[j].y = Armors[i].GetImagePointsList()[j].y;

        tdttoolkit::Debug::drawPoint(debug_msg, "lightbar",
                                     Armors[i].GetImagePointsList()[j], 3, 3,
                                     cv::Scalar(255, 0, 0));

        armors[i].armor_point_realword[j].x =
            Armors[i].GetWorldPointsList()[j].x;
        armors[i].armor_point_realword[j].y =
            Armors[i].GetWorldPointsList()[j].y;
        armors[i].armor_point_realword[j].z =
            Armors[i].GetWorldPointsList()[j].z;
      }
      armors[i].robot_center_image.x = Armors[i].Getaxis().x;
      armors[i].robot_center_image.y = Armors[i].Getaxis().y;
    }

    message.robot_armors = armors;

    qtDebug->publish(debug_msg);
  }

  message.plat_form_yaw = platForm_yaw;
  message.plat_form_pitch = platForm_pitch;

  message.mode = tdttoolkit::ARMOR_MODE;

  armor_data_pub_->publish(message);

  Armors.clear();
}

std::vector<tdttoolkit::RobotArmor> ArmorDetector::Get(cv::Mat &src) {
  //        cv::cvtColor(src,src,cv::COLOR_BGR2RGB);

  /*********************检测前数据更新************************/

  if (tdtconfig::VIDEODEBUG) {
    LoadParam::ReadParam("detect", "ArmorDetect_lightbarthre",
                         this->threshold_);
  }

  /*************************声明变量*************************/
  std::vector<ArmorDetectInfo> output_armors(0);
  std::vector<LightBarInfo> light_bars(0);
  std::vector<ArmorDetectInfo> armors(0);
  std::vector<tdttoolkit::RobotArmor> robot(0);
  std::vector<tdttoolkit::RobotArmor> outpost(0);
  std::vector<tdttoolkit::RobotArmor> output_armors_data_package(0);
  std::vector<tdttoolkit::RobotArmor> final_armors(0);
  /***************************检测**************************/
  if (!this->deeplearning_mode) {
    /**
     * 开启传统模式！！！
     */

    if (ifdebug) {
      std::cout << "开始灯条查找" << std::endl;
    }

    FindLightBar(src, light_bars);

    sort(ArmorDetector::trace_light_bars_.begin(),
         ArmorDetector::trace_light_bars_.end(),
         [](const LightBarInfo &a, const LightBarInfo &b) -> bool {
           return a.GetArea() > b.GetArea();
         });
    /**********************仅用于开火决策***********************/

    if (!light_bars.empty()) {
      // TODO 锁定操作手的选定的敌人
      if (ifdebug) {
        std::cout << "开始双灯条匹配" << std::endl;
      }
      // TODO 接下来的逻辑用于灯条的切换和使用，两种单灯条模式需要切换
      static int sentry = 0;
      DoubleLightBarDetect(src, light_bars, armors);
      std::cout << this->last_robot_type_ << std::endl;
      std::cout << sentry << std::endl;
      if (this->last_robot_type_ == 7) {
        if (ifdebug) {
          std::cout << "开始宽松单灯条匹配" << std::endl;
        }

        SingleLightBarDetect(src, light_bars, armors);
        sentry = 0;

      } else {
        if (sentry < 10) {
          if (ifdebug) {
            std::cout << "开始宽松单灯条匹配" << std::endl;
          }

          SingleLightBarDetect(src, light_bars, armors);
          sentry++;
        } else {
          if (ifdebug) {
            std::cout << "开始严格单灯条匹配" << std::endl;
          }
          SingleLightBarDetect_tough(src, light_bars, armors);
        }
      }
    }
    // if (armors.empty()) {
    //     NumberStickerDetect(src, armors);
    // }

    for (auto &i : armors) {
      // tdttoolkit::Debug::AddCustomRect("疑似装甲板",
      // i.GetArmorRotRect(),cv::Scalar(255, 0, 0));
    }
    // tdttoolkit::Debug::SetMessage("装甲板检测",
    // "机器学习前装甲板数量",int(armors.size()));

    // 使用机器学习预测装甲板
    NumPredict(src, armors);
    // if(!armors.empty())
    // std::cout<<armors[0].GetLeftBarRect().GetAngle()<<"******"<<armors[0].GetRightBarRect().GetAngle()<<std::endl;
    if (!armors.empty()) {
      if (armors.begin()->GetArmorType() != tdttoolkit::RobotType::TYPEUNKNOW) {
        last_robot_type_ = armors.begin()->GetArmorType();
      }
    }

    output_armors = armors;

    for (auto &armor_info : output_armors) {
      tdttoolkit::RobotArmor output_armor;
      ArmorTransform(armor_info, output_armor);
      final_armors.emplace_back(output_armor);
    }

  } else {
    /**
     * 开启深度学习模式
     * 为了满足接口，首先封装灯条，以两点决定长，以矩形框决定宽
     *
     */
#ifdef use_openvino
    vector<tdtml::YoloDetector::Object> detected_armors;
    vector<tdtml::YoloDetector::Object> detected_robots;
#endif
#ifdef use_tensorrt
    vector<trt_detector::trt_yolo_detector::Object> detected_armors;
    vector<trt_detector::trt_yolo_detector::Object> detected_robots;
#endif
    cv::Mat temp = src.clone();

    auto d_start = chrono::high_resolution_clock::now();
#ifdef use_openvino
    this->yolo_detector->process_frame(temp, detected_armors, detected_robots);
#endif
#ifdef use_tensorrt
    // cv::imshow("s",temp);
    // cv::waitKey(1);
    this->trt_yolo_detector->detect(temp, detected_armors, detected_robots);
#endif

    auto d_end = chrono::high_resolution_clock::now();

    std::chrono::duration<double> d_diff = d_end - d_start;

    d_avgtime = (d_avgtime * d_count + d_diff.count() * 1000) / (d_count + 1);
    ++d_count;

    TDT_INFO("yolo用时:%fms", d_diff * 1000);
    // cout << "yolo平均用时:" << d_avgtime << "ms" << endl;

    std::vector<RobotArmor> temp_robots(detected_robots.size());

    for (int i = 0; i < detected_robots.size(); i++) {
      safe_rect(detected_robots[i].rect);
      Point2f center = Point2f(
          detected_robots[i].rect.x + detected_robots[i].rect.width / 2,
          detected_robots[i].rect.y + detected_robots[i].rect.height / 2);

      temp_robots[i].Setaxis(center);
      temp_robots[i].Setrect(detected_robots[i].rect);
    }

    for (RobotArmor &temp_robot : temp_robots) {
      robot.push_back(temp_robot);
      // tdttoolkit::Debug::AddRect("输出装甲板",
      // temp_robot.Getrect(),cv::Scalar(125, 128, 128), 1);
    }

    for (int i = 0; i < detected_armors.size(); i++) {
      if (!inFrame(detected_armors[i].rect)) {
        detected_armors[i] = detected_armors.back();
        detected_armors.pop_back();
        i--;
        std::cout << "out of frame" << std::endl;
      }
    }
    roiFindLightbar(detected_armors, src);

    extractNumber(detected_armors, src);

    if (detected_armors.size() > 0) {
      auto n_start = chrono::high_resolution_clock::now();
#ifdef use_openvino
      this->openvino_number_detector->detect(detected_armors, src);
#endif

#ifdef use_tensorrt
      this->trt_number_detector->detect(detected_armors, src);
#endif

      auto n_end = chrono::high_resolution_clock::now();

      std::chrono::duration<double> n_diff = n_end - n_start;

      n_avgtime = (n_avgtime * n_count + n_diff.count() * 1000) / (n_count + 1);
      ++n_count;

      cout << "numberdetect:" << n_diff.count() * 1000 << "ms" << endl;
      cout << "numberdetect_avg:" << n_avgtime << "ms" << endl;
    }

    armors = GetArmorInfo(detected_armors);

    EraseDuplicate(armors);

    std::sort(robot.begin(), robot.end(),
              [](const tdttoolkit::RobotArmor &a,
                 const tdttoolkit::RobotArmor &b) -> bool {
                return a.Getrect().x < b.Getrect().x;
              });

    std::sort(output_armors.begin(), output_armors.end(),
              [](const ArmorDetectInfo &a, const ArmorDetectInfo &b) -> bool {
                return a.GetRect().x < b.GetRect().x;
              });

    output_armors_data_package = Info2Armor(armors);
    //匹配车筐和装甲板
    Judgement(robot, output_armors_data_package);
    //深度颜色类别只取敌方类别
    final_armors = GetenemyArmor(output_armors_data_package, src);

    //没车筐有两装甲板计算轴心
    CalAxis(final_armors);
  }

  DP_logicBalance(final_armors);
  for (auto &armor : final_armors) {
    armor.SetWorldPoints(armor.GetDPRobotType(), balance_);
  }
  /************************ARMOR_DETECT_DEBUG***************************/

  /************************检测后数据更新**********************/
  if (output_armors.empty()) {
    std::vector<ArmorDetectInfo>().swap(last_armors_info_);
  } else {
    last_armors_info_ = output_armors;
  }
  /***************************返回*************************/

  return final_armors;
}

#ifdef use_openvino
std::vector<ArmorDetectInfo> ArmorDetector::GetArmorInfo(
    std::vector<tdtml::YoloDetector::Object> &detected_armors) {
  std::vector<ArmorDetectInfo> armors(0);

  for (auto &detected_armor : detected_armors) {
    if (detected_armor.armor_id == 0) {
      continue;
    }
    std::vector<cv::Point> roi_points;

    roi_points.push_back(
        cv::Point(detected_armor.rect.x, detected_armor.rect.y));
    roi_points.push_back(
        cv::Point(detected_armor.rect.x,
                  detected_armor.rect.y + detected_armor.rect.height));
    roi_points.push_back(
        cv::Point(detected_armor.rect.x + detected_armor.rect.width,
                  detected_armor.rect.y));
    roi_points.push_back(
        cv::Point(detected_armor.rect.x + detected_armor.rect.width,
                  detected_armor.rect.y + detected_armor.rect.height));

    tdttoolkit::CustomRect roi_rect(roi_points);

    std::vector<cv::Point2f> light_bar[2];

    for (int j = 0; j < 8; j += 4) {
      std::vector<cv::Point2f> light_points;

      light_points.push_back(
          cv::Point2f(detected_armor.points[j], detected_armor.points[j + 1]));

      light_points.push_back(cv::Point2f(detected_armor.points[j + 2],
                                         detected_armor.points[j + 3]));

      light_bar[j / 4] = light_points;
    }
    // 不知道为什么矩形框特征没有了？？？
    ArmorDetectInfo temp_armor(light_bar[0], light_bar[1], roi_rect, 145);

    temp_armor.SetArmorRotRect(roi_rect);
    temp_armor.SetDPArmorType(tdttoolkit::DPRobotType(detected_armor.armor_id));
    temp_armor.Setconf(detected_armor.prob);
    armors.push_back(temp_armor);
  }
  return armors;
}
#endif

#ifdef use_tensorrt
std::vector<armor_detect::ArmorDetectInfo> ArmorDetector::GetArmorInfo(
    std::vector<trt_detector::trt_yolo_detector::Object> &detected_armors) {
  std::vector<armor_detect::ArmorDetectInfo> armors(0);

  for (auto &detected_armor : detected_armors) {
    if (detected_armor.armor_id == 0) {
      continue;
    }
    std::vector<cv::Point> roi_points;

    roi_points.push_back(
        cv::Point(detected_armor.rect.x, detected_armor.rect.y));
    roi_points.push_back(
        cv::Point(detected_armor.rect.x,
                  detected_armor.rect.y + detected_armor.rect.height));
    roi_points.push_back(
        cv::Point(detected_armor.rect.x + detected_armor.rect.width,
                  detected_armor.rect.y));
    roi_points.push_back(
        cv::Point(detected_armor.rect.x + detected_armor.rect.width,
                  detected_armor.rect.y + detected_armor.rect.height));

    tdttoolkit::CustomRect roi_rect(roi_points);

    std::vector<cv::Point2f> light_bar[2];

    for (int j = 0; j < 8; j += 4) {
      std::vector<cv::Point2f> light_points;

      light_points.push_back(
          cv::Point2f(detected_armor.points[j], detected_armor.points[j + 1]));

      light_points.push_back(cv::Point2f(detected_armor.points[j + 2],
                                         detected_armor.points[j + 3]));

      light_bar[j / 4] = light_points;
    }
    // 不知道为什么矩形框特征没有了？？？
    armor_detect::ArmorDetectInfo temp_armor(light_bar[0], light_bar[1],
                                             roi_rect, 145);

    temp_armor.SetArmorRotRect(roi_rect);
    temp_armor.SetDPArmorType(tdttoolkit::DPRobotType(detected_armor.armor_id));
    temp_armor.Setconf(detected_armor.prob);
    armors.push_back(temp_armor);
  }
  return armors;
}
#endif

std::vector<tdttoolkit::RobotArmor> ArmorDetector::Info2Armor(
    std::vector<ArmorDetectInfo> &armors) {
  std::vector<tdttoolkit::RobotArmor> output_armors_data_package;
  for (auto &armor_info : armors) {
    tdttoolkit::RobotArmor output_armor;
    DPArmorTransform(armor_info, output_armor, this->enemy_color_);
    output_armors_data_package.emplace_back(output_armor);
  }
  return output_armors_data_package;
}
std::vector<tdttoolkit::RobotArmor> ArmorDetector::GetenemyArmor(
    std::vector<tdttoolkit::RobotArmor> &output_armors_data_package,
    cv::Mat &src) {
  std::vector<tdttoolkit::RobotArmor> final_armors;
  for (auto const &armor : output_armors_data_package) {
    Rect2i temp_rect = armor.GetStickerRect().GetRect();
    if (!RectSafety(temp_rect)) continue;
    if (temp_rect.tl().x < 2 || temp_rect.br().x > (this->src_width - 2))
      continue;
    if (temp_rect.tl().y < 2 || temp_rect.br().y > (this->src_height - 2))
      continue;
    Mat rect, dst, photo;
    rect = src(temp_rect);
    cvtColor(rect, dst, COLOR_BGR2GRAY);
    cv::threshold(dst, dst, this->threshold_, 255, cv::THRESH_BINARY);
    bitwise_and(rect, rect, photo, dst);
    cv::Scalar area_sum = sum(photo);
    if ((area_sum[2 - enemy_color_] >= area_sum[enemy_color_])) {
      if (ifdebug) {
        std::cout << "模板颜色错误" << std::endl;
      }
      continue;
    }
    // if ((area_sum[0] > 0.8 * area_sum[2] &&
    //      area_sum[2] > 0.8 * area_sum[0]))  //紫色
    // {
    //   std::cout << "purple" << std::endl;
    //   continue;
    // }
    tdttoolkit::RobotArmor temp_ = armor;
    if (this->enemy_color_ < 1 && armor.GetDPRobotType() < 9) {
      temp_.SetRobotType(tdttoolkit::RobotType(armor.GetDPRobotType()));
      final_armors.push_back(temp_);
    }
    if (this->enemy_color_ > 1 && armor.GetDPRobotType() > 8) {
      temp_.SetRobotType(tdttoolkit::RobotType(armor.GetDPRobotType() - 8));
      final_armors.push_back(temp_);
    }
  }
  return final_armors;
}

void ArmorDetector::DP_FindLightBar(
    const cv::Mat &src, std::vector<LightBarInfo> &output_light_bars) {
  double t1 = cv::getTickCount();
  ////////////////////////////// 图像灯条查找预处理
  /////////////////////////////////
  cv::Mat img_gray;
  tdttoolkit::Lightbar_threshold(src, this->enemy_color_, img_gray,
                                 this->baitian_);
  // cv::threshold(img_gray, img_gray, this->threshold_, 255,
  // cv::THRESH_BINARY);
  cv::threshold(img_gray, img_gray, this->threshold_, 255,
                cv::THRESH_BINARY);  // 将灰度图二值化 寻找灯条解
  std::vector<std::vector<cv::Point2i>> contours;  // 轮廓容器

  // contours
  // 是一个vector嵌套vector,如果有机会可以用一些回归函数进行预测，从而减少下方的逻辑删除

  findContours(img_gray, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE,
               this->armor_detect_roi_.tl());  // 寻找轮廓
  for (int i = 0; i < contours.size(); i++) {
    tdttoolkit::CustomRect rot_rect(contours[i]);
    if (ifdebug) {
      std::cout << "所保存点的数量" << std::endl;
      std::cout << contours[i].size() << std::endl;
    }

    if (rot_rect.Vertical() < rot_rect.Cross() * 1.3) {
      if (ifdebug) {
        std::cout << "当前轮廓id:" << i << std::endl;

        std::cout << "灯条宽高比例错误" << std::endl;
        std::cout << "最小高度差:" << rot_rect.Vertical()
                  << "最小宽度差:" << rot_rect.Cross();
      }
      continue;
    }

    if (rot_rect.GetSize().area() < 6 || rot_rect.GetSize().area() > 51000) {
      // 面积太小或者太大, 扔掉
      if (ifdebug) {
        std::cout << "灯条面积面积错误" << std::endl;
      }
      continue;
    }
    if (rot_rect.GetAngle() > 155 || rot_rect.GetAngle() < 25) {
      if (ifdebug) {
        std::cout << "灯条角度错误" << std::endl;
      }
      continue;
    }
    if (rot_rect.GetWidth() > rot_rect.GetHeight() * 16) {
      if (ifdebug) {
        std::cout << "灯条另一种宽度比例错误" << std::endl;
        std::cout << rot_rect.GetWidth() << std::endl;
        std::cout << rot_rect.GetHeight() << std::endl;
      }
      continue;
    }

    LightBarInfo light_bar(rot_rect);
    output_light_bars.push_back(light_bar);
  }
  sort(output_light_bars.begin(), output_light_bars.end(),
       [](const LightBarInfo &a, const LightBarInfo &b) -> bool {
         return a.GetCenter().x < b.GetCenter().x;
       });
  if (ifdebug) {
    std::cout << "灯条数量" << output_light_bars.size() << std::endl;
  }
}

void ArmorDetector::EraseDuplicate(
    std::vector<ArmorDetectInfo> &output_armors) {
  if (!output_armors.empty()) {
    //        for (unsigned int i = 0; i < output_armors.size() - 1; i++) {
    //            for (unsigned int k = i + 1; k < output_armors.size();
    //            k++) {
    //                // std::cout<<"总长度："<<output_armors.size()<<"\n";
    //                //
    //                std::cout<<"开始遍历装甲板"<<i<<":"<<output_armors[i].GetArmorType()<<"
    //                "<<k<<":"<<output_armors[k].GetArmorType()<<"\n";
    //                // std::cout<<"相交面积:"<<(output_armors[i].GetRect()
    //                & output_armors[k].GetRect()).area()<<"\n";
    //
    //                if (!(output_armors[i].GetArmorRotRect().GetRect() &
    //                output_armors[k].GetArmorRotRect().GetRect()).empty())
    //                {
    //                    // std::cout<<"发现重复装甲板"<<"\n";
    //                    //  if (output_armors[i].GetDPArmorType() ==
    //                    output_armors[k].GetDPArmorType()) {
    //                    //
    //                    //
    //                    std::cout<<"本次重叠的装甲板类型："<<output_armors[i].GetArmorType()<<"//"<<output_armors[k].GetArmorType()<<"\n";
    //                    // 还有待改进过，应该是选择相似程度最高的
    //                    //
    //                    output_armors[((output_armors[i].GetArmorRotRect().GetRotRect().size.area()
    //                    >
    //                    output_armors[k].GetArmorRotRect().GetRotRect().size.area())
    //                    ? k :
    //                    i)].SetArmorType(tdttoolkit::RobotType::TYPEUNKNOW);
    //
    //                    //
    //                    if (output_armors[i].Getconf() <
    //                    output_armors[k].Getconf()) {
    //                        output_armors.erase(output_armors.begin() +
    //                        i); i--; break;
    //                    } else {
    //                        output_armors.erase(output_armors.begin() +
    //                        k); k--; continue;
    //                    }
    //
    //                    //
    //                    std::cout<<"输出概率:"<<i<<output_armors[i].Getconf()<<"输出概率:"<<k<<output_armors[k].Getconf()<<"判断结果"<<((output_armors[i].Getconf()
    //                    > output_armors[k].Getconf()) ? k : i)<<"\n";
    //                    // output_armors.erase(output_armors.begin() +
    //                    ((output_armors[i].GetArmorRotRect().GetRotRect().size.area()
    //                    <
    //                    output_armors[k].GetArmorRotRect().GetRotRect().size.area())
    //                    ? k : i));
    //                    // }else{
    //
    //                    //
    //                    if(output_armors[i].GetDPArmorType()==tdttoolkit::No){
    //                    // output_armors.erase(output_armors.begin()+i);
    //                    //     i--;
    //                    //     break;
    //                    // }
    //                    //
    //                    if(output_armors[k].GetDPArmorType()==tdttoolkit::No){
    //                    // output_armors.erase(output_armors.begin()+k);
    //                    //     k--;
    //                    //     continue;
    //                    // }
    //                    // }
    //                }
    //            }
    //        }

    vector<vector<int>> v(17);
    for (int i = 0; i < output_armors.size(); i++) {
      int type = output_armors[i].GetDPArmorType();
      if (type <= 16 && type >= 0) v[type].push_back(i);
    }
    for (int i = 0; i < v.size(); i++) {
      int maxArmorSize = 2;
      // if (i == 7 || i == 15) maxArmorSize = 1;

      if (v[i].size() > maxArmorSize) {
        std::vector<std::pair<ArmorDetectInfo, int>> temps;
        for (int j = 0; j < v[i].size(); j++) {
          std::pair<ArmorDetectInfo, int> temp =
              std::make_pair(output_armors[v[i][j]], v[i][j]);
          temps.push_back(temp);
        }
        sort(temps.begin(), temps.end(),
             [](std::pair<ArmorDetectInfo, int> &a,
                std::pair<ArmorDetectInfo, int> &b) -> bool {
               return (a.first.Getconf() > b.first.Getconf());
             });
        for (int k = maxArmorSize; k < v[i].size(); k++) {
          output_armors.erase(output_armors.begin() + temps[k].second);
        }
      }
    }
  }
}

int time, times;

float ArmorDetector::FastTargetLock(cv::Mat target_totest,
                                    CustomRect traget_rect, int test_id) {
  if (this->traget_image[test_id].empty()) {
    return 0.1;
  }
  // 考虑面积差距和位置差距：
  // 面积分数：
  float area_deviation = 0.2f *
                         fabs(this->traget_image[test_id].size().area() -
                              target_totest.size().area()) /
                         target_totest.size().area();
  float dist_deviation = 0.2f *
                         sqrt(pow(traget_rect.GetCenter().y -
                                      this->traget_rect[test_id].GetCenter().y,
                                  2) +
                              pow(traget_rect.GetCenter().x -
                                      this->traget_rect[test_id].GetCenter().x,
                                  2)) /
                         traget_rect.GetWidth();
  if (area_deviation + dist_deviation > 0.2) return false;
  static cv::HOGDescriptor *hog = new cv::HOGDescriptor(
      cv::Size(28, 28), cv::Size(14, 14), cv::Size(7, 7), cv::Size(7, 7), 9);
  std::vector<float> descriptors;
  hog->compute(target_totest, descriptors, cv::Size(1, 1), cv::Size(0, 0));
  cv::Mat des = cv::Mat(descriptors);
  cv::transpose(des, des);

  hog->compute(this->traget_image[test_id], descriptors, cv::Size(1, 1),
               cv::Size(0, 0));
  cv::Mat des2 = cv::Mat(descriptors);
  transpose(des2, des2);
  this->des[test_id] = des2.clone();

  float similarity;
  double dotSum = des2.dot(des);      // 内积
  double normFirst = cv::norm(des2);  // 取模
  double normSecond = cv::norm(des);
  if (normFirst != 0 && normSecond != 0) {
    similarity = static_cast<float>(dotSum / (normFirst * normSecond));
  } else {
    similarity = normFirst == normSecond ? 1 : 0;
  }
  // std::cout<<"\n减去偏移量之前的相似程度："<<similarity<<"\n";
  similarity -= area_deviation + dist_deviation;
  // std::cout<<similarity<<"\n";
  if (similarity > 0.90 && (test_id == 6 || test_id == 7)) {
    this->des[test_id] = des;
    this->traget_image[test_id] = target_totest.clone();
    this->traget_rect[test_id] = traget_rect;
  }
  return similarity;
}
void ArmorDetector::NumPredict(const cv::Mat &src,
                               std::vector<ArmorDetectInfo> &armors) {
  std::vector<cv::Mat> ml_rois;
  std::vector<int> results;
  std::vector<float> confs;
  int count = 0;
  for (ArmorDetectInfo &armor : armors) {
    cv::Mat armor_img;

    GetMlRoi(armor, src, armor_img);

    count++;
    ml_rois.push_back(armor_img);
  }

  struct timeval timek;
  gettimeofday(&timek, NULL);
  long int time1 = timek.tv_usec;
#ifdef use_openvino
  numberDetector->Predict(ml_rois, results, confs);
#endif
#ifdef use_tensorrt

#endif
  gettimeofday(&timek, NULL);
  long int time2 = timek.tv_usec;
  ////TODO 样本采集程序，请勿删除
  //        for(int i=0;i!=ml_rois.size();i++){
  //            static int a=0;
  //        a++;
  //        int seconds = std::time(NULL);
  //        a+=seconds;
  //        std::string dirname="/home/tdt/桌面/样本合集/采样文件夹/";
  //        dirname=dirname.append(std::to_string(results[i]));
  //        dirname=dirname.append("/");
  //        std::string number=std::to_string(a);
  //        std::string filename=dirname.append(number);
  //
  //        filename.append(".png");
  //        cv::imwrite(filename,ml_rois.at(i));
  //
  //        }

  TDT_INFO("机器学习：%d微秒", time2 - time1);

  Logic_fix(results, confs);

  // 追踪
  // TODO 追踪设计的目标：弥补灯条消失，机器学习失败带来的缺失
  gettimeofday(&timek, NULL);
  long int time5 = timek.tv_usec;

  ////追踪发现问题，可能会覆盖正确的机器学习答案
  if (!results.empty()) {
    for (int i = 0; i != results.size(); i++) {
      if (results[i] != 0 &&
          (results[i] == 1 || results[i] == 6 || results[i] == 7)) {
        cv::Mat temp = ml_rois[i].clone();

        this->traget_image[results[i]] = temp.clone();
        this->traget_rect[results[i]] = armors[i].GetArmorRotRect();
        this->id[results[i]] = results[i];
      } else {
        if (results[i] == 0) {
          cv::Mat temp = ml_rois[i].clone();

          //                std::cout<<"输出中心概率:";
          //                if(!this->traget_image.empty())
          //                cv::imshow("p2",this->traget_image);
          //                cv::imshow("n2",ml_rois[minarmor_id]);
          float similarity = -100;
          int id_temp = 0;
          for (int j = 0; j != 9; j++) {
            if (j != 6 && j != 7) {
              continue;
            }
            float similar =
                FastTargetLock(temp, armors[i].GetArmorRotRect(), j);
            if (similar > similarity) {
              similarity = similar;
              id_temp = j;
            }
          }

          std::cout << similarity << "\n";
          if (similarity > 0.85) {
            results[i] = id_temp;

            std::cout << "对应的新增加id:" << results[i] << "\n";
          }
        }
      }
    }
  }
  gettimeofday(&timek, NULL);
  long int time6 = timek.tv_usec;
  TDT_INFO("追踪：%d微秒", time6 - time5);
  Logic_fix(results, confs);

  for (uint64_t i = 0; i < armors.size(); i++) {
    float dis = sqrt(pow(armors[i].GetRect().y - src.size().height / 2, 2) +
                     pow(armors[i].GetRect().x - src.size().width / 2, 2));

    switch (results[i]) {
      case 7:
        armors[i].SetArmorType(tdttoolkit::RobotType::SENTRY);
        armors[i].Setconf(confs[i]);
        break;
      case 8:
        armors[i].SetArmorType(tdttoolkit::RobotType::BASE);
        armors[i].Setconf(confs[i]);
        break;
      default:
        armors[i].SetArmorType(tdttoolkit::RobotType(results[i]));
        armors[i].Setconf(confs[i]);
    }
  }
  //        for (int i=0;i!=9;i++){
  //            if(!this->traget_image[i].empty()){
  //                //cv::imshow(std::to_string(i),traget_image[i]);
  //            }else{
  //                std::cout<<"孔！！！！\n";
  //            }
  //
  //        }
  // 去除重复
  EraseDuplicate(armors);
//        std::cout<<"\n去除重复："<<time4-time3<<"微秒 \n";
#ifdef LIGHT_BAR_MATCH_DEBUG
  std::cout << "数字识别结果:";
  for (auto &temp : results) {
    std::cout << temp << ",";
  }
  std::cout << "\n";
#endif
  for (auto it = armors.begin(); it != armors.end();)  // 删除识别为0的装甲板
  {
    if (it->GetArmorType() == tdttoolkit::RobotType::TYPEUNKNOW) {
      armors.erase(it);
      time++;
      continue;
    }
    times++;
    it++;
  }
}

void ArmorDetector::FindLightBar(const cv::Mat &src,
                                 std::vector<LightBarInfo> &output_light_bars) {
  double t1 = cv::getTickCount();
  ////////////////////////////// 图像灯条查找预处理
  /////////////////////////////////
  cv::Mat img_gray;
  tdttoolkit::Lightbar_threshold(src, this->enemy_color_, img_gray,
                                 this->baitian_);
  cv::threshold(img_gray, img_gray, this->threshold_, 255,
                cv::THRESH_BINARY);  // 将灰度图二值化 寻找灯条解
  std::vector<std::vector<cv::Point2i>> contours;  // 轮廓容器

  // contours
  // 是一个vector嵌套vector,如果有机会可以用一些回归函数进行预测，从而减少下方的逻辑删除

  findContours(img_gray, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE,
               this->armor_detect_roi_.tl());  // 寻找轮廓
  for (int i = 0; i < contours.size(); i++) {
    tdttoolkit::CustomRect rot_rect(contours[i]);
    if (ifdebug) {
      std::cout << "所保存点的数量" << std::endl;
      std::cout << contours[i].size() << std::endl;
    }
    static cv::Point2i div_point(10, 15);
    static cv::Rect div_rect = cv::Rect();
    div_rect = cv::Rect(rot_rect.GetCenter() - div_point,
                        rot_rect.GetCenter() + div_point);
    // 这是什么逻辑？
    RectSafety(div_rect);

    if (tdtconfig::VIDEODEBUG) {
      // tdttoolkit::Debug::AddText("检测出的轮廓", std::to_string(i),
      // contours[i][0],cv::Scalar(255, 125, 0), 1, 2);
    }
    if (rot_rect.Vertical() < rot_rect.Cross() * 1.3) {
      if (ifdebug) {
        std::cout << "当前轮廓id:" << i << std::endl;

        std::cout << "灯条宽高比例错误" << std::endl;
        std::cout << "最小高度差:" << rot_rect.Vertical()
                  << "最小宽度差:" << rot_rect.Cross();
      }
      continue;
    }

    if (rot_rect.GetSize().area() < 20 || rot_rect.GetSize().area() > 51000) {
      // 面积太小或者太大, 扔掉
      if (ifdebug) {
        std::cout << "灯条面积面积错误" << std::endl;
      }
      continue;
    }
    if (rot_rect.GetAngle() > 155 || rot_rect.GetAngle() < 25) {
      if (ifdebug) {
        std::cout << "灯条角度错误" << std::endl;
      }
      continue;
    }
    if (rot_rect.GetWidth() > rot_rect.GetHeight() * 13) {
      if (ifdebug) {
        std::cout << "灯条另一种宽度比例错误" << std::endl;
        std::cout << rot_rect.GetWidth() << std::endl;
        std::cout << rot_rect.GetHeight() << std::endl;
      }
      continue;
    }
    cv::Mat temp = tdttoolkit::ROI(src, div_rect);
    if (temp.empty()) continue;
    cv::Scalar area_sum = sum(temp);
    if (area_sum[2 - enemy_color_] > area_sum[enemy_color_]) {
      if (ifdebug) {
        std::cout << "灯条颜色错误" << std::endl;
      }
      continue;
    }
#ifdef VIDEO_DEBUG
    // tdttoolkit::Debug::AddText("检测出的轮廓", std::to_string(i),
    // contours[i][0],
                       cv::Scalar(0, 125, 255), 1, 2);
#endif
                       LightBarInfo light_bar(rot_rect);
                       output_light_bars.push_back(light_bar);
  }
  sort(output_light_bars.begin(), output_light_bars.end(),
       [](const LightBarInfo &a, const LightBarInfo &b) -> bool {
         return a.GetCenter().x < b.GetCenter().x;
       });
  if (tdtconfig::VIDEODEBUG) {
    for (int i = 0; i < output_light_bars.size(); i++) {
      // tdttoolkit::Debug::AddCustomRect("寻找到的灯条",
      // output_light_bars[i],cv::Scalar(255, 192, 203), 2);
    }
  }
}

void ArmorDetector::FindLightBar_track(
    const cv::Mat &src, std::vector<LightBarInfo> &output_light_bars,
    const cv::Point2i &point) {
  ////////////////////////////// 图像灯条查找预处理
  /////////////////////////////////
  static cv::Mat img_gray;
  cv::extractChannel(src, img_gray, this->enemy_color_);
  cv::threshold(img_gray, img_gray, this->threshold_, 255,
                cv::THRESH_BINARY);  // 将灰度图二值化 寻找灯条解
  cv::dilate(img_gray, img_gray, this->element_);
  std::vector<std::vector<cv::Point2i>> contours;  // 轮廓容器
  findContours(img_gray, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE,
               point);  // 寻找轮廓
  ////////////////////////////// 根据轮廓查找灯条
  /////////////////////////////////

  for (int i = 0; i < contours.size(); i++) {
    tdttoolkit::CustomRect rot_rect(contours[i]);
    static cv::Point2i div_point(10, 10);
    static cv::Rect div_rect = cv::Rect();
    div_rect = cv::Rect(rot_rect.GetCenter() - div_point,
                        rot_rect.GetCenter() + div_point);
    RectSafety(div_rect);
    if (rot_rect.Vertical() < rot_rect.Cross() * 1.6) {
      //                std::cout << i << "灯条检测-Warning:
      //                竖向与横向比例不对:" <<
      //                rot_rect.Vertical() /
      //                rot_rect.Cross() << std::endl;
      continue;
    }
    if (rot_rect.GetSize().area() < 20 ||
        rot_rect.GetSize().area() > 19000) {  // 面积太小或者太大, 扔掉
      //                std::cout << i << "灯条检测-Warning:
      //                面积太小或者太大:" <<
      //                rot_rect.GetSize().area() <<
      //                std::endl;
      continue;
    }
    if (rot_rect.GetAngle() > 125 || rot_rect.GetAngle() < 55) {
      //                std::cout << i << "灯条检测-Warning:
      //                角度有问题:"
      //                << rot_rect.GetAngle() << std::endl;
      continue;
    }
    if (rot_rect.GetWidth() * 0.4 < rot_rect.GetHeight() ||
        rot_rect.GetWidth() > rot_rect.GetHeight() * 20) {
      //                std::cout << i << "灯条检测-Warning:
      //                长短比例有问题:" <<
      //                rot_rect.GetWidth() << "-" <<
      //                rot_rect.GetHeight() << std::endl;
      continue;
    }
    /*cv::Scalar area_sum = sum(src(div_rect));
    if (area_sum[2 - enemy_color_] > area_sum[enemy_color_])
    { std::cout << i << "灯条检测-Warning: 队友" <<
    std::endl; continue;
    }*/
    //            std::cout << i << "灯条检测-Information:
    //            正常" << std::endl;
    LightBarInfo light_bar(rot_rect);
    output_light_bars.push_back(light_bar);
  }
  sort(output_light_bars.begin(), output_light_bars.end(),
       [](const LightBarInfo &a, const LightBarInfo &b) -> bool {
         return a.GetCenter().x < b.GetCenter().x;
       });
}

void ArmorDetector::SingleLightBarDetect(
    const cv::Mat &src, std::vector<LightBarInfo> &light_bars,
    std::vector<ArmorDetectInfo> &output_armors) {
  std::vector<std::vector<ArmorDetectInfo>> armors_detected_vector(
      light_bars.size());
  std::vector<ArmorDetectInfo> tmp_output_armors;

  ////-------单灯条封装Armor函数-----------
  for (uint64_t i = 0; i < light_bars.size(); ++i) {
    //            std::cout<<light_bars[i].GetWidth()<<"\n";
    if (light_bars[i].GetFindStatus() == 2 || light_bars[i].GetArea() < 50 ||
        light_bars[i].GetFindStatus() == -1 ||
        light_bars[i].GetFindStatus() == 1 || light_bars[i].GetWidth() < 15) {
      // 左边或者右边已经查过, 跳过
      if (sdebug) {
        std::cout << "左边或者右边已经查过，或者面积太小或者"
                     "宽度太小"
                  << std::endl;
        std::cout << light_bars[i].GetArea() << std::endl;
        std::cout << light_bars[i].GetWidth() << std::endl;
      }
      continue;
    }
    LightBarInfo &light_bar = light_bars[i];
    std::vector<ArmorDetectInfo> &armors_detected = armors_detected_vector[i];
    for (int lr = -1; lr <= 1; lr += 2) {  // 搜索左侧右侧, 左-1, 右1
      if (lr == light_bar.GetFindStatus()) continue;
      // 旋转矩形中心点
      cv::Point2i rcentor =
          cv::Point2i(light_bar.GetCenter().x + 0.7f * light_bar.GetHeight() +
                          1.2f * light_bar.GetWidth() -
                          fabs(light_bar.GetWidth() *
                               cos(light_bar.GetAngle() / 180.f * M_PI)),

                      light_bar.GetCenter().y +
                          (light_bar.GetHeight() + light_bar.GetWidth()) *
                              cos(light_bar.GetAngle() / 180.f * M_PI));

      cv::Size2i rsize = cv::Size2i(
          2.4f * light_bar.GetWidth() -
              light_bar.GetWidth() *
                  fabs(cos(light_bar.GetAngle() / 180.f * M_PI)) *
                  (1 + 0.00001f * light_bar.GetRotRect().size.area()),

          1.3f * light_bar.GetWidth());

      if (last_robot_type_ == RobotType::BASE ||
          last_robot_type_ == RobotType::SENTRY ||
          last_robot_type_ == RobotType::HERO) {
        rcentor = cv::Point2i(
            light_bar.GetCenter().x +
                (0.7f * light_bar.GetHeight() + 1.f * light_bar.GetWidth()) *
                    1.7 -
                fabs(light_bar.GetWidth() *
                     cos(light_bar.GetAngle() / 180.f * M_PI)),
            light_bar.GetCenter().y +
                (light_bar.GetHeight() + light_bar.GetWidth()) *
                    cos(light_bar.GetAngle() / 180.f * M_PI));
        rsize.height *= 1.5f;
      }

      if (lr == -1) {
        rcentor = 2 * light_bar.GetCenter() - rcentor;
      }

      float angle = light_bar.GetAngle();
      if (angle < 90) {
        angle *= -1;
      }
      tdttoolkit::CustomRect T = tdttoolkit::CustomRect(rcentor, rsize, angle);
      cv::Rect2i search_rect =
          tdttoolkit::CustomRect(rcentor, rsize, light_bar.GetAngle())
              .GetRect();

      if (tdtconfig::VIDEODEBUG) {
        // tdttoolkit::Debug::AddCustomRect("单灯条检测潜在装甲板区域",
        // T,cv::Scalar(255, 255, 255), 2);
      }

      if (!RectSafety(search_rect)) {
        // 在画面外,跳过
        continue;
      }
      cv::Mat image_numb = src(search_rect);  // 更新last_image
      cv::Mat image_without_color;
      cv::extractChannel(image_numb, image_without_color, 2 - enemy_color_);
      int threshold =
          RegionOustThreshold(image_without_color, image_without_color, lr);
      if ((sum(image_without_color)[0] * 7 / 255) >
          (image_without_color.size().area() * 7)) {
        continue;
      }
      //                erode(image_without_color,
      //                image_without_color,
      //                cv::getStructuringElement(cv::MORPH_RECT,
      //                cv::Size(3, 3)), cv::Point(-1, -1),
      //                1); dilate(image_without_color,
      //                image_without_color,
      //                cv::getStructuringElement(cv::MORPH_RECT,
      //                cv::Size(3, 3)), cv::Point(-1, -1),
      //                2);
      std::vector<std::vector<cv::Point>> contours;
      findContours(image_without_color, contours, cv::RETR_EXTERNAL,
                   cv::CHAIN_APPROX_SIMPLE,
                   search_rect.tl());  // 查找轮廓
      if (contours.size() > 25 || contours.empty()) {
        continue;
      }

      std::vector<tdttoolkit::CustomRect> Numb_rects;
      for (std::vector<cv::Point> const &contour : contours) {
        if (contour.size() < 1) {  // 包含的点太少, 跳过
          if (sdebug) {
            std::cout << contour.size() << std::endl;
            std::cout << "单灯条点数量错误" << std::endl;
          }
          continue;
        }
        float angle = light_bar.GetAngle();
        tdttoolkit::CustomRect Numb_rect =
            tdttoolkit::CustomRect::minCustomRect(contour, angle);
        cv::Rect b_rect = Numb_rect.GetRect();

        // OUTPUT_SINGLE_LIGHT_BAR_DETECT_INFORMATION
        // VIDEO_DEBUG
        if (tdtconfig::VIDEODEBUG) {
          // tdttoolkit::Debug::AddText("单灯条检测潜在装甲板区域",std::to_string(contour.size()),Numb_rect.GetCenter(),cv::Scalar(255,
          // 255, 0), 1, 1);
        }
        if (b_rect.width > b_rect.height * 1.3) {  // 长宽不合要求, 跳过
          if (sdebug) {
            std::cout << "单灯条长宽比错误" << std::endl;
          }
          continue;
        }

        if (Numb_rect.GetSize().area() < 0.05 * search_rect.size().area() ||
            Numb_rect.GetSize().area() < 200) {  // 面积太小, 跳过
          if (sdebug) {
            std::cout << Numb_rect.GetSize().area() << std::endl;
            std::cout << "单灯条面积错误" << std::endl;
          }
          continue;
        }

        if (Numb_rect.GetSize().area() < light_bar.GetSize().area() * 1.5) {
          if (sdebug) {
            std::cout << "单灯条区域灯条比错误" << std::endl;
          }
          continue;
        }

        if (Numb_rect.GetSize().width < light_bar.GetWidth()) {
          if (sdebug) {
            std::cout << "单灯区域灯条宽比错误" << std::endl;
          }
          continue;
        }

        if (Numb_rect.GetSize().area() >
            search_rect.size().area()) {  // 面积太大
          if (sdebug) {
            std::cout << "单灯面积过大错误" << std::endl;
          }
          Numb_rect.SetSize(cv::Size(search_rect.height, search_rect.width));
        }

        float a, b, c;
        cv::Point2f light_bar_center;
        if (lr == -1) {
          light_bar_center = (light_bar.GetTl() + light_bar.GetBl());
          a = pow(light_bar_center.x - Numb_rect.GetCenter().x, 2) +
              pow(light_bar_center.y - Numb_rect.GetCenter().y, 2);
          b = pow(light_bar_center.x - light_bar.GetTl().x, 2) +
              pow(light_bar_center.y - light_bar.GetTl().y, 2);
          c = pow(Numb_rect.GetCenter().x - light_bar.GetTl().x, 2) +
              pow(Numb_rect.GetCenter().y - light_bar.GetTl().y, 2);
        } else {
          light_bar_center = (light_bar.GetTr() + light_bar.GetTl());
          a = pow(light_bar_center.x - Numb_rect.GetCenter().x, 2) +
              pow(light_bar_center.y - Numb_rect.GetCenter().y, 2);
          b = pow(light_bar_center.x - light_bar.GetTr().x, 2) +
              pow(light_bar_center.y - light_bar.GetTr().y, 2);
          c = pow(Numb_rect.GetCenter().x - light_bar.GetTr().x, 2) +
              pow(Numb_rect.GetCenter().y - light_bar.GetTr().y, 2);
        }
        double angle1 = ((a + b - c) / (2 * sqrt(a) * sqrt(b))) / M_PI * 180;
        if (angle1 < 45 || angle1 > 65) {
          continue;
        }
        Numb_rects.emplace_back(Numb_rect);
        //                    if (tdtconfig::VIDEODEBUG) {
        //                        //tdttoolkit::Debug::AddRotatedRect("单灯条检测潜在装甲板区域",
        //                        Numb_rect.GetRotRect(),
        //                        cv::Scalar(0, 0, 255), 2);
        //                    }
      }
      if (Numb_rects.empty()) {
        continue;
      }

      sort(Numb_rects.begin(), Numb_rects.end(),
           [](const tdttoolkit::CustomRect &a,
              const tdttoolkit::CustomRect &b) -> bool {
             return a.GetSize().area() > b.GetSize().area();
           });  // 以旋转矩形面积从大到小排序
      // 选择靠近灯条位置的轮廓
      if (Numb_rects.size() > 1) {
        Numb_rects[0] = (Numb_rects[0].GetCenter().x <
                         Numb_rects[1].GetCenter().x) == bool(lr == 1)
                            ? Numb_rects[0]
                            : Numb_rects[1];
      }
      if (tdtconfig::VIDEODEBUG) {
        // tdttoolkit::Debug::AddRotatedRect("数字区域范围",Numb_rects[0].GetRotRect(),cv::Scalar(255,
        // 192, 203), 2);
      }
      ArmorDetectInfo tmp_armor =
          ArmorDetectInfo(light_bar, Numb_rects[0], lr, threshold);
      CalaArmorRotRect(tmp_armor);

      //				light_bar.SetFindStatus(light_bar.GetFindStatus()
      //== -lr ? 2 : lr);//TODO

      auto func = [this, &tmp_output_armors](
                      const ArmorDetectInfo &input_armor) -> bool {
        for (auto &armor : tmp_output_armors) {
          if (IsArmorNearby(armor, input_armor)) {
            return true;
          }
        }
        return false;
      };

      if (func(tmp_armor)) {
        continue;
      }

      tmp_output_armors.push_back(tmp_armor);
      armors_detected.emplace_back(tmp_armor);
    }
  };
  for (uint64_t i = 0; i < light_bars.size(); ++i) {
    // tdttoolkit::Debug::AddText("单灯条检测潜在装甲板区域",
    // std::to_string(i),light_bars[i].GetRect().tl() +
    // cv::Point(0, 100),cv::Scalar(255, 200, 0), 1, 2);
    if (!armors_detected_vector[i].empty()) {
      output_armors.insert(output_armors.end(),
                           armors_detected_vector[i].begin(),
                           armors_detected_vector[i].end());
    }
    std::vector<ArmorDetectInfo>().swap(armors_detected_vector[i]);
  }
  std::vector<std::vector<ArmorDetectInfo>>().swap(armors_detected_vector);
}

void ArmorDetector::SingleLightBarDetect_tough(
    const cv::Mat &src, std::vector<LightBarInfo> &light_bars,
    std::vector<ArmorDetectInfo> &output_armors) {
  std::vector<std::vector<ArmorDetectInfo>> armors_detected_vector(
      light_bars.size());
  std::vector<ArmorDetectInfo> tmp_output_armors;

  ////-------单灯条封装Armor函数-----------
  for (uint64_t i = 0; i < light_bars.size(); ++i) {
    //            std::cout<<light_bars[i].GetWidth()<<"\n";
    if (light_bars[i].GetFindStatus() == 2 || light_bars[i].GetArea() < 300 ||
        light_bars[i].GetFindStatus() == -1 ||
        light_bars[i].GetFindStatus() == 1 ||
        light_bars[i].GetWidth() < 60) {  // 左边或者右边已经查过, 跳过
      continue;
    }
    LightBarInfo &light_bar = light_bars[i];
    std::vector<ArmorDetectInfo> &armors_detected = armors_detected_vector[i];
    for (int lr = -1; lr <= 1; lr += 2) {  // 搜索左侧右侧, 左-1, 右1
      if (lr == light_bar.GetFindStatus()) continue;
      cv::Point2i rcentor =
          cv::Point2i(light_bar.GetCenter().x + 0.7f * light_bar.GetHeight() +
                          1.2f * light_bar.GetWidth() -
                          fabs(light_bar.GetWidth() *
                               cos(light_bar.GetAngle() / 180.f * M_PI)),
                      light_bar.GetCenter().y +
                          (light_bar.GetHeight() + light_bar.GetWidth()) *
                              cos(light_bar.GetAngle() / 180.f * M_PI));
      cv::Size2i rsize = cv::Size2i(
          2.4f * light_bar.GetWidth() -
              light_bar.GetWidth() *
                  fabs(cos(light_bar.GetAngle() / 180.f * M_PI)) *
                  (1 + 0.00001f * light_bar.GetRotRect().size.area()),
          1.3f * light_bar.GetWidth());
      if (last_robot_type_ == RobotType::BASE ||
          last_robot_type_ == RobotType::SENTRY ||
          last_robot_type_ == RobotType::HERO) {
        rcentor = cv::Point2i(
            light_bar.GetCenter().x +
                (0.7f * light_bar.GetHeight() + 1.f * light_bar.GetWidth()) *
                    1.7 -
                fabs(light_bar.GetWidth() *
                     cos(light_bar.GetAngle() / 180.f * M_PI)),
            light_bar.GetCenter().y +
                (light_bar.GetHeight() + light_bar.GetWidth()) *
                    cos(light_bar.GetAngle() / 180.f * M_PI));
        rsize.height *= 2.0f;
      }
      if (lr == -1) {
        rcentor = 2 * light_bar.GetCenter() - rcentor;
      }
      float angle = light_bar.GetAngle();
      if (angle < 90) {
        angle *= -1;
      }
      tdttoolkit::CustomRect T = tdttoolkit::CustomRect(rcentor, rsize, angle);
      cv::Rect2i search_rect =
          tdttoolkit::CustomRect(rcentor, rsize, light_bar.GetAngle())
              .GetRect();
      // tdttoolkit::Debug::AddCustomRect("单灯条检测潜在装甲板区域",
      // T, cv::Scalar(255, 192, 203), 2);
      if (!RectSafety(search_rect)) {  // 在画面外,跳过
        continue;
      }
      cv::Mat image_numb = src(search_rect);  // 更新last_image
      cv::Mat image_without_color;
      cv::extractChannel(image_numb, image_without_color, 2 - enemy_color_);
      int threshold =
          RegionOustThreshold(image_without_color, image_without_color, lr);
      if ((sum(image_without_color)[0] * 7 / 255) >
          (image_without_color.size().area() * 7)) {
        continue;
      }
      //                erode(image_without_color,
      //                image_without_color,
      //                cv::getStructuringElement(cv::MORPH_RECT,
      //                cv::Size(3, 3)), cv::Point(-1, -1),
      //                1); dilate(image_without_color,
      //                image_without_color,
      //                cv::getStructuringElement(cv::MORPH_RECT,
      //                cv::Size(3, 3)), cv::Point(-1, -1),
      //                2);
      std::vector<std::vector<cv::Point>> contours;
      findContours(image_without_color, contours, cv::RETR_EXTERNAL,
                   cv::CHAIN_APPROX_SIMPLE,
                   search_rect.tl());  // 查找轮廓
      if (contours.size() > 25 || contours.empty()) {
        continue;
      }

      std::vector<tdttoolkit::CustomRect> Numb_rects;
      for (std::vector<cv::Point> const &contour : contours) {
        if (contour.size() < 18) {  // 包含的点太少, 跳过
          continue;
        }
        float angle = light_bar.GetAngle();
        tdttoolkit::CustomRect Numb_rect =
            tdttoolkit::CustomRect::minCustomRect(contour, angle);
        cv::Rect b_rect = Numb_rect.GetRect();
        if (tdtconfig::VIDEODEBUG) {
#ifdef OUTPUT_SINGLE_LIGHT_BAR_DETECT_INFORMATION
          // tdttoolkit::Debug::AddText("单灯条检测潜在装甲板区域",
                                   std::to_string(k), Numb_rect.GetCenter(),
                                   cv::Scalar(125, 255, 0), 1, 1);
#endif
        }  // OUTPUT_SINGLE_LIGHT_BAR_DETECT_INFORMATION
        // VIDEO_DEBUG
        if (b_rect.width > b_rect.height) {  // 长宽不合要求, 跳过
          continue;
        }

        if (Numb_rect.GetSize().area() < 0.05 * search_rect.size().area() ||
            Numb_rect.GetSize().area() < 350) {  // 面积太小, 跳过
          continue;
        }

        if (Numb_rect.GetSize().area() < light_bar.GetSize().area() * 1.5) {
          continue;
        }

        if (Numb_rect.GetSize().width < light_bar.GetWidth()) {
          continue;
        }

        if (Numb_rect.GetSize().area() >
            search_rect.size().area()) {  // 面积太大
          Numb_rect.SetSize(cv::Size(search_rect.height, search_rect.width));
        }

        float a, b, c;
        cv::Point2f light_bar_center;
        if (lr == -1) {
          light_bar_center = (light_bar.GetTl() + light_bar.GetBl());
          a = pow(light_bar_center.x - Numb_rect.GetCenter().x, 2) +
              pow(light_bar_center.y - Numb_rect.GetCenter().y, 2);
          b = pow(light_bar_center.x - light_bar.GetTl().x, 2) +
              pow(light_bar_center.y - light_bar.GetTl().y, 2);
          c = pow(Numb_rect.GetCenter().x - light_bar.GetTl().x, 2) +
              pow(Numb_rect.GetCenter().y - light_bar.GetTl().y, 2);
        } else {
          light_bar_center = (light_bar.GetTr() + light_bar.GetTl());
          a = pow(light_bar_center.x - Numb_rect.GetCenter().x, 2) +
              pow(light_bar_center.y - Numb_rect.GetCenter().y, 2);
          b = pow(light_bar_center.x - light_bar.GetTr().x, 2) +
              pow(light_bar_center.y - light_bar.GetTr().y, 2);
          c = pow(Numb_rect.GetCenter().x - light_bar.GetTr().x, 2) +
              pow(Numb_rect.GetCenter().y - light_bar.GetTr().y, 2);
        }
        double angle1 = ((a + b - c) / (2 * sqrt(a) * sqrt(b))) / M_PI * 180;
        if (angle1 < 45 || angle1 > 65) {
          continue;
        }
        Numb_rects.emplace_back(Numb_rect);

        // tdttoolkit::Debug::AddRotatedRect("单灯条检测潜在装甲板区域",Numb_rect.GetRotRect(),cv::Scalar(0,
        // 0, 255), 2);
      }
      if (Numb_rects.empty()) {
        continue;
      }

      sort(Numb_rects.begin(), Numb_rects.end(),
           [](const tdttoolkit::CustomRect &a,
              const tdttoolkit::CustomRect &b) -> bool {
             return a.GetSize().area() > b.GetSize().area();
           });  // 以旋转矩形面积从大到小排序
      // 选择靠近灯条位置的轮廓
      if (Numb_rects.size() > 1) {
        Numb_rects[0] = (Numb_rects[0].GetCenter().x <
                         Numb_rects[1].GetCenter().x) == bool(lr == 1)
                            ? Numb_rects[0]
                            : Numb_rects[1];
      }

      // tdttoolkit::Debug::AddRotatedRect("数字区域范围",
      // Numb_rects[0].GetRotRect(),cv::Scalar(255, 192,
      // 203), 2);

      ArmorDetectInfo tmp_armor =
          ArmorDetectInfo(light_bar, Numb_rects[0], lr, threshold);
      CalaArmorRotRect(tmp_armor);

      //				light_bar.SetFindStatus(light_bar.GetFindStatus()
      //== -lr ? 2 : lr);//TODO

      auto func = [this, &tmp_output_armors](
                      const ArmorDetectInfo &input_armor) -> bool {
        for (auto &armor : tmp_output_armors) {
          if (IsArmorNearby(armor, input_armor)) {
            return true;
          }
        }
        return false;
      };

      if (func(tmp_armor)) {
        continue;
      }

      tmp_output_armors.push_back(tmp_armor);
      armors_detected.emplace_back(tmp_armor);
    }
  };
  for (uint64_t i = 0; i < light_bars.size(); ++i) {
    // tdttoolkit::Debug::AddText("单灯条检测潜在装甲板区域",
    // std::to_string(i),light_bars[i].GetRect().tl() +
    // cv::Point(0, 100),cv::Scalar(255, 200, 0), 1, 2);
    if (!armors_detected_vector[i].empty()) {
      output_armors.insert(output_armors.end(),
                           armors_detected_vector[i].begin(),
                           armors_detected_vector[i].end());
    }
    std::vector<ArmorDetectInfo>().swap(armors_detected_vector[i]);
  }
  std::vector<std::vector<ArmorDetectInfo>>().swap(armors_detected_vector);
}

// 循环嵌套双灯条匹配
void ArmorDetector::DoubleLightBarDetect(
    const cv::Mat &src, std::vector<LightBarInfo> &light_bars,
    std::vector<ArmorDetectInfo> &output_armors) {
  std::vector<std::vector<ArmorDetectInfo>> armors_detected_vector(
      light_bars.size() - 1);
  for (uint64_t i = 0; i < (light_bars.size() - 1); ++i) {
    for (uint64_t j = i + 1; j < light_bars.size(); j++) {
      if (!GetEligibility(light_bars[i], light_bars[j]))
        continue;  // 灯条匹配判断
      std::cout << "test"
                << "\n";
      ////----------算出查找数字贴纸的区域---------------
      /*
       * 我需要整理出框选的逻辑
       * 1，找到左右匹配的两个灯条
       * 2，找长度最长的一个
       * 3，框选其中灯条中心长度*0.7 加灯条长度*1.2
       */

      LightBarInfo light_bar =
          light_bars[i].GetWidth() > light_bars[j].GetWidth() ? light_bars[i]
                                                              : light_bars[j];
      float width = 0.7 * tdttoolkit::CalcDistance(light_bars[j].GetCenter(),
                                                   light_bars[i].GetCenter());
      float height =
          1.2 * (light_bars[i].GetWidth() + light_bars[j].GetWidth());
      float angle = (light_bars[i].GetAngle() + light_bars[j].GetAngle()) / 2;
      cv::Point center =
          (light_bars[i].GetCenter() + light_bars[j].GetCenter()) / 2;

      CustomRect search_number_rect;
      // 这个地方就是为了确定angle，为什么我需要这个呢？
      if (height > width) {
        search_number_rect = CustomRect(center, cv::Size(height, width), angle);
      } else if (angle > 90 && width > height) {
        search_number_rect =
            CustomRect(center, cv::Size(width, height), angle - 90);
      } else if (angle < 90 && width > height) {
        search_number_rect =
            CustomRect(center, cv::Size(width, height), angle + 90);
      } else if (angle == 90 && width > height) {
        search_number_rect = CustomRect(center, cv::Size(width, height), 0);
      } else {
        height += 0.001;
        search_number_rect = CustomRect(center, cv::Size(height, width), angle);
      }
      cv::Rect search_rect = search_number_rect.GetRect();
      //                cv::Rect search_rect
      //                =get_search_rect(light_bars[i],
      //                light_bars[j]);

      if (!RectSafety(search_rect))  // 若矩形在图像外, 跳过, 否则取
      {
        continue;  // 安全性
      }
// 灯条匹配的结果
#ifdef VIDEO_DEBUG
      // tdttoolkit::Debug::AddCustomRect("双灯条检测潜在装甲板区域",
      // search_number_rect,
                                 cv::Scalar(255, 192, 203), 2);
                                 // tdttoolkit::Debug::AddText("双灯条检测潜在装甲板区域",
                                 // std::to_string(j),
                           search_rect.tl(), cv::Scalar(255, 192, 203), 2);
#endif
                           cv::Mat image_numb =
                               src(search_rect);  // 取感兴趣区域
                           cv::Mat image_without_color;
                           cv::extractChannel(image_numb, image_without_color,
                                              2 - enemy_color_);
                           int thre = RegionOustThreshold(
                               image_without_color, image_without_color, 0);
                           erode(image_without_color, image_without_color,
                                 this->element2_, cv::Point(-1, -1), 1);
                           // cv::imshow(std::to_string(i).append(std::to_string(j)),image_without_color);
                           dilate(image_without_color, image_without_color,
                                  this->element2_, cv::Point(-1, -1), 2);

                           std::vector<std::vector<cv::Point>> contours;
                           findContours(image_without_color, contours,
                                        cv::RETR_EXTERNAL,
                                        cv::CHAIN_APPROX_SIMPLE,
                                        search_rect.tl());  // 查找轮廓
                           tdttoolkit::CustomRect Numb_rect;
                           tdttoolkit::CustomRect temp_numb_rect;

                           light_bar.SetAngle(0.5 * (light_bars[i].GetAngle() +
                                                     light_bars[j].GetAngle()));
                           std::sort(contours.begin(), contours.end(),
                                     [](std::vector<cv::Point> &a,
                                        std::vector<cv::Point> &b) -> bool {
                                       return a.size() > b.size();
                                     });
                           uint8_t time = 0, last_contour = 0;
                           for (auto &contour :
                                contours)  // 遍历轮廓
                                           // 只遍历包含点最多的两个contour
                           {
                             if (time >= 2) {
                               break;
                             }
                             time++;
                             if (contour.size() < 10)  // 包含的点太少不要
                             {
                               continue;  // 点太少的不要
                             }
                             temp_numb_rect =
                                 tdttoolkit::CustomRect::minCustomRect(
                                     contour, light_bar.GetAngle());
                             if (temp_numb_rect.GetSize().area() <
                                 0.15 * search_number_rect.GetSize().area()) {
                               continue;
                             }
                             // 计算的两灯条中心距离数字贴纸中心的距离
                             float dist1 = tdttoolkit::CalcDistance(
                                 cv::Point(temp_numb_rect.GetCenter()),
                                 light_bars[i].GetCenter());
                             float dist2 = tdttoolkit::CalcDistance(
                                 cv::Point(temp_numb_rect.GetCenter()),
                                 light_bars[j].GetCenter());
                             float dist3 = tdttoolkit::CalcDistance(
                                 cv::Point(temp_numb_rect.GetCenter()),
                                 (light_bars[i].GetCenter() +
                                  light_bars[j].GetCenter()) /
                                     2);
                             // 如果数字贴纸离灯条确定的中心太远, 筛掉
                             if (light_bars[i].GetWidth() >
                                 light_bars[j].GetWidth()) {
                               if (dist1 > 1.7 * dist2 || dist2 > 1.8 * dist1) {
                                 continue;
                               }
                               if (dist3 / dist2 > 0.4) continue;
                             } else {
                               if (dist1 > 1.8 * dist2 || dist2 > 1.7 * dist1) {
                                 continue;
                               }
                               if (dist3 / dist1 > 0.4) continue;
                             }
                             if (temp_numb_rect.GetRect().width >
                                 2.5 * temp_numb_rect.GetRect().height) {
                               continue;
                             }
                             if (Numb_rect.GetSize().area() != 0 &&
                                 contour.size() > last_contour / 3) {
                               std::vector<cv::Point> temp =
                                   Numb_rect.GetVertices();
                               contour.insert(contour.end(), temp.begin(),
                                              temp.end());
                               Numb_rect = CustomRect::minCustomRect(
                                   contour, light_bar.GetAngle());
                               std::vector<cv::Point>().swap(temp);
                             } else if (Numb_rect.GetSize().area() <
                                        temp_numb_rect.GetSize().area()) {
                               Numb_rect = temp_numb_rect;
                               last_contour = contour.size();
                             }  // 保存可能存在的数字区域进列表
                           }
                           if (Numb_rect.GetSize().area() == 0) {
                             ArmorDetectInfo tmp_armor(
                                 light_bars[i], light_bars[j], Numb_rect, thre);
                             CalaArmorRotRect(tmp_armor, search_number_rect);
                             light_bars[i].SetFindStatus(
                                 light_bars[i].GetFindStatus() == -1
                                     ? 2
                                     : 1);  //-1表示此灯条寻找过左边
                             light_bars[j].SetFindStatus(
                                 light_bars[j].GetFindStatus() == 1
                                     ? 2
                                     : -1);  // 1表示寻找过右边
                             armors_detected_vector[i].push_back(tmp_armor);
                           } else {
                             ArmorDetectInfo tmp_armor(
                                 light_bars[i], light_bars[j], Numb_rect, thre);
                             CalaArmorRotRect(tmp_armor);
                             light_bars[i].SetFindStatus(
                                 light_bars[i].GetFindStatus() == -1
                                     ? 2
                                     : 1);  //-1表示此灯条寻找过左边
                             light_bars[j].SetFindStatus(
                                 light_bars[j].GetFindStatus() == 1
                                     ? 2
                                     : -1);  // 1表示寻找过右边
                             armors_detected_vector[i].push_back(tmp_armor);
                           }

                           // //tdttoolkit::Debug::AddRotatedRect("双灯条检测潜在装甲板区域",
                           // temp_numb_rect.GetRotRect(), cv::Scalar(0, 0,
                           // 255), 2);
#ifdef VIDEO_DEBUG
                           // tdttoolkit::Debug::AddCustomRect("数字区域范围",
                           // Numb_rect,
                                 cv::Scalar(155, 155, 255), 2);
#endif
    }
    if (!armors_detected_vector[i].empty()) {
      output_armors.insert(output_armors.end(),
                           armors_detected_vector[i].begin(),
                           armors_detected_vector[i].end());
    }
    std::vector<ArmorDetectInfo>().swap(armors_detected_vector[i]);
  }
  std::vector<std::vector<ArmorDetectInfo>>().swap(armors_detected_vector);
}

void ArmorDetector::CalaArmorRotRect(ArmorDetectInfo &armor,
                                     tdttoolkit::CustomRect lighrbarroi) {
  ////////////////////////////////////////////确定机器学习区域/////////////////////////////////////////////
  tdttoolkit::CustomRect roi;
  if (armor.HaveLeftBar() && armor.HaveRightBar()) {
    if (lighrbarroi.GetAngle() != 0) {
      roi = lighrbarroi;
    } else {
      float distance = tdttoolkit::CalcDistance(
          armor.GetLeftBarRect().GetCenter(),
          armor.GetRightBarRect().GetCenter());  // 两个矩形框的距离
      tdttoolkit::CustomRect better_light_bar(cv::Point(0, 0), cv::Size(0, 0),
                                              0);
      if (armor.GetLeftBarRect().GetAngle() != 90 &&
          armor.GetRightBarRect().GetAngle() != 90) {
        better_light_bar = 0.5 * (armor.GetLeftBarRect().GetAngle() +
                                  armor.GetRightBarRect().GetAngle()) >
                                   90
                               ? armor.GetLeftBarRect()
                               : armor.GetRightBarRect();
      } else {
        better_light_bar = armor.GetLeftBarRect().GetWidth() <
                                   armor.GetRightBarRect().GetWidth()
                               ? armor.GetLeftBarRect()
                               : armor.GetRightBarRect();
      }
      cv::Size size;
      size.width = fmax(better_light_bar.GetWidth() * 2.2,
                        armor.GetNumberRotRect().GetWidth() * 1.1);
      size.height =
          fmin(distance - 1.8 * better_light_bar.GetHeight(), size.width / 1.4);
      size.height =
          fmax(size.height, armor.GetNumberRotRect().GetHeight() * 1.05);
      float angle = better_light_bar.GetAngle();
      cv::Point center = (armor.GetLeftBarRect().GetCenter() +
                          armor.GetRightBarRect().GetCenter()) /
                         2;
      roi = tdttoolkit::CustomRect(center, size, angle);
    }
    /*
                roi = armor.GetNumberRotRect();
                roi.SetSize(cv::Size(roi.GetSize().width
       * 1.1, roi.GetSize().height * 1.1));*/

    // std::cout<<"左边灯条角度："<<armor.GetLeftBarRect().GetAngle()<<"
    // 右："<<armor.GetRightBarRect().GetAngle();

  } else if (armor.HaveLeftBar() || armor.HaveRightBar()) {
    tdttoolkit::CustomRect better_lightbar =
        armor.HaveRightBar() ? armor.GetRightBarRect() : armor.GetLeftBarRect();
    cv::Size size;
    cv::Point center = armor.GetNumberRotRect().GetCenter();
    cv::Point top_center =
        (better_lightbar.GetTl() + better_lightbar.GetTr()) / 2;

    //////////////////////////////////////////灯条间距////////////////////////////////////////////////
    int A = 0, B = 0, C = 0;
    A = top_center.y - better_lightbar.GetCenter().y;
    B = better_lightbar.GetCenter().x - top_center.x;
    C = top_center.x * better_lightbar.GetCenter().y -
        top_center.y * better_lightbar.GetCenter().x;
    // 代入点到直线距离公式
    double distance =
        2 * (fabs(A * center.x + B * center.y + C)) / (sqrt(A * A + B * B));
    ////////////////////////////////////////////////////////////////////////////////////////////////

    size.width = fmax(cvRound(better_lightbar.GetWidth() * 2.2),
                      cvRound(armor.GetNumberRotRect().GetHeight() * 1.1));
    size.height = fmin(cvRound(distance - 1.8 * better_lightbar.GetHeight()),
                       cvRound(size.width / 1.4));
    size.height =
        fmax(size.height, cvRound(armor.GetNumberRotRect().GetHeight() * 1.05));
    float angle = better_lightbar.GetAngle();
    roi = tdttoolkit::CustomRect(center, size, angle);
  } else {
    roi = armor.GetNumberRotRect();
    roi.SetSize(
        cv::Size(roi.GetSize().width * 1.1, roi.GetSize().height * 1.1));
  }
  ////////////////////////////////////////////////////////////////////////////////////////////////
  armor.SetArmorRotRect(roi);
}

void ArmorDetector::GetMlRoi(const ArmorDetectInfo &armor, const cv::Mat &src,
                             cv::Mat &ml_roi) {
  tdttoolkit::CustomRect roi = armor.GetArmorRotRect();

  cv::Rect roi_mat = roi.GetRect();

  cv::Rect roi_matcopy = cv::Rect(roi_mat);

  if (RectSafety(roi_matcopy)) {
    // 取安全的矩形
    cv::Mat roiimg;
    if (roi_matcopy != roi_mat) {  // 如果取安全矩形对矩形进行过裁剪
      roiimg = cv::Mat::zeros(roi_mat.size(),
                              CV_8UC3);  // 大小和原矩形一样
                                         // //NOLINT(hicpp-signed-bitwise)
      src(roi_matcopy)
          .copyTo(roiimg(
              cv::Rect(roi_matcopy.tl() - roi_mat.tl(), roi_matcopy.size())));
    } else {
      roiimg = src(roi_mat);
    }

    float tlx = static_cast<float>(roi_mat.tl().x);
    float tly = static_cast<float>(roi_mat.tl().y);
    cv::Point2f tl = cv::Point2f(tlx, tly);
    cv::Point2f src[] = {roi.GetTl() - tl, roi.GetTr() - tl, roi.GetBr() - tl};
    cv::Point2f dst[] = {cv::Point2f(0, 0), cv::Point2f(28, 0),
                         cv::Point2f(28, 28)};
    cv::Mat rotation_mat = cv::getAffineTransform(src, dst);
    warpAffine(roiimg, ml_roi, rotation_mat, cv::Size(28, 28));
    // 新增加颜色通道分离

    if (ml_roi.empty()) {
      ml_roi = cv::Mat::ones(28, 28,
                             CV_8UC3);  // NOLINT(hicpp-signed-bitwise)
    }
  } else {
    ml_roi = cv::Mat::ones(28, 28, CV_8UC3);  // NOLINT(hicpp-signed-bitwise)
  }

  // cv::extractChannel(ml_roi, ml_roi, 2-enemy_color_);
  cv::cvtColor(ml_roi, ml_roi, cv::COLOR_BGR2GRAY);
  // cv::imwrite(std::to_string(cv::getTickCount())+".png", ml_roi);
  //        //tdttoolkit::Debug::img=ml_roi.clone();
  //        //tdttoolkit::Debug::ShowMat();
}

bool ArmorDetector::IsArmorNearby(const ArmorDetectInfo &armor_a,
                                  const ArmorDetectInfo &armor_b) {
  if (fabs(armor_a.GetRect().x - armor_b.GetRect().x) * 3 <
      (armor_a.GetRect().width)) {
    if (fabs(armor_a.GetRect().y - armor_b.GetRect().y) * 3 <
        (armor_a.GetRect().height)) {
      return true;
    }
  }
  return false;
}

/*******************************************************************************************************************
 * 工具
 ******************************************************************************************************************/

bool ArmorDetector::RectSafety(cv::Rect2i &rect) {
  rect = rect & this->src_area_;
  return !rect.empty();
}

cv::Rect2i ArmorDetector::RectEnlarge(const cv::Rect2i &rect,
                                      const cv::Size2i &gain) {
  cv::Rect2i output_rect;
  cv::Size arg = cv::Size((gain.width - 1) * rect.size().width,
                          (gain.height - 1) * rect.size().height);
  output_rect = rect + arg;
  output_rect = output_rect - cv::Point2i(arg.width / 2, arg.height / 2);
  RectSafety(output_rect);
  return output_rect;
}
int ArmorDetector::BalanceSort(vector<Point2f> armors, RobotType type) {
  if (int(type) != tdttoolkit::INFANTRY3 && int(type) != INFANTRY4 &&
      int(type) != INFANTRY5) {
    return 0;
  }
  Point2f centerPoint[2];
  centerPoint[0] = (armors[0] + armors[1]) / 2;
  centerPoint[1] = (armors[8] + armors[9]) / 2;
  vector<Point2f> use_armor_type;
  if (armors[0].x != 0 && armors[1].x != 0 && armors[9].x != 0 &&
      armors[8].x != 0) {
    use_armor_type.push_back(armors[0]);
    use_armor_type.push_back(armors[1]);
    use_armor_type.push_back(centerPoint[0]);
    use_armor_type.push_back(armors[8]);
    use_armor_type.push_back(armors[9]);
    use_armor_type.push_back(centerPoint[1]);

    float length1 =
        sqrt((use_armor_type[0].x - use_armor_type[1].x) *
                 (use_armor_type[0].x - use_armor_type[1].x) +
             (use_armor_type[0].y - use_armor_type[1].y) *
                 (use_armor_type[0].y - use_armor_type[1].y));  // 左灯条长度
    float length2 =
        sqrt((use_armor_type[3].x - use_armor_type[4].x) *
                 (use_armor_type[3].x - use_armor_type[4].x) +
             (use_armor_type[3].y - use_armor_type[4].y) *
                 (use_armor_type[3].y - use_armor_type[4].y));  // 右灯条长度
    float length3 = sqrt(
        (use_armor_type[2].x - use_armor_type[5].x) *
            (use_armor_type[2].x - use_armor_type[5].x) +
        (use_armor_type[2].y - use_armor_type[5].y) *
            (use_armor_type[2].y - use_armor_type[5].y));  // 两灯条中点连线长度

    if (length1 != 0 && length2 != 0) {
      std::cout << "balanceAngle"
                << atan(max(fabs(length1 / length2), fabs(length2 / length1))) /
                       CV_PI * 180
                << std::endl;
      if (atan(max(fabs(length1 / length2), fabs(length2 / length1))) / CV_PI *
                  180 >
              45 &&
          // 根据需求选择判断条件：默认角判断
          atan(max(fabs(length1 / length2),
                   fabs(length2 / length1))) /
                  CV_PI * 180 <
              46) {  // 判断机器人正对条件：角判断
        //                if (fabs(length1 - length2)
        //                < 1) {
        //                //判断机器人正对条件：两灯条差值
        /** 下面的判断条件不建议修改 **/
        std::cout
            //  << "balanceJudge"
            << (length1 + length2) / 2 / length3 << std::endl;
        if ((length1 + length2) / 2 / length3 < 0.53 &&
            (length1 + length2) / 2 / length3 > 0.43) {  // 小装甲限制条件
          balance_counter[int(type) - 3][0] += 1;
        } else if ((length1 + length2) / 2 / length3 <
                   0.27) {  // 大装甲判断条件
          balance_counter[int(type) - 3][1] += 1;
        } else {
          return 0;
        }
        if (balance_counter[int(type) - 3][0] > 2 &&
            balance_counter[int(type) - 3][1] > 2) {
          if (balance_counter[int(type) - 3][0] >
              balance_counter[int(type) - 3][1]) {
            balance_counter[int(type) - 3][0] = 1;
            balance_counter[int(type) - 3][1] = 0;
          } else {
            balance_counter[int(type) - 3][0] = 0;
            balance_counter[int(type) - 3][1] = 1;
          }
        }

        /****调试用，为不影响帧率，比赛记得注释*****/
        //                    for(int i = 0; i< 3;
        //                    i++){
        //                        std::cout <<
        //                        "balance_counter";
        //                        for(int j=0; j<
        //                        2;j++)
        //                            std::cout <<
        //                            balance_counter[i][j];
        //                        std::cout <<
        //                        std::endl;
        //                    }
        /***************************************/

        if (balance_counter[int(type) - 3][0] >
            balance_counter[int(type) - 3][1]) {
          SetBalance(int(type), 1);
          return 1;
        } else {
          SetBalance(int(type), 2);
          return 2;
        }
      } else {
        return 0;
      }
    } else {
      return 0;
    }
  } else {
    return 0;
  }
}

void ArmorDetector::logicBalance(vector<Point2f> armors, RobotType type) {
  // std::cout << "BALANCEON " << balance_on << std::endl;
  if (balance_on == -1) {
    return;
  } else if (balance_on < 111 && balance_on != -1) {
    int BalanceType = BalanceSort(armors, type);
    //  std::cout << "BALANCEON " << BalanceType <<
    //  std::endl;
  } else if (balance_on >= 111 && balance_on <= 222) {
    SetBalance(3, balance_on / 100);
    SetBalance(4, balance_on % 100 / 10);
    SetBalance(5, balance_on % 10);
    int balanceSet = balance_[0] * 100 + balance_[1] * 10 + balance_[2] * 1;
  } else {
    return;
  }
}

void ArmorDetector::ArmorTransform(const ArmorDetectInfo &armor_info,
                                   tdttoolkit::RobotArmor &armor) {
  /////////////////////////////////计算图像点////////////////////////////////////
  // 从左到右依次是 左灯条{0上定点,1下顶点,2中心点}
  // 数字贴纸{3左中心点,4右中心点,5上中心点,6下中心点,7中心点}
  // 右灯条{8上定点,9下顶点,10中心点}
  std::vector<cv::Point2f> image_point_lists = std::vector<cv::Point2f>(6);

  // image_point_lists[3] = (armor_info.GetArmorRotRect().GetTl() +
  // armor_info.GetArmorRotRect().GetBl()) / 2; image_point_lists[4] =
  // (armor_info.GetArmorRotRect().GetTr() +
  // armor_info.GetArmorRotRect().GetBr()) / 2;
  //        image_point_lists[5] = (armor_info.GetArmorRotRect().GetTl() +
  //        armor_info.GetArmorRotRect().GetTr()) / 2; image_point_lists[6] =
  //        (armor_info.GetArmorRotRect().GetBl() +
  //        armor_info.GetArmorRotRect().GetBr()) / 2; image_point_lists[7] =
  //        armor_info.GetArmorRotRect().GetCenter();

  if (armor_info.HaveLeftBar() && armor_info.HaveRightBar()) {
    image_point_lists[0] = (armor_info.GetLeftBarRect().GetTl() +
                            armor_info.GetLeftBarRect().GetTr()) /
                           2;
    image_point_lists[1] = (armor_info.GetLeftBarRect().GetBl() +
                            armor_info.GetLeftBarRect().GetBr()) /
                           2;
    image_point_lists[2] = armor_info.GetLeftBarRect().GetCenter();

    image_point_lists[3] = (armor_info.GetRightBarRect().GetTl() +
                            armor_info.GetRightBarRect().GetTr()) /
                           2;
    image_point_lists[4] = (armor_info.GetRightBarRect().GetBl() +
                            armor_info.GetRightBarRect().GetBr()) /
                           2;
    image_point_lists[5] = armor_info.GetRightBarRect().GetCenter();
  } else if (armor_info.HaveLeftBar()) {
    image_point_lists[0] = (armor_info.GetLeftBarRect().GetTl() +
                            armor_info.GetLeftBarRect().GetTr()) /
                           2;
    image_point_lists[1] = (armor_info.GetLeftBarRect().GetBl() +
                            armor_info.GetLeftBarRect().GetBr()) /
                           2;
    image_point_lists[2] = armor_info.GetLeftBarRect().GetCenter();
  } else if (armor_info.HaveRightBar()) {
    image_point_lists[3] = (armor_info.GetRightBarRect().GetBl() +
                            armor_info.GetRightBarRect().GetBr()) /
                           2;
    image_point_lists[4] = (armor_info.GetRightBarRect().GetTl() +
                            armor_info.GetRightBarRect().GetTr()) /
                           2;
    image_point_lists[5] = armor_info.GetRightBarRect().GetCenter();
  }
  int believable;
  // TODO 等待增加无灯条结算：使用

  if (!armor_info.HaveLeftBar() && !armor_info.HaveRightBar())
    believable = 0;
  else if (armor_info.HaveLeftBar() && armor_info.HaveRightBar())
    believable = 1;
  else
    believable = armor_info.HaveLeftBar() ? 2 : 3;
  logicBalance(image_point_lists,
               tdttoolkit::RobotType(armor_info.GetArmorType()));
  armor = tdttoolkit::RobotArmor(armor_info.GetArmorRotRect(),
                                 armor_info.GetArmorType(), image_point_lists,
                                 believable, balance_);
  // cout << armor.GetWorldPointsList()[0] << endl;
}

void ArmorDetector::DPArmorTransform(const ArmorDetectInfo &armor_info,
                                     tdttoolkit::RobotArmor &armor,
                                     int enemy_color) {
  /////////////////////////////////计算图像点////////////////////////////////////
  // 从左到右依次是 左灯条{0上定点,1下顶点,2中心点}
  // 数字贴纸{3左中心点,4右中心点,5上中心点,6下中心点,7中心点}
  // 右灯条{8上定点,9下顶点,10中心点}
  // std::vector<cv::Point2f> image_point_lists = std::vector<cv::Point2f>(11);
  std::vector<cv::Point2f> image_point_lists = std::vector<cv::Point2f>(6);

  // image_point_lists[3] = (armor_info.GetArmorRotRect().GetTl() +
  // armor_info.GetArmorRotRect().GetBl()) / 2; image_point_lists[4] =
  // (armor_info.GetArmorRotRect().GetTr() +
  // armor_info.GetArmorRotRect().GetBr()) / 2;
  //        image_point_lists[5] = (armor_info.GetArmorRotRect().GetTl() +
  //        armor_info.GetArmorRotRect().GetTr()) / 2; image_point_lists[6] =
  //        (armor_info.GetArmorRotRect().GetBl() +
  //        armor_info.GetArmorRotRect().GetBr()) / 2; image_point_lists[7] =
  //        armor_info.GetArmorRotRect().GetCenter();

  // if (armor_info.HaveLeftBar() && armor_info.HaveRightBar()) {
  //   image_point_lists[0] = armor_info.GetTL();
  //   image_point_lists[1] = armor_info.GetBL();
  //   image_point_lists[2] = (armor_info.GetTL() + armor_info.GetBL()) / 2;

  //   image_point_lists[8] = armor_info.GetTR();
  //   image_point_lists[9] = armor_info.GetBR();
  //   image_point_lists[10] = (armor_info.GetTR() + armor_info.GetBR()) / 2;
  // } else if (armor_info.HaveLeftBar()) {
  //   image_point_lists[0] = armor_info.GetTL();
  //   image_point_lists[1] = armor_info.GetBL();
  //   image_point_lists[2] = (armor_info.GetTL() + armor_info.GetBL()) / 2;
  // } else if (armor_info.HaveRightBar()) {
  //   image_point_lists[8] = armor_info.GetTR();
  //   image_point_lists[9] = armor_info.GetBR();
  //   image_point_lists[10] = (armor_info.GetTR() + armor_info.GetBR()) / 2;
  // }

  if (armor_info.HaveLeftBar() && armor_info.HaveRightBar()) {
    image_point_lists[0] = armor_info.GetTL();
    image_point_lists[1] = armor_info.GetBL();
    image_point_lists[2] = (armor_info.GetTL() + armor_info.GetBL()) / 2;

    image_point_lists[3] = armor_info.GetTR();
    image_point_lists[4] = armor_info.GetBR();
    image_point_lists[5] = (armor_info.GetTR() + armor_info.GetBR()) / 2;
  } else if (armor_info.HaveLeftBar()) {
    image_point_lists[0] = armor_info.GetTL();
    image_point_lists[1] = armor_info.GetBL();
    image_point_lists[2] = (armor_info.GetTL() + armor_info.GetBL()) / 2;
  } else if (armor_info.HaveRightBar()) {
    image_point_lists[3] = armor_info.GetTR();
    image_point_lists[4] = armor_info.GetBR();
    image_point_lists[5] = (armor_info.GetTR() + armor_info.GetBR()) / 2;
  }

  int believable;
  // TODO 等待增加无灯条结算：使用

  if (!armor_info.HaveLeftBar() && !armor_info.HaveRightBar())
    believable = 0;
  else if (armor_info.HaveLeftBar() && armor_info.HaveRightBar())
    believable = 1;
  else
    believable = armor_info.HaveLeftBar() ? 2 : 3;
  armor = tdttoolkit::RobotArmor(
      armor_info.GetArmorRotRect(), armor_info.GetDPArmorType(),
      image_point_lists, Point2f(0, 0), believable, armor_info.Getconf());
}

bool ArmorDetector::GetEligibility(const LightBarInfo &lightbar1,
                                   const LightBarInfo &lightbar2) {
  LightBarInfo left_lightbar = lightbar1;
  LightBarInfo right_lightbar = lightbar2;
  if (left_lightbar.GetCenter().x > right_lightbar.GetCenter().x) {
    LightBarInfo tmp = left_lightbar;
    left_lightbar = right_lightbar;
    right_lightbar = tmp;
  }
  float left_lightbar_angle = left_lightbar.GetAngle();
  float right_lightbar_angle = right_lightbar.GetAngle();
  cv::Point left_lightbar_center = left_lightbar.GetCenter();
  cv::Point right_lightbar_center = right_lightbar.GetCenter();
  float length_min =
      std::min(left_lightbar.GetWidth(), right_lightbar.GetWidth());
  float length_max =
      std::max(left_lightbar.GetWidth(), right_lightbar.GetWidth());
  if (this->lock_) {
    float ratio =
        CalcDistance(left_lightbar_center, right_lightbar_center) / length_max;
    if ((last_robot_type_ == 1 || last_robot_type_ > 6) && ratio < 2.7)
      return false;
    if ((last_robot_type_ > 1 && last_robot_type_ < 7) && ratio > 3)
      return false;
  }
  if (fabs(left_lightbar.GetCenter().y - right_lightbar.GetCenter().y) >
      length_max) {
    if (fabs(left_lightbar.GetCenter().x - right_lightbar.GetCenter().x) >
        length_max)
      return false;
  }
  if (fabs(left_lightbar_angle - right_lightbar_angle) > 15) {
    return false;
  }
  float point_distance =
      CalcDistance(left_lightbar_center, right_lightbar_center);
  if ((point_distance > (6 * length_max)) || (point_distance < length_min)) {
    return false;
  }

  if (length_max / length_min > 2) {
    return false;
  }
  float tan_angle = 0;
  if (right_lightbar_center.x == left_lightbar_center.x) {  // 防止除0的尴尬情况
    tan_angle = 100;
  } else {
    tan_angle = atan(fabs((right_lightbar_center.y - left_lightbar_center.y) /
                          static_cast<float>((right_lightbar_center.x -
                                              left_lightbar_center.x))));
  }
  float grade = 200;  // 总分
  grade = grade - float((tan_angle)*180) -
          fabs(left_lightbar_angle - right_lightbar_angle);
  bool flag = grade > 100;
  return flag;
}

bool ArmorDetector::DP_GetEligibility(const LightBarInfo &lightbar1,
                                      const LightBarInfo &lightbar2) {
  LightBarInfo left_lightbar = lightbar1;
  LightBarInfo right_lightbar = lightbar2;
  if (left_lightbar.GetCenter().x > right_lightbar.GetCenter().x) {
    LightBarInfo tmp = left_lightbar;
    left_lightbar = right_lightbar;
    right_lightbar = tmp;
  }
  float left_lightbar_angle = left_lightbar.GetAngle();
  float right_lightbar_angle = right_lightbar.GetAngle();
  cv::Point left_lightbar_center = left_lightbar.GetCenter();
  cv::Point right_lightbar_center = right_lightbar.GetCenter();
  float length_min =
      std::min(left_lightbar.GetWidth(), right_lightbar.GetWidth());
  float length_max =
      std::max(left_lightbar.GetWidth(), right_lightbar.GetWidth());
  // if (this->lock_) {
  float ratio =
      CalcDistance(left_lightbar_center, right_lightbar_center) / length_max;
  //   if ((last_robot_type_ == 1 || last_robot_type_ > 6) && ratio < 2.7)

  //     return false;
  //   if ((last_robot_type_ > 1 && last_robot_type_ < 7) && ratio > 3)

  //     return false;
  // }
  if (ratio < 0.7 || ratio > 5) {
    if (ifdebug) {
      cout << "ratio:" << ratio << endl;
      cout << "ratio error" << endl;
    }
    return false;
  }
  // if (ifdebug) {
  //   cout<<"left_y"<<left_lightbar.GetCenter().y<<"
  //   right_y"<<right_lightbar.GetCenter().y<<endl;
  //   cout<<"left_x"<<left_lightbar.GetCenter().x<<"
  //   right_x"<<right_lightbar.GetCenter().x<<endl; cout << length_max << endl;
  // }
  // if (fabs(left_lightbar.GetCenter().y - right_lightbar.GetCenter().y) >
  //     length_max) {
  //   // if (fabs(left_lightbar.GetCenter().x - right_lightbar.GetCenter().x) >
  //       // length_max)
  //       if (ifdebug) {
  //         cout<<"left_y"<<left_lightbar.GetCenter().y<<"
  //         right_y"<<right_lightbar.GetCenter().y<<endl;
  //         cout<<"left_x"<<left_lightbar.GetCenter().x<<"
  //         right_x"<<right_lightbar.GetCenter().x<<endl; cout << length_max <<
  //         endl;
  //     cout << "灯条匹配xy error" << endl;
  //   }
  //     return false;
  // }
  if (fabs(left_lightbar_angle - right_lightbar_angle) > 15) {
    if (ifdebug) {
      cout << "装甲板匹配:灯条匹配angle error" << endl;
    }
    return false;
  }
  // float point_distance =
  //     CalcDistance(left_lightbar_center, right_lightbar_center);
  // if ((point_distance > (6 * length_max)) || (point_distance < length_min)) {
  //   return false;
  // }

  if (length_max / length_min > 2) {
    if (ifdebug) {
      cout << "装甲板匹配:灯条匹配长宽比error" << endl;
    }
    return false;
  }
  // float tan_angle = 0;
  // if (right_lightbar_center.x == left_lightbar_center.x) {  //
  // 防止除0的尴尬情况
  //   tan_angle = 100;
  // } else {
  //   tan_angle = atan(fabs((right_lightbar_center.y - left_lightbar_center.y)
  //   /
  //                         static_cast<float>((right_lightbar_center.x -
  //                                             left_lightbar_center.x))));
  // }
  // float grade = 200;  // 总分
  // grade = grade - float((tan_angle)*180) -
  // fabs(left_lightbar_angle - right_lightbar_angle);
  bool flag = true;
  // if(!flag)
  // {
  // if(ifdebug)
  // {
  // cout<<"装甲板匹配:灯条匹配分数"<<grade<<endl;
  // }
  // }
  return flag;
}

void ArmorDetector::Logic_fix(std::vector<int> &flags,
                              std::vector<float> &confs) {
  /*
   * 逻辑清单：
   * 基地和前哨站不可能同时出现
   * 如果p为正数哨兵不可能出现
   * 等待补充
   *
   */
  float pitch = receiveMessage.shoot_platform.pitch;
  int outpost_flag = 0;
  int base_flag = 0;
  for (int i = 0; i != flags.size(); i++) {
    if (flags.at(i) == 7 && pitch > 0) {
      flags.at(i) = 0;
      continue;
    }
    if (flags.at(i) == 6) {
      outpost_flag = 1;
    }
    if (flags.at(i) == 8) {
      base_flag = 1;
    }
  }
  if (outpost_flag == 1 && base_flag == 1) {
    for (int i = 0; i != flags.size(); i++) {
      if (flags.at(i) == 8) {
        flags.at(i) = 6;
      }
    }
  }
}

void ArmorDetector::CalAxis(std::vector<tdttoolkit::RobotArmor> &final_armors) {
  if (final_armors.size() > 1) {
    for (int i = 0; i < (final_armors.size() - 1); i++) {
      for (int j = i + 1; j < final_armors.size(); j++) {
        if (final_armors[i].GetRobotType() == final_armors[j].GetRobotType()) {
          if (final_armors[i].Getaxis().x == 0 &&
              final_armors[j].Getaxis().x == 0) {
            auto max_rect = final_armors[i].GetStickerRect().GetArea() >
                                    final_armors[j].GetStickerRect().GetArea()
                                ? final_armors[i].GetStickerRect()
                                : final_armors[j].GetStickerRect();
            auto min_rect = final_armors[i].GetStickerRect().GetArea() <
                                    final_armors[j].GetStickerRect().GetArea()
                                ? final_armors[i].GetStickerRect()
                                : final_armors[j].GetStickerRect();
            auto r =
                max_rect.GetArea() / (max_rect.GetArea() + min_rect.GetArea());
            float x_distance = max_rect.GetCenter().x - min_rect.GetCenter().x;
            final_armors[i].Setaxis(
                Point(max_rect.GetCenter().x - x_distance * (1 - r),
                      final_armors[i].GetStickerRect().GetCenter().y));
            final_armors[j].Setaxis(
                Point(max_rect.GetCenter().x - x_distance * (1 - r),
                      final_armors[i].GetStickerRect().GetCenter().y));
          }
        }
      }
    }
  }
}

#ifdef use_openvino
void ArmorDetector::roiFindLightbar(
    vector<tdtml::YoloDetector::Object> &detected_armors, cv::Mat &src) {
  for (auto i = 0; i < detected_armors.size(); ++i) {
    cv::Mat roi_armors(src(detected_armors[i].rect));
    std::vector<LightBarInfo> light_bars(0);
    std::vector<ArmorDetectInfo> armors_pro(0);
    DP_FindLightBar(roi_armors, light_bars);
    for (auto j = 0; j < light_bars.size(); j++) {
      light_bars[j].SetCenter(
          Point2f(light_bars[j].GetCenter2f().x + detected_armors[i].rect.x,
                  light_bars[j].GetCenter2f().y + detected_armors[i].rect.y));
    }
    if (light_bars.size() <= 1) {
      detected_armors[i] = detected_armors.back();
      detected_armors.pop_back();
      i--;
      continue;
    }

    DP_DoubleLightBarDetect(src, light_bars, armors_pro);
    if (armors_pro.size() == 0) {
      detected_armors[i] = detected_armors.back();
      detected_armors.pop_back();
      i--;
      continue;
    } else if (armors_pro.size() > 1) {
      /*选择最大的装甲板
      int max_area = 0;
      int max_index = 0;
      for (int j = 0; j < armors_pro.size(); j++) {
        if (armors_pro[j].GetArmorRotRect().GetArea() > max_area) {
          max_area = armors_pro[j].GetArmorRotRect().GetArea();
          max_index = j;
        }
      }
      armors_pro[0] = armors_pro[max_index];
      armors_pro.resize(1);
      */
      // 因为低阈值时中间容易被识别成灯条且匹配成装甲板，选最贴近装甲板外框的装甲板
      int distance = 100000;
      int max_index = 0;
      for (int j = 0; j < armors_pro.size(); j++) {
        int leftBar_x = (armors_pro[j].GetLeftBarRect().GetTl().x +
                         armors_pro[j].GetLeftBarRect().GetBl().x) /
                        2;
        int rightBar_x = (armors_pro[j].GetRightBarRect().GetTr().x +
                          armors_pro[j].GetRightBarRect().GetBr().x) /
                         2;
        int leftDistance = abs(leftBar_x - detected_armors[i].rect.x);
        int rightDistance = abs(rightBar_x - (detected_armors[i].rect.x +
                                              detected_armors[i].rect.width));
        int Distance = leftDistance + rightDistance;
        if (Distance < distance) {
          distance = Distance;
          max_index = j;
        }
      }
      armors_pro[0] = armors_pro[max_index];
      armors_pro.resize(1);
    }

    auto leftLight = armors_pro[0].GetLeftBarRect();
    auto rightLight = armors_pro[0].GetRightBarRect();

    detected_armors[i].points[0] =
        (leftLight.GetTl().x + leftLight.GetTr().x) / 2;
    detected_armors[i].points[1] =
        (leftLight.GetTl().y + leftLight.GetTr().y) / 2;
    detected_armors[i].points[2] =
        (leftLight.GetBl().x + leftLight.GetBr().x) / 2;
    detected_armors[i].points[3] =
        (leftLight.GetBl().y + leftLight.GetBr().y) / 2;
    detected_armors[i].points[4] =
        (rightLight.GetTl().x + rightLight.GetTr().x) / 2;
    detected_armors[i].points[5] =
        (rightLight.GetTl().y + rightLight.GetTr().y) / 2;
    detected_armors[i].points[6] =
        (rightLight.GetBl().x + rightLight.GetBr().x) / 2;
    detected_armors[i].points[7] =
        (rightLight.GetBl().y + rightLight.GetBr().y) / 2;

    float leftLightLenth = leftLight.GetWidth();
    float rightLightLenth = rightLight.GetWidth();
    cv::Point2f leftLightCenter = leftLight.GetCenter2f();
    cv::Point2f rightLightCenter = rightLight.GetCenter2f();

    float avg_Length = (leftLightLenth + rightLightLenth) / 2;
    float center_distance = cv::norm(leftLightCenter - rightLightCenter);
    float length_ratio = avg_Length / center_distance;
    if (ifdebug) {
      std::cout << "avg_Length:" << avg_Length << std::endl;
      std::cout << "center_distance:" << center_distance << std::endl;
      std::cout << "length_ratio:" << length_ratio << std::endl;
    }
    if (length_ratio < 0.36) {
      detected_armors[i].big_armor = true;
    }

    // float leftLightLenth = sqrt(
    //     pow(detected_armors[i].points[2] - detected_armors[i].points[0], 2)
    //     +
    //         pow(detected_armors[i].points[3] -
    //         detected_armors[i].points[1],
    //             2),
    //     2);
    // float rightLightLenth = sqrt(
    //     pow(detected_armors[i].points[6] - detected_armors[i].points[4], 2)
    //     +
    //         pow(detected_armors[i].points[7] -
    //         detected_armors[i].points[5],
    //             2),
    //     2);
    // float avg_LightLenth = (leftLightLenth_rightLightLenth) / 2;
  }
}
void ArmorDetector::extractNumber(
    vector<tdtml::YoloDetector::Object> &detected_armors, cv::Mat &src) {
  const int light_length = 12;
  // Image size after warp
  const int warp_height = 28;
  const int small_armor_width = 32;
  const int large_armor_width = 54;
  // Number ROI size
  const cv::Size roi_size(20, 28);
  for (auto &armor : detected_armors) {
    auto amore_rect = armor.rect;
    cv::Point2f left_top = amore_rect.tl();

    cv::Point2f lights_vertices[4] = {
        cv::Point2f(armor.points[0], armor.points[1]),
        cv::Point2f(armor.points[2], armor.points[3]),
        cv::Point2f(armor.points[4], armor.points[5]),
        cv::Point2f(armor.points[6], armor.points[7])};
    // for (int i = 0; i < 4; i++) {
    //   lights_vertices[i] -= left_top;
    // }
    const int top_light_y = (warp_height - light_length) / 2 - 1;
    const int bottom_light_y = top_light_y + light_length;
    int warp_width;
    if (armor.big_armor == false) {
      warp_width = small_armor_width;
      if (ifdebug) {
        std::cout << "小装甲" << std::endl;
      }
    } else {
      warp_width = large_armor_width;
      if (ifdebug) {
        std::cout << "大装甲" << std::endl;
      }
    }
    cv::Point2f target_vertices[4] = {
        cv::Point(0, top_light_y),
        cv::Point(0, bottom_light_y),
        cv::Point(warp_width - 1, top_light_y),
        cv::Point(warp_width - 1, bottom_light_y),
    };
    cv::Mat number_image;
    auto rotation_matrix =
        cv::getPerspectiveTransform(lights_vertices, target_vertices);
    // cv::warpPerspective(src(amore_rect), number_image, rotation_matrix,
    //                     cv::Size(warp_width, warp_height));
    cv::warpPerspective(src, number_image, rotation_matrix,
                        cv::Size(warp_width, warp_height));
    number_image = number_image(
        cv::Rect(cv::Point((warp_width - roi_size.width) / 2, 0), roi_size));
    armor.number_image = number_image;
    // cv::imshow("src", src);
    // cv::resize(number_image, number_image, cv::Size(100, 100));
    // cv::imshow("number_image", number_image);
    // char key = cv::waitKey(1);
    // if (key == 'q') {
    //   break;
    // } else if (key == 's') {
    //   cv::imwrite("number_image.jpg", number_image);
    // } else if (key == 'p') {
    //   cv::waitKey(0);
    // }
  }
}
#endif
#ifdef use_tensorrt
void ArmorDetector::roiFindLightbar(
    vector<trt_detector::trt_yolo_detector::Object> &detected_armors,
    cv::Mat &src) {
  for (auto i = 0; i < detected_armors.size(); ++i) {
    cv::Mat roi_armors(src(detected_armors[i].rect));
    std::vector<LightBarInfo> light_bars(0);
    std::vector<ArmorDetectInfo> armors_pro(0);
    DP_FindLightBar(roi_armors, light_bars);
    for (auto j = 0; j < light_bars.size(); j++) {
      light_bars[j].SetCenter(
          Point2f(light_bars[j].GetCenter2f().x + detected_armors[i].rect.x,
                  light_bars[j].GetCenter2f().y + detected_armors[i].rect.y));
    }
    if (light_bars.size() <= 1) {
      detected_armors[i] = detected_armors.back();
      detected_armors.pop_back();
      i--;
      continue;
    }

    DP_DoubleLightBarDetect(src, light_bars, armors_pro);
    if (armors_pro.size() == 0) {
      detected_armors[i] = detected_armors.back();
      detected_armors.pop_back();
      i--;
      continue;
    } else if (armors_pro.size() > 1) {
      /*选择最大的装甲板
      int max_area = 0;
      int max_index = 0;
      for (int j = 0; j < armors_pro.size(); j++) {
        if (armors_pro[j].GetArmorRotRect().GetArea() > max_area) {
          max_area = armors_pro[j].GetArmorRotRect().GetArea();
          max_index = j;
        }
      }
      armors_pro[0] = armors_pro[max_index];
      armors_pro.resize(1);
      */
      // 因为低阈值时中间容易被识别成灯条且匹配成装甲板，选最贴近装甲板外框的装甲板
      int distance = 100000;
      int max_index = 0;
      for (int j = 0; j < armors_pro.size(); j++) {
        int leftBar_x = (armors_pro[j].GetLeftBarRect().GetTl().x +
                         armors_pro[j].GetLeftBarRect().GetBl().x) /
                        2;
        int rightBar_x = (armors_pro[j].GetRightBarRect().GetTr().x +
                          armors_pro[j].GetRightBarRect().GetBr().x) /
                         2;
        int leftDistance = abs(leftBar_x - detected_armors[i].rect.x);
        int rightDistance = abs(rightBar_x - (detected_armors[i].rect.x +
                                              detected_armors[i].rect.width));
        int Distance = leftDistance + rightDistance;
        if (Distance < distance) {
          distance = Distance;
          max_index = j;
        }
      }
      armors_pro[0] = armors_pro[max_index];
      armors_pro.resize(1);
    }

    auto leftLight = armors_pro[0].GetLeftBarRect();
    auto rightLight = armors_pro[0].GetRightBarRect();

    detected_armors[i].points[0] =
        (leftLight.GetTl().x + leftLight.GetTr().x) / 2;
    detected_armors[i].points[1] =
        (leftLight.GetTl().y + leftLight.GetTr().y) / 2;
    detected_armors[i].points[2] =
        (leftLight.GetBl().x + leftLight.GetBr().x) / 2;
    detected_armors[i].points[3] =
        (leftLight.GetBl().y + leftLight.GetBr().y) / 2;
    detected_armors[i].points[4] =
        (rightLight.GetTl().x + rightLight.GetTr().x) / 2;
    detected_armors[i].points[5] =
        (rightLight.GetTl().y + rightLight.GetTr().y) / 2;
    detected_armors[i].points[6] =
        (rightLight.GetBl().x + rightLight.GetBr().x) / 2;
    detected_armors[i].points[7] =
        (rightLight.GetBl().y + rightLight.GetBr().y) / 2;

    float leftLightLenth = leftLight.GetWidth();
    float rightLightLenth = rightLight.GetWidth();
    cv::Point2f leftLightCenter = leftLight.GetCenter2f();
    cv::Point2f rightLightCenter = rightLight.GetCenter2f();

    float avg_Length = (leftLightLenth + rightLightLenth) / 2;
    float center_distance = cv::norm(leftLightCenter - rightLightCenter);
    float length_ratio = avg_Length / center_distance;
    if (ifdebug) {
      std::cout << "avg_Length:" << avg_Length << std::endl;
      std::cout << "center_distance:" << center_distance << std::endl;
      std::cout << "length_ratio:" << length_ratio << std::endl;
    }
    if (length_ratio < 0.36) {
      detected_armors[i].big_armor = true;
    }

    // float leftLightLenth = sqrt(
    //     pow(detected_armors[i].points[2] - detected_armors[i].points[0], 2)
    //     +
    //         pow(detected_armors[i].points[3] -
    //         detected_armors[i].points[1],
    //             2),
    //     2);
    // float rightLightLenth = sqrt(
    //     pow(detected_armors[i].points[6] - detected_armors[i].points[4], 2)
    //     +
    //         pow(detected_armors[i].points[7] -
    //         detected_armors[i].points[5],
    //             2),
    //     2);
    // float avg_LightLenth = (leftLightLenth_rightLightLenth) / 2;
  }
}
void ArmorDetector::extractNumber(
    vector<trt_detector::trt_yolo_detector::Object> &detected_armors,
    cv::Mat &src) {
  const int light_length = 12;
  // Image size after warp
  const int warp_height = 28;
  const int small_armor_width = 32;
  const int large_armor_width = 54;
  // Number ROI size
  const cv::Size roi_size(20, 28);
  for (auto &armor : detected_armors) {
    auto amore_rect = armor.rect;
    cv::Point2f left_top = amore_rect.tl();

    cv::Point2f lights_vertices[4] = {
        cv::Point2f(armor.points[0], armor.points[1]),
        cv::Point2f(armor.points[2], armor.points[3]),
        cv::Point2f(armor.points[4], armor.points[5]),
        cv::Point2f(armor.points[6], armor.points[7])};
    // for (int i = 0; i < 4; i++) {
    //   lights_vertices[i] -= left_top;
    // }
    const int top_light_y = (warp_height - light_length) / 2 - 1;
    const int bottom_light_y = top_light_y + light_length;
    int warp_width;
    if (armor.big_armor == false) {
      warp_width = small_armor_width;
      if (ifdebug) {
        std::cout << "小装甲" << std::endl;
      }
    } else {
      warp_width = large_armor_width;
      if (ifdebug) {
        std::cout << "大装甲" << std::endl;
      }
    }
    cv::Point2f target_vertices[4] = {
        cv::Point(0, top_light_y),
        cv::Point(0, bottom_light_y),
        cv::Point(warp_width - 1, top_light_y),
        cv::Point(warp_width - 1, bottom_light_y),
    };
    cv::Mat number_image;
    auto rotation_matrix =
        cv::getPerspectiveTransform(lights_vertices, target_vertices);
    // cv::warpPerspective(src(amore_rect), number_image, rotation_matrix,
    //                     cv::Size(warp_width, warp_height));
    cv::warpPerspective(src, number_image, rotation_matrix,
                        cv::Size(warp_width, warp_height));
    number_image = number_image(
        cv::Rect(cv::Point((warp_width - roi_size.width) / 2, 0), roi_size));
    armor.number_image = number_image;
    // cv::imshow("src", src);
    // cv::resize(number_image, number_image, cv::Size(100, 100));
    // cv::imshow("number_image", number_image);
    // char key = cv::waitKey(1);
    // if (key == 'q') {
    //   break;
    // } else if (key == 's') {
    //   cv::imwrite("number_image.jpg", number_image);
    // } else if (key == 'p') {
    //   cv::waitKey(0);
    // }
  }
}

#endif
void ArmorDetector::DP_DoubleLightBarDetect(
    const cv::Mat &src, std::vector<LightBarInfo> &light_bars,
    std::vector<ArmorDetectInfo> &output_armors) {
  std::vector<std::vector<ArmorDetectInfo>> armors_detected_vector(
      light_bars.size() - 1);
  for (uint64_t i = 0; i < (light_bars.size() - 1); ++i) {
    for (uint64_t j = i + 1; j < light_bars.size(); j++) {
      if (!DP_GetEligibility(light_bars[i], light_bars[j]))
        continue;  // 灯条匹配判断

      std::vector<cv::Point> armor_points;

      armor_points.emplace_back(
          (light_bars[i].GetTl() + light_bars[i].GetTr()) / 2);
      armor_points.emplace_back(
          (light_bars[i].GetBl() + light_bars[i].GetBr()) / 2);
      armor_points.emplace_back(
          (light_bars[j].GetTl() + light_bars[j].GetTr()) / 2);
      armor_points.emplace_back(
          (light_bars[j].GetBl() + light_bars[j].GetBr()) / 2);

      tdttoolkit::CustomRect armor_rect(armor_points);

      ArmorDetectInfo tmp_armor(light_bars[i], light_bars[j], armor_rect, 145);
      tmp_armor.SetArmorRotRect(armor_rect);
      // Debug::AddCustomRect("数字区域范围", armor_rect, cv::Scalar(0, 0,
      // 255),
      //                      2);
      armors_detected_vector[i].push_back(tmp_armor);
    }
    if (!armors_detected_vector[i].empty()) {
      output_armors.insert(output_armors.end(),
                           armors_detected_vector[i].begin(),
                           armors_detected_vector[i].end());
    }

    std::vector<ArmorDetectInfo>().swap(armors_detected_vector[i]);
  }
  if (ifdebug) {
    std::cout << "装甲板数量" << output_armors.size() << std::endl;
  }
  std::vector<std::vector<ArmorDetectInfo>>().swap(armors_detected_vector);
}

int ArmorDetector::RegionOustThreshold(const cv::Mat &input_image,
                                       cv::Mat &output_image, int lr) {
  cv::Mat tmp;
  cv::Rect o_rect;
  switch (lr) {
    case -1:
      o_rect = cv::Rect(cv::Point(cvRound(0.5 * input_image.cols),
                                  cvRound(0.35 * input_image.rows)),
                        cv::Point(cvRound(0.9 * input_image.cols),
                                  cvRound(0.65 * input_image.rows)));
      break;

    case 1:
      o_rect = cv::Rect(cv::Point(cvRound(0.1 * input_image.cols),
                                  cvRound(0.35 * input_image.rows)),
                        cv::Point(cvRound(0.5 * input_image.cols),
                                  cvRound(0.65 * input_image.rows)));
      break;

    default:
      o_rect = cv::Rect(cv::Point(cvRound(0.3 * input_image.cols),
                                  cvRound(0.35 * input_image.rows)),
                        cv::Point(cvRound(0.7 * input_image.cols),
                                  cvRound(0.65 * input_image.rows)));
      break;
  }
  int threshold =
      cvRound(cv::threshold(input_image(o_rect), tmp, 0, 255, cv::THRESH_OTSU));
  cv::threshold(input_image, output_image, threshold, 255, cv::THRESH_BINARY);
  return threshold;
}

// TODO:存在一些特殊情况待考虑
void ArmorDetector::Judgement(std::vector<tdttoolkit::RobotArmor> &robot,
                              std::vector<tdttoolkit::RobotArmor> &armor) {
  if (!robot.empty()) {
    for (int i = 0; i < armor.size(); i++) {
      if (armor[i].GetDPRobotType() == 6 || armor[i].GetDPRobotType() == 14)
        continue;
      std::vector<int> ids;
      for (int j = 0; j < robot.size(); j++) {
        if (robot[j].Getrect().tl().x > armor[i].GetStickerRect().GetTl().x ||
            robot[j].Getrect().tl().y > armor[i].GetStickerRect().GetTl().y)
          continue;
        if (robot[j].Getrect().br().x < armor[i].GetStickerRect().GetBr().x ||
            robot[j].Getrect().br().y < armor[i].GetStickerRect().GetBr().y)
          continue;
        if (robot[j].GetDPRobotType() != 0 &&
            robot[j].GetDPRobotType() != armor[i].GetDPRobotType())
          continue;
        ids.push_back(j);
      }

      if (!ids.empty()) {
        armor[i].Settimes(ids.size());
        int temp_id = ids[0];
        int min =
            robot[ids[0]].Getrect().area() /
            (armor[i].GetStickerRect().GetRect() & robot[ids[0]].Getrect())
                .area();

        for (int a = 1; a < ids.size(); a++) {
          int unit =
              (armor[i].GetStickerRect().GetRect() & robot[ids[a]].Getrect())
                  .area();
          if (robot[ids[a]].Getrect().area() / unit < min) {
            min = robot[ids[a]].Getrect().area() / unit;
            temp_id = ids[a];
          }
        }
        armor[i].Setaxis(robot[temp_id].Getaxis());
        armor[i].Setrect(robot[temp_id].Getrect());
        robot[temp_id].SetDPRobotType(armor[i].GetDPRobotType());
      }
    }
    for (int b = 1; b < armor.size(); b++) {
      if (armor[b - 1].GetDPRobotType() == armor[b].GetDPRobotType() &&
          armor[b - 1].Getaxis() != armor[b].Getaxis()) {
        if (armor[b - 1].Gettimes() > armor[b].Gettimes()) {
          armor[b - 1].Setaxis(armor[b].Getaxis());
          armor[b - 1].Setrect(armor[b].Getrect());
        } else {
          armor[b].Setaxis(armor[b - 1].Getaxis());
          armor[b].Setrect(armor[b - 1].Getrect());
        }
      }
    }
  }
}

void ArmorDetector::DP_logicBalance(vector<tdttoolkit::RobotArmor> &armors) {
  if (balance_on == -1) {
    return;
  } else if (balance_on >= 111 && balance_on <= 222) {
    SetBalance(3, balance_on / 100);
    SetBalance(4, balance_on % 100 / 10);
    SetBalance(5, balance_on % 10);
    return;
  } else {
    for (int i = 0; i < armors.size(); i++) {
      DP_BalanceSort(armors[i]);
    }
  }
}

void ArmorDetector::DP_BalanceSort(tdttoolkit::RobotArmor &armor) {
  if (armor.GetRobotType() != 3 && armor.GetRobotType() != 4 &&
          armor.GetRobotType() != 5 ||
      armor.Getaxis().x == 0) {
    return;
  }

  float axis2armorCenter_x = fabs(
      armor.Getaxis().x -
      (armor.GetImagePointsList()[2].x + armor.GetImagePointsList()[5].x) / 2);

  float lightCenter_dis = sqrt(
      pow((armor.GetImagePointsList()[2].x - armor.GetImagePointsList()[5].x),
          2) +
      pow((armor.GetImagePointsList()[2].y - armor.GetImagePointsList()[5].y),
          2));

  std::cout << "zhengdui " << axis2armorCenter_x / lightCenter_dis << std::endl;

  if (axis2armorCenter_x / lightCenter_dis < 0.2) {
    if (balance_counter[armor.GetRobotType() - 3][0] > 2 &&
        balance_counter[armor.GetRobotType() - 3][1] > 2) {
      if (balance_counter[armor.GetRobotType() - 3][0] >
          balance_counter[armor.GetRobotType() - 3][1]) {
        balance_counter[armor.GetRobotType() - 3][0] = 1;
        balance_counter[armor.GetRobotType() - 3][1] = 0;
      } else if (balance_counter[armor.GetRobotType() - 3][0] <
                 balance_counter[armor.GetRobotType() - 3][1]) {
        balance_counter[armor.GetRobotType() - 3][0] = 0;
        balance_counter[armor.GetRobotType() - 3][1] = 1;
      }
    }

    float length1 = sqrt(
        (armor.GetImagePointsList()[0].x - armor.GetImagePointsList()[1].x) *
            (armor.GetImagePointsList()[0].x -
             armor.GetImagePointsList()[1].x) +
        (armor.GetImagePointsList()[0].y - armor.GetImagePointsList()[1].y) *
            (armor.GetImagePointsList()[0].y -
             armor.GetImagePointsList()[1].y));

    float length2 = sqrt(
        pow((armor.GetImagePointsList()[3].x - armor.GetImagePointsList()[4].x),
            2) +
        pow((armor.GetImagePointsList()[3].y - armor.GetImagePointsList()[4].y),
            2));

    float light_length = (length1 + length2) / 2;

    //  std::cout << "balancejudge "
    //            << light_length / lightCenter_dis << std::endl;

    if (light_length / lightCenter_dis < 0.53 &&
        light_length / lightCenter_dis > 0.41) {
      balance_counter[armor.GetRobotType() - 3][0] += 1;
    }

    else if (light_length / lightCenter_dis < 0.29) {
      balance_counter[armor.GetRobotType() - 3][1] += 1;
    }

    if (balance_counter[armor.GetRobotType() - 3][0] >
        balance_counter[armor.GetRobotType() - 3][1]) {
      SetBalance(armor.GetRobotType(), 1);
    } else if (balance_counter[armor.GetRobotType() - 3][0] <
               balance_counter[armor.GetRobotType() - 3][1]) {
      SetBalance(armor.GetRobotType(), 2);
    }
  }
}

bool ArmorDetector::inFrame(cv::Rect &rect) {
  if (rect.x < 2) {
    return false;
  }
  if (rect.y < 2) {
    return false;
  }
  if (rect.x + rect.width > this->src_width - 2) {
    return false;
  }

  if (rect.y + rect.height > this->src_height - 2) {
    return false;
  }
  return true;
}

void ArmorDetector::safe_rect(cv::Rect &rect) {
  if (rect.x < 0) {
    rect.x = 0;
  }
  if (rect.y < 0) {
    rect.y = 0;
  }
  if (rect.x + rect.width > this->src_width) {
    if (rect.x > this->src_width) {
      rect.x = this->src_width - 2;
    }

    rect.width = this->src_width - rect.x - 1;
  }

  if (rect.y + rect.height > this->src_height) {
    if (rect.y > this->src_height) {
      rect.y = this->src_height - 2;
    }

    rect.height = this->src_height - rect.y - 1;
  }
}
}  // namespace armor_detect
