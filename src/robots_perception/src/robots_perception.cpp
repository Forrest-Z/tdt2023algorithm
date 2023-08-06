#include "robots_perception.h"

#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <string>
namespace robots_perception {


RobotsPerception::RobotsPerception() : Node("robots_perception") {

  LoadParam::InitParam("Perception", "./config/Perception.jsonc");
  LoadParam::InitParam("PercepVision", "./config/vision_param.jsonc");
  LoadParam::ReadParam("Perception", "debug", this->debug);
  LoadParam::ReadParam("Perception", "record", this->record);
  LoadParam::ReadParam("Perception", "roiFindPoints", this->roiFindPoints);

  rclcpp::SubscriptionOptions sub_options;
  rclcpp::PublisherOptions pub_options;
  rclcpp::QoS qos_best_effort(
      10);  // 选择reliable reliability和transient_local
            // durability配置可以在通信中断后保留未传递的消息，并在重新连接后重新发送
  qos_best_effort.best_effort();  // 设置为可靠传输

  InitDeepLearing();
  InitCamera();
  if (record) {
    cam1_recoder.Init(("./video/1"));
    cam2_recoder.Init(("./video/2"));
    cam3_recoder.Init(("./video/3"));
    cam4_recoder.Init(("./video/4"));
  }

  this->images.resize(4);
  publisher1 = this->create_publisher<sensor_msgs::msg::CompressedImage>(
      "image1/compressed", 1);
  publisher2 = this->create_publisher<sensor_msgs::msg::CompressedImage>(
      "image2/compressed", 1);
  publisher3 = this->create_publisher<sensor_msgs::msg::CompressedImage>(
      "image3/compressed", 1);
  publisher4 = this->create_publisher<sensor_msgs::msg::CompressedImage>(
      "image4/compressed", 1);
  Mappublisher = this->create_publisher<sensor_msgs::msg::CompressedImage>(
      "Map/compressed", 1);

  sub_pointcloud_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
      "pointcloud", 10,
      std::bind(&RobotsPerception::pointcloud_callback, this,
                std::placeholders::_1));

  pub_Local =
      this->create_publisher<perception_interface::msg::Perception2Local>(
          "Percep2Local", 1);
  pub_Nav = this->create_publisher<perception_interface::msg::Perception2Nav>(
      "Percep2Nav", rclcpp::SensorDataQoS());
  pub_Vision =
      this->create_publisher<perception_interface::msg::Perception2Nav>(
          "Percep2Vision", 1);
}

RobotsPerception::~RobotsPerception() {
  // std::cout<<"西沟函数"<<std::endl;
  // if (record) {
  //   cam1_recoder.Release();
  //   cam2_recoder.Release();
  //   cam3_recoder.Release();
  //   cam4_recoder.Release();
  // }
}

void RobotsPerception::close() {
    std::cout<<"close函数"<<std::endl;

  if (record) {
    cam1_recoder.Release();
    cam2_recoder.Release();
    cam3_recoder.Release();
    cam4_recoder.Release();
  }
}
void RobotsPerception::InitDeepLearing() {
  LoadParam::ReadParam("PercepVision", "EnemyColor", this->enemyColor);
  LoadParam::ReadParam("Perception", "ArmorDetect_lightbarthre",
                       this->threshold_);

#ifdef use_openvino
  if (this->deeplearning_mode == 0) {
    if (this->numberDetector == nullptr) {
      this->numberDetector = new tdtml::NumberDetector();
      std::string model_path;
      LoadParam::ReadRobotParam("NumberDetectPath", model_path);
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
      LoadParam::ReadRobotParam("DeepLearningModePath", model_path);

      this->yolo_detector->init(model_path, dp_cof_thre, dp_nms_area_thre,
                                this->src_width, this->src_height,
                                this->gpu_mode);
    }
    if (this->openvino_number_detector == nullptr) {
      this->openvino_number_detector = new tdtml::openvino_number_detector();
      std::string model_path;
      LoadParam::ReadRobotParam("YoloNumberDetectPath", model_path);
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
  LoadParam::ReadParam("Perception", "srcH", srcH);
  LoadParam::ReadParam("Perception", "srcW", srcW);
  LoadParam::ReadParam("Perception", "gen_engine", gen_engine);
  LoadParam::ReadParam("Perception", "yolo_engine_name", yolo_engine_name);
  LoadParam::ReadParam("Perception", "yolo_onnx_name", yolo_onnx_name);
  LoadParam::ReadParam("Perception", "kNmsThresh", kNmsThresh);
  LoadParam::ReadParam("Perception", "kConfThresh", kConfThresh);
  LoadParam::ReadParam("Perception", "kBatchSize", kBatchSize);
  LoadParam::ReadParam("Perception", "roiFindPoints", roiFindPoints);

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
  LoadParam::ReadParam("Perception", "number_engine_name", number_engine_name);
  LoadParam::ReadParam("Perception", "number_onnx_name", number_onnx_name);

  if (this->trt_number_detector == nullptr) {
    this->trt_number_detector = new trt_detector::trt_number_detector;
    this->trt_number_detector->init(number_engine_name, number_onnx_name,
                                    gen_engine);
  }
#endif
}
void RobotsPerception::InitCamera() {
  // std::string path = "./config/camera"+std::to_string(i)+".jsonc";
  camera1 = new tdtcamera::VideoDebug("./config/camera0.jsonc");
  camera2 = new tdtcamera::VideoDebug("./config/camera1.jsonc");
  camera3 = new tdtcamera::VideoDebug("./config/camera2.jsonc");
  camera4 = new tdtcamera::VideoDebug("./config/camera3.jsonc");

  for (int i = 0; i < 4; i++) {
    cv::Mat camera_matrix;
    cv::Mat dist_coeffs;
    cv::Mat rvec_;
    cv::Mat tvec_;
    cv::Mat T_Livox_Cam_;
    cv::Mat T_Cam_Livox_;
    std::string path = "./config/camera" + std::to_string(i) + ".jsonc";
    LoadParam::InitParam("cam" + std::to_string(i), path);
    LoadParam::ReadParam("cam" + std::to_string(i), "camera_matrix",
                         camera_matrix);
    LoadParam::ReadParam("cam" + std::to_string(i), "distortion_coefficients",
                         dist_coeffs);
    LoadParam::ReadParam("cam" + std::to_string(i), "rvec", rvec_);
    LoadParam::ReadParam("cam" + std::to_string(i), "tvec", tvec_);

    matrix.push_back(camera_matrix);
    distCoeffs.push_back(dist_coeffs);
    rvecs.push_back(rvec_);
    tvecs.push_back(tvec_);
  }
  float angle = -45.0 * CV_PI / 180.0;
  float pitch=15.0* CV_PI / 180.0;
  cv::Vec3f euler_camera_to_world = {pitch, angle, 0.0};
  cv::Mat rotation_matrix_camera_to_world_ =
      EulerAnglesToRotationMatrix(euler_camera_to_world, true);
  cv::Mat tvec_camera_in_world_ = (cv::Mat_<double>(3, 1) << 0.0, 0.0, 13.8);
  rotation_matrix_camera_to_world.push_back(rotation_matrix_camera_to_world_);
  tvec_camera_in_world.push_back(tvec_camera_in_world_);

  angle = -135.0 * CV_PI / 180.0;
  euler_camera_to_world = {pitch, angle, 0.0};
  rotation_matrix_camera_to_world_ =
      EulerAnglesToRotationMatrix(euler_camera_to_world, true);
  tvec_camera_in_world_ = (cv::Mat_<double>(3, 1) << 0.0, 0.0, 13.8);
  rotation_matrix_camera_to_world.push_back(rotation_matrix_camera_to_world_);
  tvec_camera_in_world.push_back(tvec_camera_in_world_);

  angle = 135.0 * CV_PI / 180.0;
  euler_camera_to_world = {pitch, angle, 0.0};
  rotation_matrix_camera_to_world_ =
      EulerAnglesToRotationMatrix(euler_camera_to_world, true);
  tvec_camera_in_world_ = (cv::Mat_<double>(3, 1) << 0.0, 0.0, 13.8);
  rotation_matrix_camera_to_world.push_back(rotation_matrix_camera_to_world_);
  tvec_camera_in_world.push_back(tvec_camera_in_world_);

  angle = 59.0 * CV_PI / 180.0;
  euler_camera_to_world = {pitch, angle, 0.0};
  rotation_matrix_camera_to_world_ =
      EulerAnglesToRotationMatrix(euler_camera_to_world, true);
  tvec_camera_in_world_ = (cv::Mat_<double>(3, 1) << 0.0, 0.0, 13.8);
  rotation_matrix_camera_to_world.push_back(rotation_matrix_camera_to_world_);
  tvec_camera_in_world.push_back(tvec_camera_in_world_);

  Round_Map = cv::imread("./config/around.png");
}

std::vector<Robot> RobotsPerception::MatchRobots(
    std::vector<trt_detector::trt_yolo_detector::Object> &detected_armors,
    std::vector<trt_detector::trt_yolo_detector::Object> &detected_robots,
    cv::Mat &src, int caarmorID) {
  // 只有带装甲板的机器人
  std::vector<Robot> Robots_With_Armor;

  std::vector<tdttoolkit::RobotArmor> robot(detected_robots.size());
  std::vector<armor_detect::ArmorDetectInfo> armors;
  std::vector<tdttoolkit::RobotArmor> output_armors_data_package;
  for (int i = 0; i < detected_robots.size(); i++) {
    safe_rect(detected_robots[i].rect, src);
    cv::Point2f center = cv::Point2f(
        detected_robots[i].rect.x + detected_robots[i].rect.width / 2,
        detected_robots[i].rect.y + detected_robots[i].rect.height / 2);
    robot[i].Setaxis(center);
    robot[i].Setrect(detected_robots[i].rect);
  }
  armors = GetArmorInfo(detected_armors);
  EraseDuplicate(armors);
  std::sort(robot.begin(), robot.end(),
            [](const tdttoolkit::RobotArmor &a, const tdttoolkit::RobotArmor &b)
                -> bool { return a.Getrect().x < b.Getrect().x; });
  std::sort(armors.begin(), armors.end(),
            [](const armor_detect::ArmorDetectInfo &a,
               const armor_detect::ArmorDetectInfo &b) -> bool {
              return a.GetRect().x < b.GetRect().x;
            });
  output_armors_data_package = Info2Armor(armors);
  Judgement(robot, output_armors_data_package);
  output_armors_data_package = GetenemyArmor(output_armors_data_package, src);
  CalAxis(output_armors_data_package);

  DP_logicBalance(output_armors_data_package);
  for (auto &armor : output_armors_data_package) {
    armor.SetWorldPoints(armor.GetDPRobotType(), armor.IsBalance());
  }
  Resolve(output_armors_data_package, caarmorID);
  Robots_With_Armor = Armor2Robot(output_armors_data_package);

  return Robots_With_Armor;
}
void RobotsPerception::safe_rect(cv::Rect &rect, cv::Mat &src) {
  if (rect.x < 0) {
    rect.x = 0;
  }
  if (rect.y < 0) {
    rect.y = 0;
  }
  if (rect.x + rect.width > src.size().width) {
    if (rect.x > src.size().width) {
      rect.x = src.size().width - 2;
    }

    rect.width = src.size().width - rect.x - 1;
  }

  if (rect.y + rect.height > src.size().height) {
    if (rect.y > src.size().height) {
      rect.y = src.size().height - 2;
    }

    rect.height = src.size().height - rect.y - 1;
  }
}
bool RobotsPerception::inFrame(cv::Rect &rect, cv::Mat &src) {
  if (rect.x < 2) {
    return false;
  }
  if (rect.y < 2) {
    return false;
  }
  if (rect.x + rect.width > src.size().width - 2) {
    return false;
  }

  if (rect.y + rect.height > src.size().height - 2) {
    return false;
  }
  return true;
}
std::vector<armor_detect::ArmorDetectInfo> RobotsPerception::GetArmorInfo(
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

void RobotsPerception::DPArmorTransform(
    const armor_detect::ArmorDetectInfo &armor_info,
    tdttoolkit::RobotArmor &armor,
    int enemy_color) {  /////////////////////////////////计算图像点////////////////////////////////////
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

void RobotsPerception::EraseDuplicate(
    std::vector<armor_detect::ArmorDetectInfo> &output_armors) {
  if (!output_armors.empty()) {
    vector<vector<int>> v(17);
    for (int i = 0; i < output_armors.size(); i++) {
      int type = output_armors[i].GetDPArmorType();
      if (type <= 16 && type >= 0) v[type].push_back(i);
    }
    for (int i = 0; i < v.size(); i++) {
      int maxArmorSize = 2;
      if (i == 7 || i == 15) maxArmorSize = 1;

      if (v[i].size() > maxArmorSize) {
        std::vector<std::pair<armor_detect::ArmorDetectInfo, int>> temps;
        for (int j = 0; j < v[i].size(); j++) {
          std::pair<armor_detect::ArmorDetectInfo, int> temp =
              std::make_pair(output_armors[v[i][j]], v[i][j]);
          temps.push_back(temp);
        }
        sort(temps.begin(), temps.end(),
             [](std::pair<armor_detect::ArmorDetectInfo, int> &a,
                std::pair<armor_detect::ArmorDetectInfo, int> &b) -> bool {
               return (a.first.Getconf() > b.first.Getconf());
             });
        for (int k = maxArmorSize; k < v[i].size(); k++) {
          output_armors.erase(output_armors.begin() + temps[k].second);
        }
      }
    }
  }
}

void RobotsPerception::Judgement(std::vector<tdttoolkit::RobotArmor> &robot,
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

std::vector<tdttoolkit::RobotArmor> RobotsPerception::Info2Armor(
    std::vector<armor_detect::ArmorDetectInfo> &armors) {
  std::vector<tdttoolkit::RobotArmor> output_armors_data_package;
  for (auto &armor_info : armors) {
    tdttoolkit::RobotArmor output_armor;
    DPArmorTransform(armor_info, output_armor, this->enemy_color_);
    output_armors_data_package.emplace_back(output_armor);
  }
  return output_armors_data_package;
}

std::vector<tdttoolkit::RobotArmor> RobotsPerception::GetenemyArmor(
    std::vector<tdttoolkit::RobotArmor> &output_armors_data_package,
    cv::Mat &src) {
  std::vector<tdttoolkit::RobotArmor> final_armors;
  for (auto const &armor : output_armors_data_package) {
    cv::Rect2i temp_rect = armor.GetStickerRect().GetRect();
    if (!RectSafety(temp_rect, src)) continue;
    if (temp_rect.tl().x < 7 || temp_rect.br().x > (src.size().width - 7))
      continue;
    if (temp_rect.tl().y < 7 || temp_rect.br().y > (src.size().height - 7))
      continue;
    tdttoolkit::RobotArmor temp_ = armor;
    final_armors.push_back(temp_);
  }
  return final_armors;
}
void RobotsPerception::CalAxis(
    std::vector<tdttoolkit::RobotArmor> &final_armors) {
  if (final_armors.size() > 1) {
    for (int i = 0; i < (final_armors.size() - 1); i++) {
      for (int j = i + 1; j < final_armors.size(); j++) {
        if (final_armors[i].GetDPRobotType() ==
            final_armors[j].GetDPRobotType()) {
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
void RobotsPerception::DP_logicBalance(
    std::vector<tdttoolkit::RobotArmor> &final_armors) {
  for (auto &armor : final_armors) {
    if (armor.GetDPRobotType() != 3 && armor.GetDPRobotType() != 4 &&
            armor.GetDPRobotType() != 5 && armor.GetDPRobotType() != 11 &&
            armor.GetDPRobotType() != 12 && armor.GetDPRobotType() != 13 ||
        armor.Getaxis().x == 0) {
      return;
    }
    float axis2armorCenter_x =
        fabs(armor.Getaxis().x - (armor.GetImagePointsList()[2].x +
                                  armor.GetImagePointsList()[5].x) /
                                     2);

    float lightCenter_dis = sqrt(
        (armor.GetImagePointsList()[2].x - armor.GetImagePointsList()[5].x) *
            (armor.GetImagePointsList()[2].x -
             armor.GetImagePointsList()[5].x) +
        (armor.GetImagePointsList()[2].y - armor.GetImagePointsList()[5].y) *
            (armor.GetImagePointsList()[2].y -
             armor.GetImagePointsList()[5].y));

    // std::cout << "zhengdui " << axis2armorCenter_x / lightCenter_dis
    //           << std::endl;

    float length1 = sqrt(
        (armor.GetImagePointsList()[0].x - armor.GetImagePointsList()[1].x) *
            (armor.GetImagePointsList()[0].x -
             armor.GetImagePointsList()[1].x) +
        (armor.GetImagePointsList()[0].y - armor.GetImagePointsList()[1].y) *
            (armor.GetImagePointsList()[0].y -
             armor.GetImagePointsList()[1].y));

    float length2 = sqrt(
        (armor.GetImagePointsList()[3].x - armor.GetImagePointsList()[4].x) *
            (armor.GetImagePointsList()[3].x -
             armor.GetImagePointsList()[4].x) +
        (armor.GetImagePointsList()[3].y - armor.GetImagePointsList()[4].y) *
            (armor.GetImagePointsList()[3].y -
             armor.GetImagePointsList()[4].y));

    float light_length = (length1 + length2) / 2;

    if (light_length / lightCenter_dis < 0.53 &&
        light_length / lightCenter_dis > 0.41) {
      armor.setBalance(false);
    }

    else if (light_length / lightCenter_dis < 0.29) {
      armor.setBalance(true);
    }
  }
}

void RobotsPerception::Resolve(vector<RobotArmor> &armors, int &camera_id) {
  // TODO:根据相机id选择相机参数
  if (!armors.empty()) {
    cv::Mat camera_matrix_ = matrix[camera_id];
    cv::Mat dist_coeffs_ = distCoeffs[camera_id];
    // cv::Mat T_Cam_Livox_   = T_Cam_Livox[camera_id];
    cv::Mat T_Cam_Livox_ = (cv::Mat_<double>(4, 4) << 1., 0., 0., 0., 0., 1.,
                            0., 0., 0., 0., 1., -0.15, 0., 0., 0., 1.);

    cv::Mat rotation_matrix_camera_to_world_ =
        rotation_matrix_camera_to_world[camera_id];
    cv::Mat tvec_camera_in_world_ = tvec_camera_in_world[camera_id];
    // for (int i = 0; i < armors.size(); i++) {
    // auto      armor      = armors[i];
    // RobotType robot_type = armor.GetRobotType();
    for (auto &armor : armors) {
      auto aImagePointList = armor.GetImagePointsList();  // 装甲板的二维坐标
      auto aWorldPointList = armor.GetWorldPointsList();  // 装甲板的三维坐标

      std::vector<cv::Point3f> worldPointList;
      std::vector<cv::Point2f> imagePointList;

      for (int j = 0; j < aImagePointList.size(); ++j) {
        if (aImagePointList[j].x * aImagePointList[j].y != 0 &&
            (aWorldPointList[j].x != 0 || aWorldPointList[j].y != 0)) {
          imagePointList.push_back(aImagePointList[j]);
          worldPointList.push_back(aWorldPointList[j]);
        }
      }

      Mat pnp_rvec, pnp_tvec,
          // TVecObjInWorld = Mat(3, 1, CV_32F, Scalar::all(0)),
          RVecObjToWorld = Mat(3, 1, CV_32F, Scalar::all(0));
      cv::Mat TVecObjInRobot = cv::Mat::zeros(3, 1, CV_32F);
      cv::Mat TVecObjInArmor = cv::Mat::zeros(3, 1, CV_32F);

      if (imagePointList.size() < 4) {
        continue;
      }
      bool solved =
          solvePnP(worldPointList, imagePointList, camera_matrix_, dist_coeffs_,
                   pnp_rvec, pnp_tvec, false, cv::SOLVEPNP_IPPE);

      if (!solved) {
        continue;
      }

      float axis_length = 20;
      // cv::Mat axis_points = (Mat_<float>(4, 3) << axis_length, 0, 20, 0,
      //                        axis_length, 20, 0, 0, axis_length + 20, 0, 0,
      //                        20);
      cv::Mat axis_points = (Mat_<float>(4, 3) << axis_length, 0, 0, 0,
                             axis_length, 0, 0, 0, axis_length, 0, 0, 0);
      std::vector<Point2f> img_points;

      projectPoints(axis_points, pnp_rvec, pnp_tvec, camera_matrix_,
                    dist_coeffs_, img_points);

      // line(images[camera_id], img_points[3], img_points[0], Scalar(255, 0, 0),
      //      3);
      // line(images[camera_id], img_points[3], img_points[1], Scalar(0, 255, 0),
      //      3);
      // line(images[camera_id], img_points[3], img_points[2], Scalar(0, 0, 255),
      //      3);
      // imshow("Object Coordinate System", images[camera_id - 1]);
      // cv::waitKey(1);
      float distance = cv::norm(pnp_tvec);
      std::cout << "相机distance" << distance << std::endl;
      cv::Point3f center_Cam =
          Point3f(pnp_tvec.at<float>(0, 0), pnp_tvec.at<float>(1, 0),
                  pnp_tvec.at<float>(2, 0));
      cv::Point3f center_Livox = Cam_Livox3D(center_Cam, T_Cam_Livox_);
      cv::Mat tvec_camera_in_world = cv::Mat_<float>(tvec_camera_in_world_);
      std::cout << "pnp_tvec" << pnp_tvec << std::endl;
      // std::cout << "tvec_camera_in_world" << tvec_camera_in_world <<
      // std::endl;
      cv::Mat rotation_matrix_camera_to_world =
          cv::Mat_<float>(rotation_matrix_camera_to_world_);
      cv::Mat pnp_tvec_ = cv::Mat_<float>(pnp_tvec);
      TVecObjInArmor =
          rotation_matrix_camera_to_world_ * (pnp_tvec_ + tvec_camera_in_world);
      // std::cout << "TVecObjInArmor" << TVecObjInArmor << std::endl;

      cv::Mat tvec_armor_in_robot_ = (cv::Mat_<float>(3, 1) << 0.0, 0.0, 20);
      TVecObjInRobot =
          rotation_matrix_camera_to_world_ *
          (pnp_tvec_ + tvec_camera_in_world + tvec_armor_in_robot_);
      std::cout << "TVecObjInRobot" << TVecObjInRobot << std::endl;
      float distance2 = cv::norm(TVecObjInRobot);

            std::cout << "车体distance" << distance2 << std::endl;

      armor.SetCenterRobot(*TVecObjInRobot.ptr<Point3f>());
      armor.SetCenterCam(center_Cam);
      armor.SetCenterLivox(center_Livox);
      armor.SetCamId(camera_id);
      armor.SetSolved(1);
    }
  }
}

std::vector<Robot> RobotsPerception::Armor2Robot(
    std::vector<RobotArmor> &armors) {
  std::vector<Robot> robots;
  for (auto &armor : armors) {
    if (!armor.GetSolved()) continue;
    Robot robot;
    robot.pixelCentroid = armor.Getaxis();
    robot.camCentroid = armor.GetCenterCam();
    robot.livoxCentroid = armor.GetCenterLivox();
    robot.cameraID = armor.GetCamId();
    robot.armorID = armor.GetDPRobotType();
    robot.robotRect = armor.Getrect();
    robot.armorRect = armor.GetStickerRect().GetRect();
    robot.image_point_list_ = armor.GetImagePointsList();
    robot.world_point_list_ = armor.GetWorldPointsList();
    robot.center_Robot = armor.GetCenterRobot();
    robot.conf = armor.Getconf();
    robots.push_back(robot);
  }
  return robots;
}

cv::Point3f RobotsPerception::Cam_Livox3D(cv::Point3f &Camera_Point,
                                          cv::Mat &T_Livox_Cam) {
  cv::Point3f Cam_3D_Points;
  cv::Mat X(4, 1, cv::DataType<double>::type);  // 雷达系齐次坐标  4*1
  cv::Mat Y(4, 1, cv::DataType<double>::type);  // 相机系齐次坐标  4*1
  X.at<double>(0, 0) = Camera_Point.x;
  X.at<double>(1, 0) = Camera_Point.y;
  X.at<double>(2, 0) = Camera_Point.z;
  X.at<double>(3, 0) = 1;
  Y = T_Livox_Cam.inv() * X;  // 4*4*4*1=4*1
  Cam_3D_Points.x = Y.at<double>(0, 0) / Y.at<double>(3, 0);
  Cam_3D_Points.y = Y.at<double>(1, 0) / Y.at<double>(3, 0);
  Cam_3D_Points.z = Y.at<double>(2, 0) / Y.at<double>(3, 0);
  return Cam_3D_Points;
}

void RobotsPerception::Lacate(std::vector<Robot> &Robots) {
  for (size_t i = 0; i < Robots.size(); i++) {
    if (Robots[i].Robot_With_Armor) {
      //    Robots[i].SetWorldPoints(tdttoolkit::DPRobotType(Robots[i].armorID),
      //    tdttoolkit::DPRobotColor(Robots[i].armorID),
      //    tdttoolkit::DPRobotNumber(Robots[i].armorID));
    }
  }
}


      void RobotsPerception::showTheshold(std::vector<cv::Mat> &images){
        for (int i =0;i<images.size();i++)
        {
          cv::Mat img_gray;
          cvtColor(images[i],img_gray,COLOR_BGR2GRAY);
          cv::threshold(img_gray, img_gray, this->threshold_, 255, cv::THRESH_BINARY);
          cv::imshow(std::to_string(i),img_gray);
        }
        cv::waitKey(1);
      }


void RobotsPerception::run() {
  ReadData();
  if (!HasData()) {
    std::cout << "no data" << std::endl;
    return;
  }
  if (record) {
    cam1_recoder.Recorder(images[0]);
    cam2_recoder.Recorder(images[1]);
    cam3_recoder.Recorder(images[2]);
    cam4_recoder.Recorder(images[3]);
  }
  std::vector<std::vector<trt_detector::trt_yolo_detector::Object>>
      detected_armors(images.size());
  std::vector<std::vector<trt_detector::trt_yolo_detector::Object>>
      detected_robots(images.size());
  std::vector<Robot> finalRobots;
  auto start = std::chrono::high_resolution_clock::now();
  trt_yolo_detector->batch_detect(images, detected_armors, detected_robots);
  auto end = std::chrono::high_resolution_clock::now();
  std::chrono::duration<double> diff = end - start;
  std::cout << "batch_detect " << diff.count() * 1000 << "ms" << std::endl;

  for (size_t cam_id = 0; cam_id < images.size(); cam_id++) {
    std::vector<Robot> Robots;
    cv::Mat src = images[cam_id];
    for (int j = 0; j < detected_armors[cam_id].size(); j++) {
      if (!inFrame(detected_armors[cam_id][j].rect, src)) {
        detected_armors[cam_id][j] = detected_armors[cam_id].back();
        detected_armors[cam_id].pop_back();
        j--;
        std::cout << "out of frame" << std::endl;
      }
    }
    if (roiFindPoints) {
      roiFindLightbar(detected_armors[cam_id], src);
    }

    extractNumber(detected_armors[cam_id], src);
    if (detected_armors[cam_id].size() != 0) {
      auto start = std::chrono::high_resolution_clock::now();
      this->trt_number_detector->detect(detected_armors[cam_id], src);
      auto end = std::chrono::high_resolution_clock::now();
      std::chrono::duration<double> diff = end - start;
      std::cout << "num_detect " << diff.count() * 1000 << "ms" << std::endl;
    }
    Robots = MatchRobots(detected_armors[cam_id], detected_robots[cam_id], src,
                         cam_id);
    finalRobots.insert(finalRobots.end(), Robots.begin(), Robots.end());
  }

  finalRobots = removeDuplicate(finalRobots);
  auto process = std::chrono::high_resolution_clock::now();
  std::chrono::duration<double> diff_process = process - start;
  std::cout << "total time" << diff_process.count() * 1000 << "ms" << std::endl;
  Publish(finalRobots);
  if (debug) {
    PerceptionDebug(finalRobots);
  }
  auto end1 = std::chrono::high_resolution_clock::now();
  std::chrono::duration<double> diff1 = end1 - start;
  std::chrono::duration<double> diff2 = end1 - process;
  std::cout << "Publish&Debug " << diff2.count() * 1000 << "ms" << std::endl;
  std::cout << "run " << diff1.count() * 1000 << "ms" << std::endl;
}

void RobotsPerception::ReadData() {
  camera1->GetImage(frame1);
  camera2->GetImage(frame2);
  camera3->GetImage(frame3);
  camera4->GetImage(frame4);
  images[0] = frame1.cvimage_;
  images[1] = frame2.cvimage_;
  images[2] = frame3.cvimage_;
  images[3] = frame4.cvimage_;
  camera_time = this->now();
}
bool RobotsPerception::HasData() {
  for (size_t i = 0; i < images.size(); i++) {
    if (images[i].empty()) {
      images[i] = empty_frame;
      std::cout << "image " << i << " is empty" << std::endl;
    }
  }
  return true;
}
void RobotsPerception::Publish(std::vector<Robot> &Robots) {
  // perception_interface::msg::Perception2Local Perception2Local;
  // for(auto &robot:Robots){
  // perception_interface::msg::PerceptionRect PerceptionRect;
  //  PerceptionRect.position.push_back(robot.)
  // }
  perception_interface::msg::Perception2Nav Perception2Nav;
  for (auto &robot : Robots) {
    perception_interface::msg::PerceptionRobot PerceptionRobot;
    PerceptionRobot.class_id = robot.armorID;
    PerceptionRobot.center.x = robot.center_Robot.x;
    PerceptionRobot.center.y = robot.center_Robot.z;
    PerceptionRobot.center.z = 0;
    Perception2Nav.robots.push_back(PerceptionRobot);
  }
  Perception2Nav.num_robots = Robots.size();
  Perception2Nav.header.stamp = camera_time;
  pub_Nav->publish(Perception2Nav);

  perception_interface::msg::Perception2Nav Perception2Vision;
  for (auto &robot : Robots) {
    if ((robot.armorID < 9 && enemyColor == 0) ||
        robot.armorID > 8 && enemyColor == 2) {
      perception_interface::msg::PerceptionRobot PerceptionRobot;
      if (enemyColor == 2) {
        PerceptionRobot.class_id = robot.armorID - 8;
      } else if (enemyColor == 0) {
        PerceptionRobot.class_id = robot.armorID;
      }
      PerceptionRobot.center.x = robot.center_Robot.x;
      PerceptionRobot.center.y = robot.center_Robot.z;
      PerceptionRobot.center.z = 0;
      Perception2Vision.robots.push_back(PerceptionRobot);
    }
  }
  Perception2Vision.num_robots = Perception2Vision.robots.size();
  Perception2Vision.header.stamp = camera_time;
  pub_Vision->publish(Perception2Vision);
}

void RobotsPerception::roiFindLightbar(
    vector<trt_detector::trt_yolo_detector::Object> &detected_armors,
    cv::Mat &src) {
  for (auto i = 0; i < detected_armors.size(); ++i) {
    cv::Mat roi_armors(src(detected_armors[i].rect));
    std::vector<armor_detect::LightBarInfo> light_bars(0);
    std::vector<armor_detect::ArmorDetectInfo> armors_pro(0);
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
  }
}
void RobotsPerception::extractNumber(
    vector<trt_detector::trt_yolo_detector::Object> &detected_armors,
    cv::Mat &src) {
  const int light_length = 12;
  const int warp_height = 28;
  const int small_armor_width = 32;
  const int large_armor_width = 54;
  const cv::Size roi_size(20, 28);
  for (auto &armor : detected_armors) {
    auto amore_rect = armor.rect;
    cv::Point2f rect_left_top = amore_rect.tl();

    cv::Point2f left_top = cv::Point2f(armor.points[0], armor.points[1]);
    cv::Point2f left_bottom = cv::Point2f(armor.points[2], armor.points[3]);
    cv::Point2f right_top = cv::Point2f(armor.points[4], armor.points[5]);
    cv::Point2f right_bottom = cv::Point2f(armor.points[6], armor.points[7]);

    float leftLightLenth = cv::norm(left_top - left_bottom);
    float rightLightLenth = cv::norm(right_top - right_bottom);
    cv::Point2f leftLightCenter = (left_top + left_bottom) / 2;
    cv::Point2f rightLightCenter = (right_top + right_bottom) / 2;

    float avg_Length = (leftLightLenth + rightLightLenth) / 2;
    float center_distance = cv::norm(leftLightCenter - rightLightCenter);
    float length_ratio = avg_Length / center_distance;
    if (debug) {
      std::cout << "avg_Length:" << avg_Length << std::endl;
      std::cout << "center_distance:" << center_distance << std::endl;
      std::cout << "length_ratio:" << length_ratio << std::endl;
    }
    if (length_ratio < 0.36) {
      armor.big_armor = true;
    }

    cv::Point2f lights_vertices[4] = {left_top, left_bottom, right_top,
                                      right_bottom};
    // for (int i = 0; i < 4; i++) {
    //   lights_vertices[i] -= rect_left_top;
    // }
    const int top_light_y = (warp_height - light_length) / 2 - 1;
    const int bottom_light_y = top_light_y + light_length;
    int warp_width;
    if (armor.big_armor == false) {
      warp_width = small_armor_width;
      if (debug) {
        std::cout << "小装甲" << std::endl;
      }
    } else {
      warp_width = large_armor_width;
      if (debug) {
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

void RobotsPerception::DP_FindLightBar(
    const cv::Mat &src,
    std::vector<armor_detect::LightBarInfo> &output_light_bars) {
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
    if (debug) {
      std::cout << "所保存点的数量" << std::endl;
      std::cout << contours[i].size() << std::endl;
    }

    if (rot_rect.Vertical() < rot_rect.Cross() * 1.3) {
      if (debug) {
        std::cout << "当前轮廓id:" << i << std::endl;

        std::cout << "灯条宽高比例错误" << std::endl;
        std::cout << "最小高度差:" << rot_rect.Vertical()
                  << "最小宽度差:" << rot_rect.Cross();
      }
      continue;
    }

    if (rot_rect.GetSize().area() < 6 || rot_rect.GetSize().area() > 51000) {
      // 面积太小或者太大, 扔掉
      if (debug) {
        std::cout << "灯条面积面积错误" << std::endl;
      }
      continue;
    }
    if (rot_rect.GetAngle() > 155 || rot_rect.GetAngle() < 25) {
      if (debug) {
        std::cout << "灯条角度错误" << std::endl;
      }
      continue;
    }
    if (rot_rect.GetWidth() > rot_rect.GetHeight() * 16) {
      if (debug) {
        std::cout << "灯条另一种宽度比例错误" << std::endl;
        std::cout << rot_rect.GetWidth() << std::endl;
        std::cout << rot_rect.GetHeight() << std::endl;
      }
      continue;
    }

    armor_detect::LightBarInfo light_bar(rot_rect);
    output_light_bars.push_back(light_bar);
  }
  sort(output_light_bars.begin(), output_light_bars.end(),
       [](const armor_detect::LightBarInfo &a,
          const armor_detect::LightBarInfo &b) -> bool {
         return a.GetCenter().x < b.GetCenter().x;
       });
  if (debug) {
    std::cout << "灯条数量" << output_light_bars.size() << std::endl;
  }
}
bool RobotsPerception::DP_GetEligibility(
    const armor_detect::LightBarInfo &lightbar1,
    const armor_detect::LightBarInfo &lightbar2) {
  armor_detect::LightBarInfo left_lightbar = lightbar1;
  armor_detect::LightBarInfo right_lightbar = lightbar2;
  if (left_lightbar.GetCenter().x > right_lightbar.GetCenter().x) {
    armor_detect::LightBarInfo tmp = left_lightbar;
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
    if (debug) {
      cout << "ratio:" << ratio << endl;
      cout << "ratio error" << endl;
    }
    return false;
  }

  if (fabs(left_lightbar_angle - right_lightbar_angle) > 15) {
    if (debug) {
      cout << "装甲板匹配:灯条匹配angle error" << endl;
    }
    return false;
  }

  if (length_max / length_min > 2) {
    if (debug) {
      cout << "装甲板匹配:灯条匹配长宽比error" << endl;
    }
    return false;
  }
  bool flag = true;
  return flag;
}
void RobotsPerception::DP_DoubleLightBarDetect(
    const cv::Mat &src, std::vector<armor_detect::LightBarInfo> &light_bars,
    std::vector<armor_detect::ArmorDetectInfo> &output_armors) {
  std::vector<std::vector<armor_detect::ArmorDetectInfo>>
      armors_detected_vector(light_bars.size() - 1);
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

      armor_detect::ArmorDetectInfo tmp_armor(light_bars[i], light_bars[j],
                                              armor_rect, 145);
      tmp_armor.SetArmorRotRect(armor_rect);
      armors_detected_vector[i].push_back(tmp_armor);
    }
    if (!armors_detected_vector[i].empty()) {
      output_armors.insert(output_armors.end(),
                           armors_detected_vector[i].begin(),
                           armors_detected_vector[i].end());
    }

    std::vector<armor_detect::ArmorDetectInfo>().swap(
        armors_detected_vector[i]);
  }
  if (debug) {
    std::cout << "装甲板数量" << output_armors.size() << std::endl;
  }
  std::vector<std::vector<armor_detect::ArmorDetectInfo>>().swap(
      armors_detected_vector);
}

void RobotsPerception::PerceptionDebug(std::vector<Robot> &Robots) {
  cv::Mat Map = Round_Map.clone();
  std::vector<cv::Mat> imgs;
  for (int i=0;i<images.size();i++)
  {
    imgs.push_back(images[i].clone());
  }
  // auto img1 = images[0].clone();
  //   auto img2 = images[1].clone();
  // auto img3 = images[2].clone();
  // auto img4 = images[3].clone();

  for (auto &robot : Robots) {
    cv::rectangle(imgs[robot.cameraID], robot.armorRect,
                  cv::Scalar(0, 255, 0), 2);
    cv::rectangle(imgs[robot.cameraID], robot.robotRect,
                  cv::Scalar(0, 255, 0), 2);
    cv::circle(imgs[robot.cameraID], robot.pixelCentroid, 2,
               cv::Scalar(0, 255, 0), 2);

    cv::putText(imgs[robot.cameraID], std::to_string(robot.armorID),
                robot.pixelCentroid, cv::FONT_HERSHEY_SIMPLEX, 0.5,
                cv::Scalar(0, 255, 0), 2);
    std::cout << "识别：装甲板ID" << robot.armorID << std::endl;
    for (auto &point : robot.world_point_list_) {
      cv::circle(imgs[robot.cameraID], cv::Point(point.x, point.y), 2,
                 cv::Scalar(0, 255, 0), 2);
    }
    for (auto &point : robot.image_point_list_) {
      cv::circle(imgs[robot.cameraID], cv::Point(point.x, point.y), 2,
                 cv::Scalar(0, 255, 0), 2);
    }
    int x_around = robot.center_Robot.x + 300;
    int y_around = -robot.center_Robot.z + 400;
    cv::putText(Map, std::to_string(robot.armorID),
                cv::Point(x_around, y_around), 0, 0.5, cv::Scalar(0, 0, 255), 3,
                8);
  }
  sensor_msgs::msg::CompressedImage compressed_image_msg1;
  sensor_msgs::msg::CompressedImage compressed_image_msg2;
  sensor_msgs::msg::CompressedImage compressed_image_msg3;
  sensor_msgs::msg::CompressedImage compressed_image_msg4;
  sensor_msgs::msg::CompressedImage roundMap;
  compressed_image_msg1.format = "jpeg";
  compressed_image_msg2.format = "jpeg";
  compressed_image_msg3.format = "jpeg";
  compressed_image_msg4.format = "jpeg";
  roundMap.format = "jpeg";

  compressed_image_msg1.header.stamp = this->now();
  compressed_image_msg2.header.stamp = this->now();
  compressed_image_msg3.header.stamp = this->now();
  compressed_image_msg4.header.stamp = this->now();
  roundMap.header.stamp = this->now();

  cv::imencode(".jpg", imgs[0], compressed_image_msg1.data);
  cv::imencode(".jpg", imgs[1], compressed_image_msg2.data);
  cv::imencode(".jpg", imgs[2], compressed_image_msg3.data);
  cv::imencode(".jpg", imgs[3], compressed_image_msg4.data);
  cv::imencode(".jpg", Map, roundMap.data);

  publisher1->publish(compressed_image_msg1);
  publisher2->publish(compressed_image_msg2);
  publisher3->publish(compressed_image_msg3);
  publisher4->publish(compressed_image_msg4);
  Mappublisher->publish(roundMap);
}
void RobotsPerception::ClearDate() {
  image_buff1.setTo(0);
  image_buff2.setTo(0);
  image_buff3.setTo(0);
  image_buff4.setTo(0);
  images.assign(images.size(), cv::Mat());
}

void RobotsPerception::pointcloud_callback(
    const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::fromROSMsg(*msg, *cloud);
}

std::vector<robots_perception::Robot> RobotsPerception::removeDuplicate(
    std::vector<robots_perception::Robot> &robots) {
  std::vector<int> repeat_indexes;
  std::vector<robots_perception::Robot> final_robots;
  for (int j = 0; j < robots.size(); ++j) {
    std::vector<int>::iterator it =
        std::find(repeat_indexes.begin(), repeat_indexes.end(), j);
    if (it != repeat_indexes.end()) continue;

    bool repeat_flag = false;
    for (int k = j + 1; k < robots.size(); ++k) {
      if (robots[j].armorID == robots[k].armorID) {
        float dis =
            sqrt((robots[j].center_Robot.x - robots[k].center_Robot.x) *
                     (robots[j].center_Robot.x - robots[k].center_Robot.x) +
                 (robots[j].center_Robot.y - robots[k].center_Robot.y) *
                     (robots[j].center_Robot.y - robots[k].center_Robot.y));
        std::cout << "识别：距离" << dis << std::endl;
        if (dis < 50) {
          repeat_flag = true;
          repeat_indexes.push_back(k);
          Robot robot_temp = robots[j];
          robot_temp.center_Robot.x =
              (robots[j].center_Robot.x + robots[k].center_Robot.x) / 2;
          robot_temp.center_Robot.y =
              (robots[j].center_Robot.y + robots[k].center_Robot.y) / 2;
          final_robots.push_back(robot_temp);
          continue;
        }
      }
    }
    if (!repeat_flag) {
      final_robots.push_back(robots[j]);
    }
  }
  return final_robots;
}

}  // namespace robots_perception