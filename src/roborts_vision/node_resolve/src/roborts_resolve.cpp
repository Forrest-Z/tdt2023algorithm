#include "roborts_resolve/roborts_resolve.h"

#include "geometry_msgs/msg/point32.hpp"
namespace Resolve {

ArmorResolve::ArmorResolve() : rclcpp::Node("armor_resolve_node") {
  RCLCPP_INFO(this->get_logger(), "begin resolved ");
  LoadParam::InitParam("resolve", "./config/camera_param.jsonc");

  armorResolveSub = this->create_subscription<vision_interface::msg::DetectPub>(
      "armor_detection", 1,
      std::bind(&ArmorResolve::subFromDetect, this, std::placeholders::_1));

  // usartSub = this->create_subscription<vision_interface::msg::UsartPub>(
  //     "visionUsartPub", 1,
  //     std::bind(&ArmorResolve::visionUsartCallBack, this,
  //               std::placeholders::_1));

  armorResolvePub = this->create_publisher<vision_interface::msg::ResolvePub>(
      "armor_resolve", 1);

  LoadParam::ReadParam("resolve", "camera_matrix", cameraMatrix);  // 内参
  LoadParam::ReadParam("resolve", "dist_coeffs", distCoeffs);  // 畸变矩阵
  LoadParam::ReadParam("resolve", "tvec_camera_in_world",
                       TvecCameraInWorld_);  //相机在世界坐标系下的平移向量
}

void ArmorResolve::publish2Predict() {}

void ArmorResolve::subFromDetect(const vision_interface::msg::DetectPub &msg) {
  shootPlatform.yaw = msg.plat_form_yaw;
  shootPlatform.pitch = msg.plat_form_pitch;

  armorTime =
      msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9;  //单位 ：秒
  armorsNum = msg.num;

  //识别一帧画面所有装甲板
  for (int i = 0; i < armorsNum; i++) {
    Resolve::robotArmors singalArmor;
    singalArmor.armortype = msg.robot_armors[i].armortype;
    //灯条四点循环，单装甲板
    for (int j = 0; j < 6; j++) {
      cv::Point2f image_point =  //灯条四点像素坐标
          cv::Point2f(msg.robot_armors[i].armor_point_image[j].x,
                      msg.robot_armors[i].armor_point_image[j].y);
      cv::Point3f real_point =  //灯条四点世界坐标
          cv::Point3f(msg.robot_armors[i].armor_point_realword[j].x,
                      msg.robot_armors[i].armor_point_realword[j].y,
                      msg.robot_armors[i].armor_point_realword[j].z);
      singalArmor.armor_center_image +=
          image_point;  //装甲板中心像素点 由灯条四点取均值计算得到
      singalArmor.armor_point_image.push_back(image_point);
      singalArmor.armor_point_realword.push_back(real_point);
    }

    singalArmor.armor_center_image /= 4;
    singalArmor.robot_center_image =
        cv::Point2f(msg.robot_armors[i].robot_center_image.x,
                    msg.robot_armors[i].robot_center_image.y);

    singalArmor.armortype = msg.robot_armors[i].armortype;

    armors.push_back(singalArmor);
  }

  detectPub = msg;
  resolve();
}

void ArmorResolve::resolve() {
  //将装甲板按照像素x坐标从小到大排序
  sort(armors.begin(), armors.end(), [](robotArmors a, robotArmors b) -> bool {
    return (a.armor_center_image.x < b.armor_center_image.x);
  });

  auto resolved_armors = vision_interface::msg::ResolvePub();

  resolved_armors.header.frame_id = "armor_resolved";
  resolved_armors.header.stamp = detectPub.header.stamp;
  resolved_armors.num = armorsNum;

  if (armorsNum) {
    std::vector<vision_interface::msg::ResolvedArmor> resolved_armors_(
        armorsNum);

    for (int i = 0; i < armorsNum; i++) {
      cv::Mat pnp_rvec = cv::Mat::zeros(3, 1, CV_32F),
              pnp_tvec = cv::Mat::zeros(3, 1, CV_32F),
              TVecObjInWorld = cv::Mat::zeros(3, 1, CV_32F),
              RVecObjToWorld = cv::Mat::zeros(3, 1, CV_32F);

      resolved_armors_[i].armortype = armors[i].armortype;

      std::cout << "type= " << armors[i].armortype << std::endl;

      bool solvePnP_flag = false;

      solvePnP_flag =
          cv::solvePnP(armors[i].armor_point_realword,
                       armors[i].armor_point_image, cameraMatrix, distCoeffs,
                       pnp_rvec, pnp_tvec, false, cv::SOLVEPNP_IPPE);

      armors[i].solvePnP_flag = solvePnP_flag;

      if (!solvePnP_flag) {  //解算失败
        resolved_armors_[i] = vision_interface::msg::ResolvedArmor();
        continue;
      }

      UnifyCoordinate(pnp_rvec, pnp_tvec, TVecObjInWorld, RVecObjToWorld);
      //赋值进行改动措施
      // clang-format off
    geometry_msgs::msg::Point32
        tvec_world_armor ,
        rvec_world_armor,
        tvec_camera_armor,
        rvec_camera_armor;

      tvec_world_armor.x =TVecObjInWorld.at<float>(0, 0);
      tvec_world_armor.y=TVecObjInWorld.at<float>(1, 0);
      tvec_world_armor.z=TVecObjInWorld.at<float>(2, 0);
        

         rvec_world_armor.x=RVecObjToWorld.at<float>(0, 0);
         rvec_world_armor.y=RVecObjToWorld.at<float>(1, 0);
         rvec_world_armor.z=RVecObjToWorld.at<float>(2, 0);
       
         tvec_camera_armor.x=pnp_tvec.at<float>(0, 0); 
         tvec_camera_armor.y=pnp_tvec.at<float>(1, 0);
         tvec_camera_armor.z= pnp_tvec.at<float>(2, 0);
       
        rvec_camera_armor.x=pnp_rvec.at<float>(0, 0); 
        rvec_camera_armor.y=pnp_rvec.at<float>(1, 0);
        rvec_camera_armor.z=pnp_rvec.at<float>(2, 0);

    resolved_armors_[i].tvec_world_armor = tvec_world_armor;

    resolved_armors_[i].rvec_world_armor = rvec_world_armor;

    resolved_armors_[i].tvec_camera_armor = tvec_camera_armor;

    resolved_armors_[i].rvec_camera_armor = rvec_camera_armor;

    //添加灯条六点的像素坐标
    resolved_armors_[i].armor_point_image =
        detectPub.robot_armors[i].armor_point_image;

    resolved_armors_[i].robot_center_image =
        detectPub.robot_armors[i].robot_center_image;

    // std::cout << "TVecObjInWorld= " << TVecObjInWorld << std::endl;
  }
    resolved_armors.resolved_armors = resolved_armors_;

  }
  resolved_armors.plat_form_yaw = detectPub.plat_form_yaw;
  resolved_armors.plat_form_pitch = detectPub.plat_form_pitch;
  resolved_armors.mode = detectPub.mode;

  //发布解算结果

  armorResolvePub->publish(resolved_armors);
  armors.clear();  //解算结束 清空此帧装甲板数据
}
// clang-format on

//求得装甲板中心点在世界坐标系下的 平移矩阵与旋转向量
void ArmorResolve::UnifyCoordinate(cv ::Mat &pnp_rvec, cv ::Mat &pnp_tvec,

                                   cv::Mat TVecObjInWorld,
                                   cv::Mat RVecObjToWorld) {
  /// 统一数据类型为 float（CV_32F）, 类型不一致经常报错

  pnp_tvec = cv::Mat_<float>(pnp_tvec);

  // 世界系到相机系的欧拉角 (Z-Y-X欧拉角)
  cv::Vec3f euler_world_camera = {-shootPlatform.pitch, -shootPlatform.yaw, 0};

  cv::Mat rotation_matrix_world_camera =
      tdttoolkit::EulerAnglesToRotationMatrix(euler_world_camera,
                                              true);  // Z - Y - X
  TVecObjInWorld = rotation_matrix_world_camera *
                   (pnp_tvec + cv::Mat_<float>(TvecCameraInWorld_));

  // cv::Mat a = rotation_matrix_world_camera *
  // cv::Mat_<float>(TvecCameraInWorld_); std::cout<<"pitch =
  // "<<shootPlatform.pitch<<"  "<<shootPlatform.yaw<<std::endl;
  // std::cout<<"camera in world = "<< a<<std::endl;
  // std::cout<<"TVecObjInWorld= "<<TVecObjInWorld<<std::endl;
  // rotation_matrix_camera_to_world * (pnp_tvec + TvecCameraInWorld);

  // 物体系到相机系的旋转矩阵       R(物体系 -> 相机系)
  cv::Mat rotation_matrix_camera_object(3, 3, CV_32F);

  tdttoolkit::Rodrigues(pnp_rvec, rotation_matrix_camera_object);
  rotation_matrix_camera_object =
      cv::Mat_<float>(rotation_matrix_camera_object);

  // 物体系到世界系的旋转矩阵       R(物体系 -> 世界系)
  cv::Mat rotation_matrix_world_object =
      rotation_matrix_world_camera * rotation_matrix_camera_object;

  tdttoolkit::Rodrigues(rotation_matrix_world_object,
                        RVecObjToWorld);  // 旋转向量
}

}  // namespace Resolve