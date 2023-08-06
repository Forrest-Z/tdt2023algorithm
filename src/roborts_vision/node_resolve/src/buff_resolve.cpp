#include "roborts_resolve/buff_resolve.h"

#include "geometry_msgs/msg/point32.hpp"

namespace tdtbuff {

void tdtbuff::Buff::UnifyCoordinate(cv::Mat &rvec, cv::Mat &tvec,
                                    tdttoolkit::ShootPlatform shootPlatform,
                                    bool to_world) {
  // tvec = t(物体系 -> 相机系)

  /// R(物体系 ->　相机系)*（相机系　->　世界系) = R(物体系->世界系)
  /// R(物体系 ->　世界系)*（世界系  ->　相机系) =R(物体系 ->相机系）

  //(|①)
  // ①:平移矩阵表示单位，旋转矩阵表示转轴

  // 世界系->相机系的欧拉角             (|相机系)
  cv::Vec3f euler_camera_to_world = {-shootPlatform.pitch, -shootPlatform.yaw,
                                     0};

  /// 世界系原点在相机坐标系下的坐标取反　　 (|相机系)　
  cv::Vec3f vec_tvec_camera_in_world = {-1., -4.93, 10.46};  // 世界到相机

  if (to_world) {
    // 相机系到世界系的平移矩阵      (|相机系)
    cv::Mat tvec_camera_in_world = cv::Mat(vec_tvec_camera_in_world);

    cv::Mat rotation_matrix_camera_to_world, rotation_matrix_object_to_camera;
    // 相机到世界的旋转矩阵           (|世界系)
    rotation_matrix_camera_to_world =
        tdttoolkit::EulerAnglesToRotationMatrix(euler_camera_to_world, true);
    // 物体到世界的平移矩阵(物体 -> 相机 + 相机 -> 世界)               (|世界系)
    tvec = rotation_matrix_camera_to_world * (tvec + tvec_camera_in_world);  //
    // 物体到相机的旋转矩阵               (|相机系)
    tdttoolkit::Rodrigues(
        rvec,
        rotation_matrix_object_to_camera);  // 旋转向量转换为旋转矩阵(物体到相机)

    // 物体->相机*相机->世界 >> 物体到世界的旋转矩阵 >> 旋转向量 (|世界系)
    tdttoolkit::Rodrigues(
        rotation_matrix_camera_to_world * rotation_matrix_object_to_camera,
        rvec);

  } else {
    // 世界坐标系到相机坐标系的欧拉角          (|相机系)
    euler_camera_to_world = -euler_camera_to_world;
    // 世界坐标系到相机坐标系的平移向量         (|相机系)
    vec_tvec_camera_in_world = -vec_tvec_camera_in_world;
    // 世界坐标系到相机坐标系的平移矩阵         (|相机系)
    cv::Mat tvec_world_in_camera = cv::Mat(vec_tvec_camera_in_world);

    cv::Mat rotation_matrix_world_to_camera, rotation_matrix_object_to_world;
    // 世界坐标系到相机坐标系的旋转矩阵
    rotation_matrix_world_to_camera =
        tdttoolkit::EulerAnglesToRotationMatrix(euler_camera_to_world, false);

    // 相机坐标系到世界坐标系的平移矩阵 :
    // 物体->相机的旋转矩阵*世界->物体旋＋物体->相机的平移矩阵　>>世界到相机的平移矩阵　(|相机系)
    tvec = rotation_matrix_world_to_camera * (tvec) + tvec_world_in_camera;
    // 物体到世界的旋转矩阵
    tdttoolkit::Rodrigues(rvec, rotation_matrix_object_to_world);
    // 物体到相机的旋转矩阵       物体->世界*世界->相机  >>　物体->相机
    tdttoolkit::Rodrigues(
        rotation_matrix_world_to_camera * rotation_matrix_object_to_world,
        rvec);
  }
}

double tdtbuff::Buff::Pixel2Angle(double pixel, char axis, cv::Mat cam_matrix) {
  if (axis == 'y' || axis == 'Y') {
    return atan(pixel / cam_matrix.at<double>(0, 0));
  } else {
    return atan(pixel / cam_matrix.at<double>(1, 1));
  }
}

// 由于误差较大，现在不主动解算R的距离，而是直接用700表示结果，该段代码暂且无用
double tdtbuff::Buff::GetRDistance(float H, cv::Point2f center_point,
                                   int src_hight, cv::Mat cam_matrix_,
                                   float pixel_y, float pixel_z,
                                   tdttoolkit::ShootPlatform shoot_platform_) {
  double pixel_pitch_angle = tdtbuff::Buff::Pixel2Angle(
      src_hight / 2.0 - center_point.y, 'y', cam_matrix_);
  double angle1 = atan(pixel_y / pixel_z);
  double angle2 = CV_PI / 2.0 + shoot_platform_.pitch - angle1;
  double y = fabs(sqrt(pixel_z * pixel_z + pixel_y * pixel_y) * cos(angle2));
  // y=4;
  std::cout << "idea_dis_angle:" << pixel_pitch_angle << ' '
            << shoot_platform_.pitch << ' ' << y << std::endl;
  double true_angle = fabs(-pixel_pitch_angle + shoot_platform_.pitch);
  double distance = fabs((H - y) / sin(true_angle));
  return distance;
}

void tdtbuff::Buff::calc_idea_rt(bool trust_pnp_eular) {
  // 物体坐标系在世界坐标系理想的平移与旋转矩阵。
  this->idea_tvec_obj2cam_ = this->pnp_tvec_obj2cam_ * this->idea_R_distance_ /
                             cv::norm(this->pnp_tvec_obj2cam_);
  // this->idea_tvec_obj2cam_ = this->pnp_tvec_obj2cam_;
  this->idea_rvec_obj2cam_ = this->pnp_rvec_obj2cam_.clone();

  this->idea_tvec_obj2world_ = this->idea_tvec_obj2cam_.clone();
  this->idea_rvec_obj2world_ = this->idea_rvec_obj2cam_.clone();
  UnifyCoordinate(this->idea_rvec_obj2world_, this->idea_tvec_obj2world_,
                  this->mock_shootPlatform_, true);
  if (!trust_pnp_eular) {
    // buff_armors_[0].GetAngle() ==> [-180, 180]，正下方为180度，正上方为0
    // 这里对大符建的系y轴朝下，与像素角的系建的相反，因此要加上一个π，与世界坐标系建的相同
    float rotate_angle = CV_PI + this->buff_armors_[0].GetAngle();

    // 先设一个roll的初值，然后通过不断尝试新的roll并与投影点做差收敛到大符真实的roll
    // 经测试，这个初始值随便设，收敛速度很快的
    // TODO:idea_yaw需要求解，或者给定点赋值
    cv::Vec3f eular_object_to_world = {
        0, this->idea_yaw_, rotate_angle};  // 物体系相对虚拟世界系的欧拉角
    //        eular_object_to_world = { 0,0, rotate_angle};

    // 像素为单位的能量机关的半径

    // 物体到世界理想的旋转矩阵和平移矩阵
    std::vector<cv::Point3f>
        real_world_points;  // 流水灯装甲板物体坐标系下的点。
    tdttoolkit::WorldPointLists::GetBuffWorldPoints(0, real_world_points);
    std::vector<cv::Point2f> pro_image_points;
    cv::Mat rvec_obj2world;
    for (int i = 0; i < 15; i++) {
      rvec_obj2world =
          tdttoolkit::EulerAnglesToRotationMatrix(eular_object_to_world, true);
      tdttoolkit::Rodrigues(rvec_obj2world, rvec_obj2world);
      cv::Mat rvec_obi2cam = rvec_obj2world.clone();
      cv::Mat tvec_obi2cam = this->idea_tvec_obj2world_.clone();

      UnifyCoordinate(rvec_obi2cam, tvec_obi2cam, this->mock_shootPlatform_,
                      false);
      pro_image_points.clear();

      // 5个点都投影
      projectPoints(real_world_points, rvec_obi2cam, tvec_obi2cam,
                    camera_matrix, distortion_matrix, pro_image_points);
      //            cout<<pro_image_points<<endl;

      // 用虚拟云台得到的投影角应该和实际的像素角相同
      cv::Point2f dif = pro_image_points[1] - pro_image_points[0];
      float angle = atan2(dif.x, -dif.y);
      float diffangle = tdttoolkit::CalcAngleDifference(
          this->buff_armors_[0].GetAngle(), angle);
      if (diffangle < 0.001 && i > 5) {
        break;
      }
      eular_object_to_world[2] += diffangle;
    }

    // 获得物体到世界坐标系下理想的旋转向量
    this->idea_rvec_obj2world_ = rvec_obj2world.clone();
    this->idea_rvec_obj2cam_ = this->idea_rvec_obj2world_.clone();
    this->idea_tvec_obj2cam_ = this->idea_tvec_obj2world_.clone();
    UnifyCoordinate(this->idea_rvec_obj2cam_, this->idea_tvec_obj2cam_,
                    this->mock_shootPlatform_, false);
  }
  if (VIDEODEBUG) {
    std::vector<cv::Point2f> pro_image_points;
    std::vector<cv::Point3f> obj_points;
    for (int i = 0; i < 5; i++) {
      if (i == 0) {
        obj_points.push_back(
            tdttoolkit::WorldPointLists::GetBuffWorldPoints_Index(i, 1));
      } else {
        obj_points.push_back(
            tdttoolkit::WorldPointLists::GetBuffWorldPoints_Index(i, 0));
      }
    }

    // 计算物体系上的点投影到图像平面的点的位置
    projectPoints(obj_points, this->idea_rvec_obj2cam_,
                  this->idea_tvec_obj2cam_, camera_matrix, distortion_matrix,
                  pro_image_points);

    // if (tdtconfig::VIDEODEBUG) {
    //     for (int i = 0; i < 5; i++)
    //         Debug::AddCircle("ideaproject点", pro_image_points[i], 8,
    //         cv::Scalar(2, 110, 255));
    // }
  }
}

/**
 * @class Buff
 */
Buff::Buff() { empty_ = true; }

Buff::Buff(std::vector<tdttoolkit::BuffArmor> const &buff_armors,
           cv::Mat const &pnp_tvec_obj2cam, cv::Mat const &pnp_rvec_obj2cam,
           tdttoolkit::ShootPlatform shoot_platform, float idea_R_distance,
           float idea_yaw, bool trust_pnp_eular, bool disturb) {
  this->empty_ = false;
  this->buff_armors_ = buff_armors;
  this->pnp_tvec_obj2cam_ = pnp_tvec_obj2cam;
  this->pnp_rvec_obj2cam_ = pnp_rvec_obj2cam;
  this->idea_R_distance_ = idea_R_distance;
  this->idea_yaw_ = idea_yaw;
  this->shootPlatform_ = shoot_platform;
  this->pnp_tvec_obj2world_ = pnp_tvec_obj2cam.clone();
  this->pnp_rvec_obj2world_ = pnp_rvec_obj2cam.clone();
  this->disturbbuff = disturb;

  // 计算伪云台，用R标计算出的伪云台
  cv::Point3f R_world_point = *pnp_tvec_obj2cam.ptr<cv::Point3f>();
  tdttoolkit::Polar3f R_world_polar =
      tdttoolkit::RectangularToPolar(R_world_point);
  // TODO:虚拟云台的yaw是始终对准R标的，但pitch不是，原因待发现
  // 构建了一个虚拟的世界系，假设当前激活大符时云台的yaw=0，则R标相对当前云台的yaw=R.yaw，由于原云台的yaw向左为正，等价于x轴向左，因此要加上-号
  this->mock_shootPlatform_ =
      tdttoolkit::ShootPlatform({0, -R_world_polar.yaw});
  //        this->mock_shootPlatform_ = ShootPlatform({0, 0});
  //        this->mock_shootPlatform_              =shoot_platform;

  UnifyCoordinate(this->pnp_rvec_obj2world_, this->pnp_tvec_obj2world_,
                  this->mock_shootPlatform_, true);

  if (!disturbbuff)
    this->calc_idea_rt(
        trust_pnp_eular);  // 计算理想的位姿，像素有可能映射不回去。
}

tdtbuff::Buff tdtbuff::BuffResolver::BuffResolve(
    std::vector<tdttoolkit::BuffArmor> &buff_armors) {
  std::vector<cv::Point2f> image_points;
  std::vector<cv::Point3f> world_points;
  tdtbuff::Buff buffTarget;
  // 不使用pnp的解算结果，用重投影法求解roll
  bool trust_pnp = false;
  bool deep_learning_mode = false;
  image_points.push_back(buff_armors[0].GetCirclePoint());
  if (buff_armors.empty()) return Buff();
  // judge detection mode
  if (!deep_learning_mode)
    image_points.push_back(buff_armors[0].GetBuffCenter());
  else {
    image_points.push_back(
        buff_armors[0].GetBuffRect().GetCenter2f());  // 装甲版中心点(传统识别)
  }
  tdttoolkit::WorldPointLists::PushBuffWorldPoints(0, world_points);

  return buffTarget;
}

void tdtbuff::BuffResolver::subFromDetect(
    const vision_interface::msg::DetectPub &msg) {
  shootPlatform.yaw = msg.plat_form_yaw;
  shootPlatform.pitch = msg.plat_form_pitch;
}

void tdtbuff::BuffResolver::publish2Predict() {}

}  // namespace tdtbuff