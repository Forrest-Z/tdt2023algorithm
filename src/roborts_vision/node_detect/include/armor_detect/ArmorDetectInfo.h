#ifndef TDTVISION_RM2021_ARMORDETECTINFO_H
#define TDTVISION_RM2021_ARMORDETECTINFO_H

#include "armor_detect/LightBarInfo.h"
#include "opencv2/opencv.hpp"
#include "vector"
// #include "tdt_config/config.h"
// #include "tdt_log/log.h"
#include "roborts_utils/config.h"
#include "roborts_utils/base_class.h"
#include "roborts_utils/base_toolkit.h"

// #include "toolkit/base_class.h"
// #include "toolkit/base_toolkit.h"
using namespace std;
using namespace cv;
using namespace tdttoolkit;
namespace armor_detect {
class ArmorDetectInfo {
 public:
  ArmorDetectInfo() = default;
  ArmorDetectInfo(const tdttoolkit::CustomRect &number_rot_rect,
                  int threshold) {
    this->threshold_ = threshold;
    this->number_rot_rect_ = number_rot_rect;
    this->angle = number_rot_rect.GetAngle();
  }
  ArmorDetectInfo(const LightBarInfo &signal_light_bar,
                  const tdttoolkit::CustomRect &number_rot_rect, int lr,
                  int threshold) {
    this->threshold_ = threshold;
    this->number_rot_rect_ = number_rot_rect;
    this->angle = number_rot_rect.GetAngle();
    if (lr == -1) {
      this->have_right_bar_ = true;
      this->right_bar_rect_ = tdttoolkit::CustomRect(
          signal_light_bar);  // NOLINT(cppcoreguidelines-slicing)
                              // //强制类型转换,丢掉查找状态信息
      return;
    }
    this->have_left_bar_ = true;
    this->left_bar_rect_ = tdttoolkit::CustomRect(
        signal_light_bar);  // NOLINT(cppcoreguidelines-slicing)
                            // //强制类型转换,丢掉查找状态信息
  }
  ArmorDetectInfo(const LightBarInfo &left_light_bar,
                  const LightBarInfo &right_light_bar,
                  const tdttoolkit::CustomRect &number_rot_rect,
                  int threshold) {
    this->number_rot_rect_ = number_rot_rect;
    this->angle = number_rot_rect.GetAngle();
    this->have_left_bar_ = true;
    this->have_right_bar_ = true;
    this->threshold_ = threshold;
    this->right_bar_rect_ = tdttoolkit::CustomRect(
        right_light_bar);  // NOLINT(cppcoreguidelines-slicing)
                           // //强制类型转换,丢掉查找状态信息
    this->left_bar_rect_ = tdttoolkit::CustomRect(
        left_light_bar);  // NOLINT(cppcoreguidelines-slicing)
                          // //强制类型转换,丢掉查找状态信息
  }

  ArmorDetectInfo(const vector<Point2f> &left_light_bar,
                  const vector<Point2f> &right_light_bar,
                  const tdttoolkit::CustomRect &number_rot_rect,
                  int threshold) {
    this->number_rot_rect_ = number_rot_rect;
    this->angle = number_rot_rect.GetAngle();
    this->have_left_bar_ = true;
    this->have_right_bar_ = true;
    this->threshold_ = threshold;
    this->left_bar_ = left_light_bar;
    this->right_bar_ = right_light_bar;
  }

  inline tdttoolkit::RobotType GetArmorType() const {
    return this->armor_type_;
  };
  inline tdttoolkit::DPRobotType GetDPArmorType() const {
    return this->DP_armor_type_;
  };
  inline cv::Rect2i GetRect() const {
    return this->GetNumberRotRect().GetRect();
  };
  inline int GetThreshold() const { return this->threshold_; };
  inline float GetAngle() const { return this->angle; };
  inline tdttoolkit::CustomRect GetNumberRotRect() const {
    return this->number_rot_rect_;
  };
  inline tdttoolkit::CustomRect GetLeftBarRect() const {
    return this->left_bar_rect_;
  };
  inline tdttoolkit::CustomRect GetRightBarRect() const {
    return this->right_bar_rect_;
  };
  inline tdttoolkit::CustomRect GetArmorRotRect() const {
    return this->armor_rot_rect_;
  };
  inline Point2f GetTL() const { return this->left_bar_[0]; };
  inline Point2f GetBL() const { return this->left_bar_[1]; };
  inline Point2f GetTR() const { return this->right_bar_[0]; };
  inline Point2f GetBR() const { return this->right_bar_[1]; };

  inline bool HaveRightBar() const { return this->have_right_bar_; };
  inline bool HaveLeftBar() const { return this->have_left_bar_; };
  inline float Getconf() const { return this->conf; };
  inline void Setconf(float conf_) { this->conf = conf_; };

  inline void SetArmorRotRect(const tdttoolkit::CustomRect &armor_rot_rect) {
    this->armor_rot_rect_ = armor_rot_rect;
  };
  inline void SetArmorType(tdttoolkit::RobotType armor_type) {
    this->armor_type_ = armor_type;
  };
  inline void SetDPArmorType(tdttoolkit::DPRobotType DP_armor_type) {
    this->DP_armor_type_ = DP_armor_type;
  };

 private:
  tdttoolkit::RobotType armor_type_ = tdttoolkit::RobotType::TYPEUNKNOW;
  tdttoolkit::DPRobotType DP_armor_type_ = tdttoolkit::DPRobotType::No;
  tdttoolkit::CustomRect number_rot_rect_ =
      tdttoolkit::CustomRect(cv::Point2i(0, 0), cv::Size2i(0, 0), 0);
  tdttoolkit::CustomRect left_bar_rect_ =
      tdttoolkit::CustomRect(cv::Point2i(0, 0), cv::Size2i(0, 0), 0);
  tdttoolkit::CustomRect right_bar_rect_ =
      tdttoolkit::CustomRect(cv::Point2i(0, 0), cv::Size2i(0, 0), 0);
  tdttoolkit::CustomRect armor_rot_rect_ =
      tdttoolkit::CustomRect(cv::Point2i(0, 0), cv::Size2i(0, 0), 0);

  bool have_right_bar_ = false;
  bool have_left_bar_ = false;
  vector<Point2f> left_bar_;
  vector<Point2f> right_bar_;
  int threshold_ = 0;
  float conf = 0.0;
  float angle = 0;
};
}  // namespace armor_detect
#endif  // TDTVISION_RM2021_ARMORDETECTINFO_H
