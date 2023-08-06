#ifndef TDTVISION_RM2021_LIGHTBARINFO_H
#define TDTVISION_RM2021_LIGHTBARINFO_H
#include "roborts_utils/config.h"

// #include "tdt_config/config.h"
// #include "tdt_log/log.h"
// #include "toolkit/base_class.h"
// #include "toolkit/base_toolkit.h"
#include "roborts_utils/base_class.h"
#include "roborts_utils/base_toolkit.h"

using namespace tdttoolkit;
namespace armor_detect {
class LightBarInfo : public tdttoolkit::CustomRect {
 public:
  inline LightBarInfo() = default;
  inline explicit LightBarInfo(const tdttoolkit::CustomRect& rot_rect)
      : CustomRect(rot_rect) {
    this->find_status_ = 0;
  };
  inline int GetFindStatus() const { return find_status_; };
  inline void SetFindStatus(int find_status) {
    this->find_status_ = find_status;
  };

 private:
  //标志该灯条的左右两边是否被搜索过, -1代表左边被搜索过, 1代表右边,
  //0代表没有搜索过, 2代表全部搜索过
  int find_status_ = 0;
};
}  // namespace armor_detect
#endif  // TDTVISION_RM2021_LIGHTBARINFO_H
