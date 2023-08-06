#ifndef __BASR_BACKEND_H
#define __BASR_BACKEND_H
#include "roborts_utils/roborts_utils.h"

class BaseResult {
 public:
  bool empty_ = true;

  virtual tdttoolkit::Vec2d calcSpeed(double t) = 0;
  virtual tdttoolkit::Vec2d calcPosition(double t) = 0;

  inline bool isEmpty() { return empty_; }
};

namespace navigation {
class BaseNavigationBackend {};

}  // namespace navigation

#endif