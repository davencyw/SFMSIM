#ifndef __CAMERAMODEL_HH__
#define __CAMERAMODEL_HH__

#include "global.hh"

#include <opencv2/core/core.hpp>

namespace sfmsimulator::cameramodel {

class Cameramodel {
 public:
  const cv::Matx33d getK() const {
    return cv::Matx33d(_f, 0, _cx, 0, _f, _cy, 0, 0, 1);
  }

 private:
  precision_t _f;
  precision_t _cx;
  precision_t _cy;
  mat33_t _intrinsics;
};

}  // namespace sfmsimulator::cameramodel

#endif /* end of include guard: __CAMERAMODEL_HH__ */
