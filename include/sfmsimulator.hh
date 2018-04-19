#ifndef __SFMSIMULATOR_HH__
#define __SFMSIMULATOR_HH__

#include "cameramodel.hh"
#include "framesimulator.hh"
#include "global.hh"
#include "pointclassifier.hh"
#include "points.hh"

#include <memory>

namespace sfmsimulator {
class Sfmsimulator {
 public:
 private:
  // calls opencv::sfm::reconstruct to estimate camera odometry and 3dlandmarks
  mat44_t reconstruct(std::shared_ptr<points::Points2d> points_frame1,
                      std::shared_ptr<points::Points2d> points_frame2);

  framesimulator::Framesimulator _framesimulator;
  pointclassifier::Pointclassifier _pointclassifier;
  cameramodel::Cameramodel _cameramodel;
};
}  // namespace sfmsimulator

#endif /* end of include guard: __SFMSIMULATOR_HH__ */
