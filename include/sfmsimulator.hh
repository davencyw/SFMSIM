#ifndef __SFMSIMULATOR_HH__
#define __SFMSIMULATOR_HH__

#include "cameramodel.hh"
#include "framesimulator.hh"
#include "global.hh"
#include "pointclassifier.hh"
#include "points.hh"

#include <memory>

namespace sfmsimulator {

struct Sfmconfig {
  Sfmconfig() = default;
  Sfmconfig(cameramodel::Cameramodel camera) : cameramodel(camera) {}

  pointclassifier::Pointclassifier_type type_pointclassifier;

  // 0 camera_poses
  // 1 static_3d_landmarks
  // 2 dynamic_3d_landmarks
  std::vector<std::string> filepaths;

  cameramodel::Cameramodel cameramodel;
};

class Sfmsimulator {
 public:
  Sfmsimulator(Sfmconfig config)
      : _config(config),
        _cameramodel(config.cameramodel),
        _framesimulator(framesimulator::Framesimulator(config.filepaths,
                                                       config.cameramodel)) {}
  // TODO(dave): write run
  void run();
  void doSteps(const size_t steps);
  void step();
  void enablevisualization() { _visualize = true; }
  void disablevisualization() { _visualize = false; }

 private:
  // calls opencv::sfm::reconstruct to estimate camera odometry and 3d landmarks
  void reconstruct(std::shared_ptr<points::Points2d> points_frame1,
                   std::shared_ptr<points::Points2d> points_frame2);

  const Sfmconfig _config;
  const framesimulator::Framesimulator _framesimulator;
  const cameramodel::Cameramodel _cameramodel;

  std::unique_ptr<pointclassifier::Pointclassifier> _pointclassifier;

  // simulation variables
  size_t _step = 0;
  bool _visualize = 0;
};
}  // namespace sfmsimulator

#endif /* end of include guard: __SFMSIMULATOR_HH__ */
