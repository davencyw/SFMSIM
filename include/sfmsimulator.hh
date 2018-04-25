#ifndef __SFMSIMULATOR_HH__
#define __SFMSIMULATOR_HH__

#include "cameramodel.hh"
#include "framesimulator.hh"
#include "global.hh"
#include "pointclassifier.hh"
#include "points.hh"

#include <iostream>
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
  Sfmsimulator(Sfmconfig config);
  Sfmsimulator(std::string config_file_path);

  // TODO(dave): write run
  void run();
  void doSteps(const size_t steps);
  void step();
  inline void enablevisualization() { _visualize = true; }
  inline void disablevisualization() { _visualize = false; }

  ~Sfmsimulator() { std::cout << "\n\n\n\n"; }

private:
  // calls opencv::sfm::reconstruct to estimate camera odometry and 3d landmarks
  void reconstruct(std::shared_ptr<points::Points2d> points_frame1,
                   std::shared_ptr<points::Points2d> points_frame2);

  const Sfmconfig _config;
  framesimulator::Framesimulator _framesimulator;
  const cameramodel::Cameramodel _cameramodel;

  std::unique_ptr<pointclassifier::Pointclassifier> _pointclassifier;

  // simulation variables
  size_t _step = 0;
  bool _visualize = 0;
};
} // namespace sfmsimulator

#endif /* end of include guard: __SFMSIMULATOR_HH__ */
