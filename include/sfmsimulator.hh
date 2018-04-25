#ifndef __SFMSIMULATOR_HH__
#define __SFMSIMULATOR_HH__

#include "cameramodel.hh"
#include "framesimulator.hh"
#include "global.hh"
#include "pointclassifier.hh"
#include "points.hh"

#include <deque>
#include <iostream>
#include <memory>

namespace sfmsimulator {

struct Sfmconfig {
  Sfmconfig() = default;
  Sfmconfig(cameramodel::Cameramodel camera) : cameramodel(camera) {}

  pointclassifier::Pointclassifier_type type_pointclassifier;
  cameramodel::Cameramodel cameramodel;

  // 0 camera_poses
  // 1 static_3d_landmarks
  // 2 dynamic_3d_landmarks
  std::vector<std::string> filepaths;
};

class Sfmsimulator {
public:
  Sfmsimulator(Sfmconfig config);
  Sfmsimulator(std::string config_file_path);

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

  // model configuration
  const Sfmconfig _config;
  framesimulator::Framesimulator _framesimulator;
  std::unique_ptr<pointclassifier::Pointclassifier> _pointclassifier;
  const cameramodel::Cameramodel _cameramodel;
  const cv::Matx33d _K;

  // scene window
  std::deque<std::shared_ptr<points::Points2d>> _scene_window;

  // simulation variables
  size_t _step = 0;
  bool _visualize = 0;
};
} // namespace sfmsimulator

#endif /* end of include guard: __SFMSIMULATOR_HH__ */
