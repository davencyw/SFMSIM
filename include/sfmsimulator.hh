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

struct Sfmreconstruction {
  std::vector<std::shared_ptr<mat44_t>> camerpose_estimate;
  std::shared_ptr<points::Points3d> point3d_estimate;
  std::vector<precision_t> reprojection_error;
};

class Sfmsimulator {
public:
  Sfmsimulator(Sfmconfig config);
  Sfmsimulator(std::string config_file_path);

  void run();
  void doSteps(const size_t steps);
  void step();
  inline void enableVisualization() { _visualize = true; }
  inline void disableVisualization() { _visualize = false; }

  ~Sfmsimulator() { std::cout << "\n\n\n\n"; }

private:
  // model configuration
  const Sfmconfig _config;
  std::unique_ptr<pointclassifier::Pointclassifier> _pointclassifier;
  const cameramodel::Cameramodel _cameramodel;
  framesimulator::Framesimulator _framesimulator;

  // scene window
  std::deque<std::shared_ptr<points::Points2d>> _scene_window_image;
  std::deque<std::shared_ptr<points::Points3d>> _scene_window_world;
  std::deque<std::shared_ptr<mat44_t>> _scene_camera_poses;

  // simulation variables
  size_t _step = 0;
  bool _visualize = 0;
};
} // namespace sfmsimulator

#endif /* end of include guard: __SFMSIMULATOR_HH__ */
