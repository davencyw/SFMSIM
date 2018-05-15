#ifndef __SFMSIMULATOR_HH__
#define __SFMSIMULATOR_HH__

#include "cameramodel.hh"
#include "framesimulator.hh"
#include "global.hh"
#include "pointclassifier.hh"
#include "points.hh"

#include <deque>
#include <fstream>
#include <iostream>
#include <memory>

namespace sfmsimulator {

struct Sfmconfig {
  Sfmconfig() = default;
  Sfmconfig(cameramodel::Cameramodel camera) : cameramodel(camera) {}

  pointclassifier::Pointclassifier_type type_pointclassifier;
  cameramodel::Cameramodel cameramodel;

  // reconstruction settings
  size_t slidingwindow_size = 20;

  // noise parameter
  precision_t camera_noise_amount = 0.3;
  precision_t image_detection_noise_amount = -1;
  precision_t world_position_noise_amount = 0.3;

  // 0 camera_poses
  // 1 static_3d_landmarks
  // 2 dynamic_3d_landmarks
  // 3 output_filename
  std::array<std::string, 4> filepaths;
};

class Sfmsimulator {
public:
  Sfmsimulator(Sfmconfig config);
  Sfmsimulator(std::string config_file_path);

  void run();
  inline void enableVisualization() { _visualize = true; }
  inline void disableVisualization() { _visualize = false; }

  ~Sfmsimulator() {
    _fstream_output_weights->close();
    _fstream_output_camera_trajectory->close();
    _fstream_output_camera_trajectory_groundtruth->close();
    std::cout << "\n\n\n\n";
  }

private:
  void step();
  void updateSlidingWindow();
  void addCameraNoise();
  void addImageDetectionNoise();
  void addWorldPositionNoise();

  // model configuration
  const Sfmconfig _config;
  std::unique_ptr<pointclassifier::Pointclassifier> _pointclassifier;
  const cameramodel::Cameramodel _cameramodel;
  framesimulator::Framesimulator _framesimulator;
  bool _hold_first_camera = true;

  // pointweights
  array_t _weights;

  // scene window
  std::deque<std::shared_ptr<points::Points2d>> _scene_window_image;
  std::deque<vec6_t> _scene_window_cameraposes;
  std::shared_ptr<points::Points3d> _world_points;

  // estimates
  std::vector<std::vector<vec6_t>> _scene_full_camera_estimate;

  // simulation variables
  size_t _step = 0;
  bool _visualize = 0;

  // noise parameter
  std::mt19937 _rn_generator;

  // outputstream
  std::string _file_output = "";
  std::unique_ptr<std::ofstream> _fstream_output_weights;
  std::unique_ptr<std::ofstream> _fstream_output_camera_trajectory;
  std::unique_ptr<std::ofstream> _fstream_output_camera_trajectory_groundtruth;
};
} // namespace sfmsimulator

#endif /* end of include guard: __SFMSIMULATOR_HH__ */
