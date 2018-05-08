#ifndef __FRAMESIMULATOR_HH__
#define __FRAMESIMULATOR_HH__

#include "cameramodel.hh"
#include "global.hh"
#include "points.hh"

#include <fstream>
#include <iostream>
#include <memory>
#include <random>
#include <string>

namespace sfmsimulator::framesimulator {
class Framesimulator {
public:
  Framesimulator(const std::vector<std::string> filepaths,
                 const cameramodel::Cameramodel cameramodel,
                 const bool noise = false)
      : Framesimulator(filepaths[0], filepaths[1], filepaths[2], cameramodel,
                       noise) {}

  Framesimulator(const std::string file_camera_poses,
                 const std::string file_3d_static_landmarks,
                 const std::string file_3d_dynamic_landmarks,
                 const cameramodel::Cameramodel cameramodel,
                 const bool noise = false);

  ~Framesimulator() {
    _fstream_camera_poses->close();
    _fstream_3d_dynamic_landmarks->close();
  }

  // projects current landmarks onto image plan of camerapose
  size_t getNumPoints() const;
  points::Points2d getImagePoints() const;
  points::Points3d getWorldPoints() const;
  mat44_t getCameraPoseMat() const;
  vec6_t getCameraPose() const;

  void step();
  const inline size_t updatesLeft() const {
    return _header_camera_poses - _steps;
  }

private:
  // updates all 3d landmarks in the scene at the current step
  void update3dScenePoints();
  // update camerapose to current pose
  void updateCameraPose();

  // image detection noise
  void inline addNoise(precision_t *ux, precision_t *uy) {
    *ux += _d(_gen);
    *uy += _d(_gen);
  }
  std::mt19937 _gen;
  std::normal_distribution<> _d;

  // filepaths
  const std::string _file_camera_poses;
  const std::string _file_3d_static_landmarks;
  const std::string _file_3d_dynamic_landmarks;

  // fstreams
  std::unique_ptr<std::ifstream> _fstream_camera_poses;
  std::unique_ptr<std::ifstream> _fstream_3d_dynamic_landmarks;

  // number of total points
  size_t _numtotalpoints = 0;
  // number of frames
  size_t _header_camera_poses;
  // number of landmarks
  size_t _header_3d_static_landmarks;
  // number of frames, number of landmarks
  std::tuple<size_t, size_t> _header_3d_dynamic_landmarks;
  // number of executed steps / updates
  size_t _steps = 0;

  // noise flag
  bool _noise;

  const cameramodel::Imageplane _imageplane;
  const mat33_t _K_eigen;

  // current step data
  points::Points2d _step_image_points;
  points::Points3d _step_world_points;
  vec6_t _step_camera_pose;
  mat44_t _step_camera_pose_mat;
};
} // namespace sfmsimulator::framesimulator

#endif /* end of include guard: __FRAMESIMULATOR_HH__ */
