#ifndef __FRAMESIMULATOR_HH__
#define __FRAMESIMULATOR_HH__

#include "cameramodel.hh"
#include "global.hh"
#include "points.hh"

#include <fstream>
#include <iostream>
#include <memory>
#include <string>

namespace sfmsimulator::framesimulator {
class Framesimulator {
 public:
  Framesimulator(const std::vector<std::string> filepaths,
                 const cameramodel::Cameramodel cameramodel)
      : Framesimulator(filepaths[0], filepaths[1], filepaths[2], cameramodel) {}

  Framesimulator(const std::string file_camera_poses,
                 const std::string file_3d_static_landmarks,
                 const std::string file_3d_dynamic_landmarks,
                 const cameramodel::Cameramodel cameramodel);

  ~Framesimulator() {
    _fstream_camera_poses->close();
    _fstream_3d_dynamic_landmarks->close();
  }

  // projects current landmarks onto image plan of camerapose
  points::Points2d step_GetImagePoints();
  const inline size_t updatesLeft() const {
    return _header_camera_poses - _steps;
  }

 private:
  // updates all 3d landmarks in the scene at the current step
  void update3dScenePoints();

  // update camerapose to current pose
  void updateCameraPose();

  // filepaths
  const std::string _file_camera_poses;
  const std::string _file_3d_static_landmarks;
  const std::string _file_3d_dynamic_landmarks;

  // fstreams
  std::unique_ptr<std::ifstream> _fstream_camera_poses;
  std::unique_ptr<std::ifstream> _fstream_3d_dynamic_landmarks;

  // number of frames
  size_t _header_camera_poses;
  // number of landmarks
  size_t _header_3d_static_landmarks;
  // number of frames, number of landmarks
  std::tuple<size_t, size_t> _header_3d_dynamic_landmarks;
  // number of executed steps / updates
  size_t _steps = 0;

  // all landmarks
  points::Points3d _scene_3d_points;

  const cameramodel::Imageplane _imageplane;
  const cv::Matx33d _K;
};
}  // namespace sfmsimulator::framesimulator

#endif /* end of include guard: __FRAMESIMULATOR_HH__ */
