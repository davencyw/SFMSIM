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
                 const cameramodel::Cameramodel cameramodel)
      : _file_camera_poses(file_camera_poses),
        _file_3d_static_landmarks(file_3d_static_landmarks),
        _file_3d_dynamic_landmarks(file_3d_dynamic_landmarks),
        _imageplane(cameramodel.getImageplane()),
        _K(cameramodel.getK()) {
    _fstream_camera_poses = std::make_unique<std::ifstream>();
    _fstream_3d_dynamic_landmarks = std::make_unique<std::ifstream>();

    // start opening camera poses file
    _fstream_camera_poses->open(_file_camera_poses);
    if (_fstream_camera_poses->good()) {
      *_fstream_camera_poses >> _header_camera_poses;
    } else {
      std::cout << "\n\nF A I L E D  READING CAMERA_POSES!\n\n\n";
    };

    // start opening dynamic landmark poses file
    _fstream_3d_dynamic_landmarks->open(_file_3d_dynamic_landmarks);
    if (_fstream_3d_dynamic_landmarks->good()) {
      *_fstream_3d_dynamic_landmarks >>
          std::get<0>(_header_3d_dynamic_landmarks) >>
          std::get<1>(_header_3d_dynamic_landmarks);
    } else {
      std::cout << "\n\nF A I L E D  READING DYNAMIC_LANDMARKS!\n\n\n";
    };

    // start opening static landmark poses file
    std::ifstream fstream_3d_static_landmarks;
    fstream_3d_static_landmarks.open(_file_3d_static_landmarks);
    if (fstream_3d_static_landmarks.good()) {
      fstream_3d_static_landmarks >> _header_3d_static_landmarks;

      _scene_3d_points =
          points::Points3d(_header_3d_static_landmarks +
                           std::get<1>(_header_3d_dynamic_landmarks));

      array_t* xposition_3d_all(&(_scene_3d_points.coord[0]));
      array_t* yposition_3d_all(&(_scene_3d_points.coord[1]));
      array_t* zposition_3d_all(&(_scene_3d_points.coord[2]));

      // start after dynamic points
      size_t global_landmark_index(std::get<1>(_header_3d_dynamic_landmarks));
      // read all static landmarks
      for (size_t landmark_i(0); landmark_i < _header_3d_static_landmarks;
           landmark_i++) {
        fstream_3d_static_landmarks >>
            (*xposition_3d_all)(global_landmark_index) >>
            (*yposition_3d_all)(global_landmark_index) >>
            (*zposition_3d_all)(global_landmark_index);
        ++global_landmark_index;
      }

    } else {
      std::cout << "\n\nF A I L E D  READING STATIC_LANDMARKS!\n\n\n";
    };
    fstream_3d_static_landmarks.close();
  };

  ~Framesimulator() {
    _fstream_camera_poses->close();
    _fstream_3d_dynamic_landmarks->close();
  }

  // projects current landmarks onto image plan of camerapose
  points::Points2d step_GetImagePoints();
  const size_t updatesLeft() const { return _header_camera_poses - _steps - 1; }

 private:
  // returns all 3d landmarks in the scene at the current step
  void update3dScenePoints();

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

  // position data
  array_t _xposition_3d_static_landmarks;
  array_t _yposition_3d_static_landmarks;
  array_t _zposition_3d_static_landmarks;

  // all landmarks
  points::Points3d _scene_3d_points;

  const cameramodel::Imageplane _imageplane;
  const cv::Matx33d _K;
};
}  // namespace sfmsimulator::framesimulator

#endif /* end of include guard: __FRAMESIMULATOR_HH__ */
