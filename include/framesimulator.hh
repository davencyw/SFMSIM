#ifndef __FRAMESIMULATOR_HH__
#define __FRAMESIMULATOR_HH__

#include "global.hh"

#include <fstream>
#include <iostream>
#include <memory>
#include <string>

namespace sfmsimulator::framesimulator {
class Framesimulator {
 public:
  Framesimulator(std::string file_camera_poses,
                 std::string file_3d_static_landmarks,
                 std::string file_3d_dynamic_landmarks)
      : _file_camera_poses(file_camera_poses),
        _file_3d_static_landmarks(file_3d_static_landmarks),
        _file_3d_dynamic_landmarks(file_3d_dynamic_landmarks) {
    _fstream_camera_poses = std::make_unique<std::ifstream>();
    _fstream_3d_dynamic_landmarks = std::make_unique<std::ifstream>();

    // start opening camera poses file
    _fstream_camera_poses->open(_file_camera_poses);
    if (_fstream_camera_poses->good()) {
      *_fstream_camera_poses >> _header_camera_poses;
    } else {
      std::cout << "\n\nF A I L E D  READING CAMERA_POSES!\n\n\n";
    };

    // start opening static landmark poses file
    std::ifstream fstream_3d_static_landmarks;
    fstream_3d_static_landmarks.open(_file_3d_static_landmarks);
    if (fstream_3d_static_landmarks.good()) {
      fstream_3d_static_landmarks >> _header_3d_static_landmarks;

      _xposition_3d_static_landmarks = array_t(_header_3d_static_landmarks);
      _yposition_3d_static_landmarks = array_t(_header_3d_static_landmarks);

      // read all landmarks
      for (size_t landmark_i(0); landmark_i < _header_3d_static_landmarks;
           landmark_i++) {
        fstream_3d_static_landmarks >>
            _xposition_3d_static_landmarks(landmark_i) >>
            _yposition_3d_static_landmarks(landmark_i);
      }

    } else {
      std::cout << "\n\nF A I L E D  READING STATIC_LANDMARKS!\n\n\n";
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
  };

  ~Framesimulator() {
    _fstream_camera_poses->close();
    _fstream_3d_dynamic_landmarks->close();
  }

  void step_GetImagePoints();

 private:
  // filepaths
  const std::string _file_camera_poses;
  const std::string _file_3d_static_landmarks;
  const std::string _file_3d_dynamic_landmarks;

  // fstreams
  std::unique_ptr<std::ifstream> _fstream_camera_poses;
  std::unique_ptr<std::ifstream> _fstream_3d_dynamic_landmarks;

  // number of frames
  size_t _header_camera_poses;
  // number of frames
  size_t _header_3d_static_landmarks;
  // number of frames, number of landmarks
  std::tuple<size_t, size_t> _header_3d_dynamic_landmarks;

  // position data
  array_t _xposition_3d_static_landmarks;
  array_t _yposition_3d_static_landmarks;
};
}  // namespace sfmsimulator::framesimulator

#endif /* end of include guard: __FRAMESIMULATOR_HH__ */
