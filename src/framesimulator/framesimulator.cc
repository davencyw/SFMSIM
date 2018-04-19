#include "framesimulator.hh"

#include "cameramodel.hh"

namespace sfmsimulator::framesimulator {

Framesimulator::Framesimulator(const std::string file_camera_poses,
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
    std::cout << "\n\nF A I L E D  READING CAMERA_POSES!\n"
              << _file_camera_poses << "\n\n";
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

void Framesimulator::update3dScenePoints() {
  // read and update dynamic points from next timestep
  size_t num_dynamic_points(std::get<1>(_header_3d_dynamic_landmarks));
  const size_t num_all_landmarks(_header_3d_static_landmarks +
                                 num_dynamic_points);
  array_t* xposition_3d_all(&(_scene_3d_points.coord[0]));
  array_t* yposition_3d_all(&(_scene_3d_points.coord[1]));
  array_t* zposition_3d_all(&(_scene_3d_points.coord[2]));

  for (size_t dynamic_point_i(0); dynamic_point_i < num_dynamic_points;
       dynamic_point_i++) {
    *_fstream_3d_dynamic_landmarks >> (*xposition_3d_all)(dynamic_point_i) >>
        (*yposition_3d_all)(dynamic_point_i) >>
        (*zposition_3d_all)(dynamic_point_i);
  }

  ++_steps;
}

void Framesimulator::updateCameraPose() {}

points::Points2d Framesimulator::step_GetImagePoints() {
  // read next poses of dynamic landmarks
  update3dScenePoints();

  // TODO(dave): next poses of camera

  // TODO(dave):project all landmarks at current 3d poses onto imageplane
  points::Points2d projected;

  return projected;
}

}  // namespace sfmsimulator::framesimulator
