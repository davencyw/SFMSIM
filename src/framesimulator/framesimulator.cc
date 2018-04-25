#include "framesimulator.hh"

#include "cameramodel.hh"
#include "geometry.hh"

#include <opencv2/core/eigen.hpp>

namespace sfmsimulator::framesimulator {

Framesimulator::Framesimulator(const std::string file_camera_poses,
                               const std::string file_3d_static_landmarks,
                               const std::string file_3d_dynamic_landmarks,
                               const cameramodel::Cameramodel cameramodel)
    : _file_camera_poses(file_camera_poses),
      _file_3d_static_landmarks(file_3d_static_landmarks),
      _file_3d_dynamic_landmarks(file_3d_dynamic_landmarks),
      _imageplane(cameramodel.getImageplane()), _K_ocv(cameramodel.getK_ocv()),
      _K_eigen(cameramodel.getK_eigen()) {
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

    array_t *xposition_3d_all(&(_scene_3d_points.coord[0]));
    array_t *yposition_3d_all(&(_scene_3d_points.coord[1]));
    array_t *zposition_3d_all(&(_scene_3d_points.coord[2]));

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
  array_t *xposition_3d_all(&(_scene_3d_points.coord[0]));
  array_t *yposition_3d_all(&(_scene_3d_points.coord[1]));
  array_t *zposition_3d_all(&(_scene_3d_points.coord[2]));

  for (size_t dynamic_point_i(0); dynamic_point_i < num_dynamic_points;
       dynamic_point_i++) {
    *_fstream_3d_dynamic_landmarks >> (*xposition_3d_all)(dynamic_point_i) >>
        (*yposition_3d_all)(dynamic_point_i) >>
        (*zposition_3d_all)(dynamic_point_i);
  }

  ++_steps;
}

mat44_t Framesimulator::updateCameraPose() {

  precision_t x_rotation(0.0), y_rotation(0.0), z_rotation(0.0);
  vec4_t translation = vec4_t::Ones();

  *_fstream_camera_poses >> translation(0) >> translation(1) >>
      translation(2) >> x_rotation >> y_rotation >> z_rotation;

  // euler angles
  mat33_t rotation;
  rotation =
      Eigen::AngleAxis<precision_t>(x_rotation * M_PI,
                                    Eigen::Matrix<precision_t, 3, 1>::UnitX()) *
      Eigen::AngleAxis<precision_t>(y_rotation * M_PI,
                                    Eigen::Matrix<precision_t, 3, 1>::UnitY()) *
      Eigen::AngleAxis<precision_t>(z_rotation * M_PI,
                                    Eigen::Matrix<precision_t, 3, 1>::UnitZ());

  mat44_t transformation = mat44_t::Zero();
  transformation.block<3, 3>(0, 0) = rotation;
  transformation.col(3) = translation;

  return transformation;
}

points::Points2d Framesimulator::step_GetImagePoints() {
  // read next poses of dynamic landmarks
  update3dScenePoints();

  // next poses of camera
  const mat44_t world_to_camera(updateCameraPose());
  const mat44_t camera_to_world(world_to_camera.inverse());

  const size_t image_width(_imageplane.width);
  const size_t image_height(_imageplane.height);

  const size_t numpoints(_scene_3d_points.numpoints);
  points::Points2d projected(numpoints);

  // TODO(dave) rewrite matrix-vector^n computation efficiently without creating
  // vectors!
  for (size_t point_i(0); point_i < numpoints; point_i++) {
    vec4_t point_vec4(vec4_t::Ones());
    point_vec4(0) = _scene_3d_points.coord[0](point_i);
    point_vec4(1) = _scene_3d_points.coord[1](point_i);
    point_vec4(2) = _scene_3d_points.coord[2](point_i);

    // project onto camera
    vec4_t lm = camera_to_world * point_vec4;
    const vec3_t pt_h = _K_eigen * Eigen::Map<vec3_t>(lm.data(), 3);

    // get image coordinates and write
    const precision_t x_image_coord_local = pt_h(0) / pt_h(2);
    const precision_t y_image_coord_local = pt_h(1) / pt_h(2);

    // TODO(dave)handle unobservable point ids!
    // TODO(dave): check if  observable
    // TODO(dave): do this polygon check outside of the loop for efficiency
    // TODO(dave): work with projected.coord pointer instead of bracket ops
    bool is_in_image(geometry::isInside2dPolygon(
        x_image_coord_local, y_image_coord_local, image_height, image_width));
    if (is_in_image) {
      projected.coord[0](point_i) = x_image_coord_local;
      projected.coord[1](point_i) = y_image_coord_local;
    }
  }

  return projected;
}

} // namespace sfmsimulator::framesimulator
