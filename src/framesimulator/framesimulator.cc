#include "framesimulator.hh"

#include "cameramodel.hh"

namespace sfmsimulator::framesimulator {

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
