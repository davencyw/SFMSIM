#include "pc_triangulationerror.hh"
#include "sfmsimulator.hh"

#define CERES_FOUND 1
#define CV_OVERRIDE override

#include <opencv2/core/core.hpp>
#include <opencv2/sfm.hpp>

#include <cassert>

namespace sfmsimulator {

Sfmsimulator::Sfmsimulator(Sfmconfig config)
    : _config(config), _cameramodel(config.cameramodel),
      _K(config.cameramodel.getK_ocv()),
      _framesimulator(framesimulator::Framesimulator(config.filepaths,
                                                     config.cameramodel)) {
  using pct = pointclassifier::Pointclassifier_type;

  switch (config.type_pointclassifier) {
  case pct::PC_Triangulationerror_t:
    _pointclassifier = std::make_unique<pointclassifier::Pointclassifier>(
        pointclassifier::PC_Triangulationerror(_cameramodel));
    break;
  default:
    break;
  }

  std::cout
      << "\033[0m\n"
      << "\n\n\n\n"
      << "            __\n"
      << "     ___   / _|  _ __ ___      \n"
      << "    / __| | |_  | '_ ` _ \\     \n"
      << "    \\__ \\ |  _| | | | | | |     \n"
      << "    |___/ |_|   |_| |_| |_|   \n\n"
      << "           _                       _           _\n"
      << "     ___  (_)  _ __ ___    _   _  | |   __ _  | |_    ___    _ __ "
      << "\n    / __| | | | '_ ` _ \\  | | | | | |  / _` | | __|  / _ \\  | "
      << "'__|\n"
      << "    \\__ \\ | | | | | | | | | |_| | | | | (_| | | |_  | (_) | | |  "
      << " \n"
      << "    |___/ |_| |_| |_| |_|  \\__,_| |_|  \\__,_|  \\__|  \\___/  "
      << "|_| \n\n\n\n"
      << "    author: david schmidig [david@davencyw.net]\n"
      << "            davencyw code  [davencyw.net]\n"
      << "            ETH Zurich\n\n"
      << "_____________________________________________________________________"
         "\n\n\n";

  _scene_camera_pose.setIdentity();
}

void Sfmsimulator::run() {
  const size_t steps(_framesimulator.updatesLeft());
  doSteps(steps);
}

void Sfmsimulator::doSteps(const size_t steps) {
  for (size_t step_i(0); step_i < steps; step_i++) {
    step();
  }
}

// TODO(dave) add generic depending sliding window to pc_classifier
// TODO(dave) separate initializationsteps (_steps=0,1) from this method
void Sfmsimulator::step() {
  std::cout << " - STEP[ " << _step << " ]\n";

  _scene_window_image.push_front(std::make_shared<points::Points2d>(
      _framesimulator.step_GetImagePoints()));

  // initialization step
  if (_scene_window_image.size() < 2) {
    assert(_step == 0);
    ++_step;
    return;
  }

  assert(_scene_window_world.size() == 2);
  // newer frame is older (frame2.time > frame1.time) but is placed in the front
  // of the deque!
  std::shared_ptr<points::Points3d> points_world2;
  Sfmreconstruction reconstruction(
      reconstruct(_scene_window_image[1], _scene_window_image[0]));
  _scene_window_world.push_front(reconstruction.point3d_estimate);

  _scene_camera_pose = _scene_camera_pose * (*reconstruction.transformation);
  _scene_camera_transforms.push_front(reconstruction.transformation);

  // needs two 3d pointestimate for initialization
  if (_scene_window_world.size() < 2) {
    assert(_step == 1);
    ++_step;
    return;
  }

  if (_pointclassifier) {
    // classify and reconstruct with only static points
    _pointclassifier->classify(_scene_window_image[1], _scene_window_image[0],
                               _scene_window_world[1], _scene_window_world[0]);
  }

  // TODO(dave): reconstruct again with classified features!

  _scene_window_world.pop_back();
  _scene_window_image.pop_back();
  ++_step;
}

Sfmreconstruction
Sfmsimulator::reconstruct(std::shared_ptr<points::Points2d> points_frame1,
                          std::shared_ptr<points::Points2d> points_frame2) {
  std::vector<cv::Mat> points_collapsed;
  Sfmreconstruction reconstruction;

  // cast SOA-pointdata to the reconstruct sfm api of opencv
  const size_t num_points_frame1(points_frame1->coord[0].size());
  const size_t num_points_frame2(points_frame2->coord[0].size());
  assert(num_points_frame1 == num_points_frame2);

  cv::Mat_<double> frame1(2, num_points_frame1);
  cv::Mat_<double> frame2(2, num_points_frame1);

  array_t *xcoord_frame_1 = &(points_frame1->coord[0]);
  array_t *xcoord_frame_2 = &(points_frame2->coord[0]);
  array_t *ycoord_frame_1 = &(points_frame1->coord[1]);
  array_t *ycoord_frame_2 = &(points_frame2->coord[1]);

  for (size_t point_i(0); point_i < num_points_frame1; ++point_i) {
    frame1(0, point_i) = (*xcoord_frame_1)(point_i);
    frame1(1, point_i) = (*ycoord_frame_1)(point_i);
    frame2(0, point_i) = (*xcoord_frame_2)(point_i);
    frame2(1, point_i) = (*ycoord_frame_2)(point_i);
  }

  points_collapsed.push_back(frame1);
  points_collapsed.push_back(frame2);

  std::vector<cv::Mat> Rs_est, ts_est, points3d_estimated;
  bool is_projective(true);
  cv::sfm::reconstruct(points_collapsed, Rs_est, ts_est, _K, points3d_estimated,
                       is_projective);

  reconstruction.transformation;
  reconstruction.point3point3d_estimate;
}

} // namespace sfmsimulator
