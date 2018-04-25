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
      << "author: david schmidig [david@davencyw.net]\n"
      << "        davencyw code  [davencyw.net]\n"
      << "        ETH Zurich\n\n"
      << "_____________________________________________________\n\n\n";
  ;
}

void Sfmsimulator::run() {
  const size_t steps(_framesimulator.updatesLeft());
  // TODO(dave): check initial step, which does not need an update
  doSteps(steps);
}

void Sfmsimulator::doSteps(const size_t steps) {
  for (size_t step_i(0); step_i < steps; step_i++) {
    step();
  }
}

void Sfmsimulator::step() {
  std::cout << " - STEP[ " << _step << " ]\n";

  const points::Points2d image_points = _framesimulator.step_GetImagePoints();
  //*DEBUG*/ std::cout << image_points.coord[0];
  //*DEBUG*/ std::cout << "\n";
  //*DEBUG*/ std::cout << image_points.coord[1];

  // TODO(dave): implement: reconstruct - classify - reconstruct

  ++_step;
}

// TODO(dave): figure out return type and what to return.
void Sfmsimulator::reconstruct(
    std::shared_ptr<points::Points2d> points_frame1,
    std::shared_ptr<points::Points2d> points_frame2) {
  std::vector<cv::Mat> points_collapsed;

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

  cv::Matx33d K = _cameramodel.getK_ocv();

  std::vector<cv::Mat> Rs_est, ts_est, points3d_estimated;
  bool is_projective(true);
  cv::sfm::reconstruct(points_collapsed, Rs_est, ts_est, points3d_estimated,
                       is_projective);
}

} // namespace sfmsimulator
