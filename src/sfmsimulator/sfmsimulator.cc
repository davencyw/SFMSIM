#include "sfmsimulator.hh"

#include <opencv2/core/core.hpp>
//#include <opencv2/sfm.hpp>

#include <cassert>

namespace sfmsimulator {

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

  array_t* xcoord_frame_1 = &(points_frame1->coord[0]);
  array_t* xcoord_frame_2 = &(points_frame2->coord[0]);
  array_t* ycoord_frame_1 = &(points_frame1->coord[1]);
  array_t* ycoord_frame_2 = &(points_frame2->coord[1]);

  for (size_t point_i(0); point_i < num_points_frame1; ++point_i) {
    frame1(0, point_i) = (*xcoord_frame_1)(point_i);
    frame1(1, point_i) = (*ycoord_frame_1)(point_i);
    frame2(0, point_i) = (*xcoord_frame_2)(point_i);
    frame2(1, point_i) = (*ycoord_frame_2)(point_i);
  }

  points_collapsed.push_back(frame1);
  points_collapsed.push_back(frame2);

  cv::Matx33d K = _cameramodel.getK();

  std::vector<cv::Mat> Rs_est, ts_est, points3d_estimated;
  bool is_projective(true);
  // TODO(dave): install opencv::sfm
  // cv::sfm::reconstruct(points_collapsed, Rs_est, ts_est,
  // points3points3d_estimated, is_projective);
}

}  // namespace sfmsimulator
