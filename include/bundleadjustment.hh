#ifndef __BUNDLEADJUSTMENT_HH__
#define __BUNDLEADJUSTMENT_HH__

#include "bundleadjustment.hh"
#include "cameramodel.hh"
#include "global.hh"
#include "points.hh"
#include "sfmsimulator.hh"

#include <algorithm>
#include <iostream>

#include <ceres/ceres.h>
#include <ceres/rotation.h>

namespace sfmsimulator::bundleadjustment {

struct SimpleReprojectionError {
  SimpleReprojectionError(precision_t observed_x, precision_t observed_y,
                          precision_t weight, precision_t focal)
      : observed_x(observed_x), observed_y(observed_y), weight(weight),
        focal(focal) {}
  template <typename T>
  bool operator()(const T *const camera, const T *const point,
                  T *residuals) const {
    T p[3];
    T angleaxis[3];
    T eulerangles[3];
    T rotation[9];

    p[0] = point[0];
    p[1] = point[1];
    p[2] = point[2];

    // Rotate: camera[0,1,2] are the angle-axis rotation.
    // ceres::EulerAnglesToRotationMatrix(eulerangles, 1, rotation);
    // ceres::RotationMatrixToAngleAxis(rotation, angleaxis);
    // ceres::AngleAxisRotatePoint(camera, point, p);

    // Translate: camera[3,4,5] are the translation.
    p[0] -= camera[3];
    p[1] -= camera[4];
    p[2] -= camera[5];

    // Perspective divide
    const T xp = p[0] / p[2];
    const T yp = p[1] / p[2];

    // Compute final projected point position.
    const T predicted_x = T(focal) * xp;
    const T predicted_y = T(focal) * yp;

    // The error is the difference between the predicted and observed position.
    residuals[0] = (predicted_x - T(observed_x)) * T(weight);
    residuals[1] = (predicted_y - T(observed_y)) * T(weight);
    return true;
  }
  // Factory to hide the construction of the CostFunction object from
  // the client code.
  static ceres::CostFunction *Create(const precision_t observed_x,
                                     const precision_t observed_y,
                                     const precision_t weight,
                                     const precision_t focal) {
    return (new ceres::AutoDiffCostFunction<SimpleReprojectionError, 2, 6, 3>(
        new SimpleReprojectionError(observed_x, observed_y, weight, focal)));
  }
  precision_t observed_x;
  precision_t observed_y;
  precision_t weight;
  precision_t focal;
};

Sfmreconstruction adjustBundle(
    const std::vector<std::shared_ptr<points::Points2d>> &points_frames,
    const std::shared_ptr<points::Points3d> &points_world,
    const std::vector<vec6_t> &cameraposes,
    const cameramodel::Cameramodel &camera, const array_t &weights,
    const bool set_first_camera_const) {

  // Create residuals for each observation in the bundle adjustment problem.
  // The
  // parameters for cameras and points are added automatically.
  ceres::Problem problem;

  const size_t numframes(points_frames.size());
  const mat33_t intrinsics(camera.getK_eigen());
  precision_t focal = intrinsics(0, 0);

  size_t numpoints = points_frames[0]->numpoints;
  std::vector<vec3_t> mutable_points3d(numpoints);
  std::vector<vec6_t> mutable_cameraposes(cameraposes);

  const std::array<array_t, 3> *coord_world(&(points_world->coord));
  std::vector<ceres::ResidualBlockId> residual_block_ids;

  const precision_t cx(intrinsics(0, 2));
  const precision_t cy(intrinsics(1, 2));

  size_t skipped(0);

  // TODO(dave): change loop order
  for (size_t point_i(0); point_i < numpoints; ++point_i) {
    mutable_points3d[point_i] =
        vec3_t((*coord_world)[0](point_i), (*coord_world)[1](point_i),
               (*coord_world)[2](point_i));

    size_t framecounter(0);
    for (auto &points_frame_i : points_frames) {

      if (!(points_frame_i->visible[point_i])) {
        ++skipped;
        continue;
      }
      const precision_t uvx(points_frame_i->coord[0](point_i));
      const precision_t uvy(points_frame_i->coord[1](point_i));

      // std::cout << "RBLOCK:\n"
      //           << uvx << " : " << uvy << "\t||||\t"
      //           << mutable_points3d[point_i][0] << " : "
      //           << mutable_points3d[point_i][1] << " : "
      //           << mutable_points3d[point_i][2] << "\tc: " << framecounter
      //           << "\n"
      //           << mutable_cameraposes[framecounter] << "\n";

      ceres::CostFunction *cost_function = SimpleReprojectionError::Create(
          uvx - cx, uvy - cy, weights(point_i), focal);

      ceres::ResidualBlockId blockid = problem.AddResidualBlock(
          cost_function, nullptr, mutable_cameraposes[framecounter].data(),
          mutable_points3d[point_i].data());
      residual_block_ids.push_back(blockid);
      ++framecounter;
    }
  }

  if (set_first_camera_const) {
    // translation is fixed in first camera
    const std::vector<int> const_indices = {3, 4, 5};
    ceres::SubsetParameterization *subset_parameterization =
        new ceres::SubsetParameterization(6, const_indices);
    problem.SetParameterization(mutable_cameraposes[0].data(),
                                subset_parameterization);
  }

  if (skipped) {
    std::cout << "\nskipped " << skipped << " points\n";
  }
  // Make Ceres automatically detect the bundle structure.
  ceres::Solver::Options options;
  options.linear_solver_type = ceres::ITERATIVE_SCHUR;
  options.minimizer_progress_to_stdout = true;
  options.max_num_iterations = 1024;
  options.logging_type = ceres::LoggingType::SILENT;
  ceres::Solver::Summary summary;
  ceres::Solve(options, &problem, &summary);
  // std::cout << summary.BriefReport() << "\n";

  ceres::Problem::EvaluateOptions evaloptions;
  evaloptions.residual_blocks = residual_block_ids;
  double total_cost = 0.0;
  std::vector<precision_t> residuals;
  problem.Evaluate(evaloptions, &total_cost, &residuals, nullptr, nullptr);

  Sfmreconstruction reconstruct;
  array_t eigen_residuals =
      Eigen::Map<array_t>(residuals.data(), residuals.size());
  array_t error = array_t::Zero(numpoints);
  array_t visible_in_num_frames = array_t::Ones(numpoints);

  // collapse residuals
  size_t index(0);
  for (size_t point_i(0); point_i < numpoints; ++point_i) {
    size_t visible_in_num_frames_local(0);
    for (size_t frame_i(0); frame_i < numframes; ++frame_i) {
      // if point visible in frame then there is a residual
      if (points_frames[frame_i]->visible[point_i]) {
        visible_in_num_frames_local++;
        const precision_t local_residual_0(eigen_residuals(index++) /
                                           (weights(point_i) + 0.00000001));
        const precision_t local_residual_1(eigen_residuals(index++) /
                                           (weights(point_i) + 0.00000001));

        error(point_i) += local_residual_0 * local_residual_0 +
                          local_residual_1 * local_residual_1;
      }
    }

    visible_in_num_frames[point_i] = std::clamp(
        visible_in_num_frames_local, static_cast<size_t>(1), numframes);
  }

  error = error.cwiseQuotient(visible_in_num_frames);
  reconstruct.reprojection_error = error;

  if (not(summary.termination_type == ceres::CONVERGENCE)) {
    std::cerr << "Bundle adjustment failed." << std::endl;
    Sfmreconstruction empty;
    return empty;
  }

  /*DEBUG/
  for (auto &c : mutable_cameraposes) {
    std::cout << "\n" << c << "\n";
  }*/

  /*DEBUG/
  for (auto &p : mutable_points3d) {
    std::cout << "\n" << p << "\n";
  }*/

  std::shared_ptr<points::Points3d> world_estimate =
      std::make_shared<points::Points3d>(points::Points3d(numpoints));
  for (size_t point_i(0); point_i < numpoints; ++point_i) {
    world_estimate->coord[0](point_i) = mutable_points3d[point_i](0);
    world_estimate->coord[1](point_i) = mutable_points3d[point_i](1);
    world_estimate->coord[2](point_i) = mutable_points3d[point_i](2);
  }

  reconstruct.point3d_estimate = world_estimate;

  reconstruct.camerapose_estimate.insert(
      std::end(reconstruct.camerapose_estimate),
      std::begin(mutable_cameraposes), std::end(mutable_cameraposes));

  return reconstruct;
}

} /* sfmsimulator::bundleadjustment */

#endif /* end of include guard: __BUNDLEADJUSTMENT_HH__ */
