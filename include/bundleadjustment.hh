#ifndef __BUNDLEADJUSTMENT_HH__
#define __BUNDLEADJUSTMENT_HH__

#include "bundleadjustment.hh"
#include "cameramodel.hh"
#include "global.hh"
#include "points.hh"
#include "sfmsimulator.hh"

#include <iostream>

#include <ceres/ceres.h>
#include <ceres/rotation.h>

namespace sfmsimulator::bundleadjustment {

struct SimpleReprojectionError {
  SimpleReprojectionError(precision_t observed_x, precision_t observed_y,
                          precision_t weight)
      : observed_x(observed_x), observed_y(observed_y), weight(weight) {}
  template <typename T>
  bool operator()(const T *const camera, const T *const point,
                  const T *const focal, T *residuals) const {
    T p[3];
    T angleaxis[3];
    T eulerangles[3];
    T rotation[9];

    // Rotate: camera[0,1,2] are the angle-axis rotation.
    ceres::EulerAnglesToRotationMatrix(eulerangles, 1, rotation);
    ceres::RotationMatrixToAngleAxis(rotation, angleaxis);
    ceres::AngleAxisRotatePoint(camera, point, p);

    // Translate: camera[3,4,5] are the translation.
    p[0] -= camera[3];
    p[1] -= camera[4];
    p[2] -= camera[5];

    // Perspective divide
    const T xp = p[0] / p[2];
    const T yp = p[1] / p[2];

    // Compute final projected point position.
    const T predicted_x = *focal * xp;
    const T predicted_y = *focal * yp;

    // The error is the difference between the predicted and observed position.
    residuals[0] = (predicted_x - T(observed_x)) * weight;
    residuals[1] = (predicted_y - T(observed_y)) * weight;
    return true;
  }
  // Factory to hide the construction of the CostFunction object from
  // the client code.
  static ceres::CostFunction *Create(const precision_t observed_x,
                                     const precision_t observed_y,
                                     const precision_t weight) {
    return (
        new ceres::AutoDiffCostFunction<SimpleReprojectionError, 2, 6, 3, 1>(
            new SimpleReprojectionError(observed_x, observed_y, weight)));
  }
  precision_t observed_x;
  precision_t observed_y;
  precision_t weight;
};

Sfmreconstruction adjustBundle(
    const std::vector<std::shared_ptr<points::Points2d>> &points_frames,
    const std::shared_ptr<points::Points3d> &points_world,
    const std::vector<vec6_t> &cameraposes,
    const cameramodel::Cameramodel &camera, const array_t &weights) {

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

  for (size_t point_i(0); point_i < numpoints; ++point_i) {
    mutable_points3d[point_i] =
        vec3_t((*coord_world)[0](point_i), (*coord_world)[1](point_i),
               (*coord_world)[2](point_i));

    size_t framecounter(0);
    for (auto &points_frame_i : points_frames) {

      // TODO(dave): check if cx,cy have to be subtracted
      const precision_t uvx(points_frame_i->coord[0](point_i));
      const precision_t uvy(points_frame_i->coord[1](point_i));

      if (uvx < 0 && uvy < 0) {
        std::cout << "\nSKIPPED\n";
        continue;
      }

      ceres::CostFunction *cost_function =
          SimpleReprojectionError::Create(uvx - cx, uvy - cy, weights(point_i));

      ceres::ResidualBlockId blockid = problem.AddResidualBlock(
          cost_function, nullptr, mutable_cameraposes[framecounter].data(),
          mutable_points3d[point_i].data(), &focal);
      residual_block_ids.push_back(blockid);
      ++framecounter;
    }
  }

  // Make Ceres automatically detect the bundle structure.
  ceres::Solver::Options options;
  options.linear_solver_type = ceres::DENSE_SCHUR;
  options.minimizer_progress_to_stdout = true;
  options.max_num_iterations = 500;
  options.eta = 1e-2;
  options.max_solver_time_in_seconds = 10;
  options.logging_type = ceres::LoggingType::SILENT;
  ceres::Solver::Summary summary;
  ceres::Solve(options, &problem, &summary);
  std::cout << summary.BriefReport() << "\n";

  ceres::Problem::EvaluateOptions evaloptions;
  evaloptions.residual_blocks = residual_block_ids;
  double total_cost = 0.0;
  std::vector<precision_t> residuals;
  problem.Evaluate(evaloptions, &total_cost, &residuals, nullptr, nullptr);

  Sfmreconstruction reconstruct;
  array_t temp_error = Eigen::Map<array_t>(residuals.data(), residuals.size());
  array_t error = array_t::Zero(numpoints);

  // collapse residuals
  for (size_t residual_i(0); residual_i < temp_error.size() / numframes;
       residual_i += 2) {
    size_t index(residual_i);
    for (size_t residual_j(0); residual_j < numframes; ++residual_j) {
      index += numpoints;
      error(residual_i / 2) += temp_error(index) * temp_error(index) +
                               temp_error(index + 1) * temp_error(index + 1);
    }
  }

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

  for (auto &camerapose_i : mutable_cameraposes) {
    std::shared_ptr<mat44_t> pose = std::make_shared<mat44_t>(mat44_t());
    precision_t rotationmat[9] = {0};
    ceres::AngleAxisToRotationMatrix(camerapose_i.data(), rotationmat);

    for (int r(0); r < 3; r++) {
      for (int c(0); c < 3; c++) {
        (*pose)(c, r) = rotationmat[r * 3 + c]; //`rotationMat` is
                                                //      col-major...
      }
    }
    // translation
    (*pose)(0, 3) = camerapose_i[3];
    (*pose)(1, 3) = camerapose_i[4];
    (*pose)(2, 3) = camerapose_i[5];

    reconstruct.camerapose_estimate_mat.push_back(pose);
    reconstruct.camerapose_estimate.push_back(camerapose_i);
  }
  return reconstruct;
}

} /* sfmsimulator::bundleadjustment */

#endif /* end of include guard: __BUNDLEADJUSTMENT_HH__ */
