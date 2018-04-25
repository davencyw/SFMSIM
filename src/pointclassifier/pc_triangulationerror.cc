#include "pc_triangulationerror.hh"

namespace sfmsimulator::pointclassifier {

const std::vector<bool> PC_Triangulationerror::classify(
    const std::shared_ptr<points::Points2d> image_points_frame_1,
    const std::shared_ptr<points::Points2d> image_points_frame_2,
    const std::shared_ptr<points::Points3d> world_points_frame_1,
    const std::shared_ptr<points::Points3d> world_points_frame_2) const {
  size_t numpoints(image_points_frame_1->coord[0].size());
  std::vector<bool> types(numpoints);

  const std::array<array_t, 3> *const world_coord_frame_1(
      &(world_points_frame_1->coord));
  //
  const std::array<array_t, 2> *const image_coord_frame_2(
      &(image_points_frame_2->coord));

  // TODO(dave): transform and reproject
  std::array<array_t, 2> image_coord_frame_2_projected_from_1;
  std::array<array_t, 2> reprojection_error;

  reprojection_error[0] =
      ((*image_coord_frame_2)[0] - image_coord_frame_2_projected_from_1[0])
          .square();
  reprojection_error[1] =
      ((*image_coord_frame_2)[1] - image_coord_frame_2_projected_from_1[1])
          .square();
  array_t total_reprojection_error(reprojection_error[0] +
                                   reprojection_error[1]);

  // TODO(dave): reprojection error:
  // TODO(dave): classification rule
}

void PC_Triangulationerror::cluster(const points::Points2d image_points,
                                    const std::vector<bool> type) const {
  // TODO(dave): spherical histogram to determine movements
  // TODO(dave): mean-shift clustering
  // TODO(dave): compute complex hull of clusters
}
} // namespace sfmsimulator::pointclassifier
