#include "pc_triangulationerror.hh"

namespace sfmsimulator::pointclassifier {

const std::vector<bool> PC_Triangulationerror::classify(
    const std::shared_ptr<points::Points2d> image_points_frame_1,
    const std::shared_ptr<points::Points2d> image_points_frame_2,
    const std::shared_ptr<points::Points3d> world_points_frame_1,
    const std::shared_ptr<points::Points3d> world_points_frame_2) const {
  size_t numpoints(image_points_frame_1->coord[0].size());
  std::vector<bool> types(numpoints);

  const std::array<array_t, 3>* const world_coord_frame_1(
      &(world_points_frame_1->coord));
  const std::array<array_t, 3>* const world_coord_frame_2(
      &(world_points_frame_2->coord));

  const array_t xdisplacement((*world_coord_frame_1)[0] -
                              (*world_coord_frame_2)[0]);
  const array_t ydisplacement((*world_coord_frame_1)[1] -
                              (*world_coord_frame_2)[1]);
  const array_t zdisplacement((*world_coord_frame_1)[2] -
                              (*world_coord_frame_2)[2]);

  const array_t absolutdisplacement(
      xdisplacement.square() + ydisplacement.square() + zdisplacement.square());
}

void PC_Triangulationerror::cluster(const points::Points2d image_points,
                                    const std::vector<bool> type) const {}

}  // namespace sfmsimulator::pointclassifier
