#include "pc_triangulationerror.hh"

namespace sfmsimulator::pointclassifier {

void PC_Triangulationerror::classify(points::Points2d image_points_frame_1,
                                     points::Points2d image_points_frame_2,
                                     points::Points3d world_points_frame_1,
                                     points::Points3d world_points_frame_2) {}
void PC_Triangulationerror::cluster(points::Points2d image_points,
                                    std::vector<bool> type) {}

}  // namespace sfmsimulator::pointclassifier
