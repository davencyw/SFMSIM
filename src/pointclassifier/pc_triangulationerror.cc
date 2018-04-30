#include "pc_triangulationerror.hh"

namespace sfmsimulator::pointclassifier {

const array_t
PC_Triangulationerror::classifynext(Sfmreconstruction reconstruct) const {
  // TODO(dave): extract reprojection_error and weight them

  array_t weights = reconstruct.reprojection_error;

  // TODO(dave): classification rule
  return weights;
}

void PC_Triangulationerror::cluster(const points::Points2d image_points,
                                    const std::vector<bool> type) const {
  // TODO(dave): spherical histogram to determine movements
  // TODO(dave): mean-shift clustering
  // TODO(dave): compute complex hull of clusters
}
} // namespace sfmsimulator::pointclassifier
