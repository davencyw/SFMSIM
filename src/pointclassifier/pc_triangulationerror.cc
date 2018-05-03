#include "pc_triangulationerror.hh"

namespace sfmsimulator::pointclassifier {

const array_t
PC_Triangulationerror::classifynext(Sfmreconstruction reconstruct) const {
  array_t weights = reconstruct.reprojection_error;
  precision_t max = weights.maxCoeff();
  precision_t expweightdist(0.1);
  weights /= max;
  weights = 1.0 - (weights.pow(expweightdist) - 1.0) / (expweightdist - 1.0);
  return weights;
}

void PC_Triangulationerror::cluster(const points::Points2d image_points,
                                    const std::vector<bool> type) const {
  // TODO(dave): spherical histogram to determine movements
  // TODO(dave): mean-shift clustering
  // TODO(dave): compute complex hull of clusters
}
} // namespace sfmsimulator::pointclassifier
