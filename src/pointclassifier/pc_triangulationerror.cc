#include "pc_triangulationerror.hh"

namespace sfmsimulator::pointclassifier {

void PC_Triangulationerror::classifynext(const Sfmreconstruction &reconstruct,
                                         array_t &weights) const {
  array_t new_weights = reconstruct.reprojection_error;

  // set tolerance
  constexpr precision_t reproject_error_tolerance(0.01);
  constexpr precision_t reproject_error_max(5.0);
  constexpr precision_t expweightdist(0.5);

  for (size_t i(0); i < new_weights.size(); ++i) {
    new_weights(i) = new_weights(i) == 0 ? reproject_error_max : new_weights(i);
    new_weights(i) =
        new_weights(i) < reproject_error_tolerance ? 0 : new_weights(i);
    new_weights(i) = new_weights(i) > reproject_error_max ? reproject_error_max
                                                          : new_weights(i);
  }

  precision_t max = reproject_error_max;
  new_weights /= max;

  for (size_t i(0); i < new_weights.size(); ++i) {
    new_weights(i) = std::pow(expweightdist, new_weights(i));
  }

  for (size_t i(0); i < new_weights.size(); ++i) {
    new_weights(i) =
        new_weights(i) < reproject_error_tolerance ? 0 : new_weights(i);
  }

  new_weights = 1.0 - (new_weights - 1.0) / (expweightdist - 1.0);

  // weights = new_weights;
  // return;

  // dependency on old weights

  const array_t diff = new_weights - weights;
  // weights = weights * diff + new_weights; // dep1
  weights = new_weights * diff + weights; // dep2
  const precision_t resultmax = weights.maxCoeff();
  weights /= resultmax;
}

void PC_Triangulationerror::cluster(const points::Points2d image_points,
                                    const std::vector<bool> type) const {
  // TODO(dave): spherical histogram to determine movements
  // TODO(dave): mean-shift clustering
  // TODO(dave): compute complex hull of clusters
}
} // namespace sfmsimulator::pointclassifier
