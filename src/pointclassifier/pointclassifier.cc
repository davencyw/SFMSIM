#include "pointclassifier.hh"

namespace sfmsimulator::pointclassifier {

void setInvisibleToOldWeights(const array_t &old_weights, array_t &new_weights,
                              std::shared_ptr<points::Points2d> points) {

  for (size_t point_i(0); point_i < points->numpoints; ++point_i) {
    if (!(points->visible[point_i])) {
      new_weights[point_i] = old_weights[point_i];
    }
  }
}
} // end namespace sfmsimulator::pointclassifier
