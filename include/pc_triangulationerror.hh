#ifndef __PC_TRIANGULATIONERROR_HH__
#define __PC_TRIANGULATIONERROR_HH__

#include "pointclassifier.hh"

#include <iostream>

namespace sfmsimulator::pointclassifier {

class PC_Triangulationerror : public Pointclassifier {
public:
  PC_Triangulationerror(cameramodel::Cameramodel camera)
      : Pointclassifier(camera) {}

  void classifynext(const Sfmreconstruction &reconstruct,
                    array_t &weights) const override {
    array_t new_weights = reconstruct.reprojection_error;

    // set tolerance
    constexpr precision_t reproject_error_tolerance(0.01);
    constexpr precision_t reproject_error_max(5.0);
    constexpr precision_t expweightdist(0.5);

    for (size_t i(0); i < new_weights.size(); ++i) {
      new_weights(i) =
          new_weights(i) == 0 ? reproject_error_max : new_weights(i);
      new_weights(i) =
          new_weights(i) < reproject_error_tolerance ? 0 : new_weights(i);
      new_weights(i) = new_weights(i) > reproject_error_max
                           ? reproject_error_max
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
    //
    // weights = new_weights;
    // return;

    // dependency on old weights

    const array_t diff = new_weights - weights;
    weights = weights * diff + new_weights; // dep1
    // weights = new_weights * diff + weights; // dep2
    const precision_t resultmax = weights.maxCoeff();
    weights /= resultmax;
  }

private:
};

} // namespace sfmsimulator::pointclassifier

#endif /* end of include guard: __PC_TRIANGULATIONERROR_HH__ */
