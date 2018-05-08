#ifndef __PC_TRIANGULATIONERROR_HH__
#define __PC_TRIANGULATIONERROR_HH__

#include "pointclassifier.hh"

#include <iostream>
#include <string>

namespace sfmsimulator::pointclassifier {

inline void ExponentialWeighting(const precision_t reproject_error_tolerance,
                                 const precision_t reproject_error_max,
                                 const precision_t expweightdist,
                                 array_t &new_weights) {
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
}

struct PC_ReprojectionErrorNodep : public Pointclassifier {

  std::string getDescription() const { return "Reprojectionerror Nodep"; }

  void classifynext(const Sfmreconstruction &reconstruct,
                    array_t &weights) const override {
    array_t new_weights = reconstruct.reprojection_error;

    // set tolerance
    constexpr precision_t reproject_error_tolerance(0.01);
    constexpr precision_t reproject_error_max(5.0);
    constexpr precision_t expweightdist(0.5);
    ExponentialWeighting(reproject_error_tolerance, reproject_error_max,
                         expweightdist, new_weights);

    weights = new_weights;
    return;
  }
};

struct PC_ReprojectionErrorDep1 : public Pointclassifier {
  std::string getDescription() const { return "Reprojectionerror Dep1"; }

  void classifynext(const Sfmreconstruction &reconstruct,
                    array_t &weights) const override {
    array_t new_weights = reconstruct.reprojection_error;

    // set tolerance
    constexpr precision_t reproject_error_tolerance(0.01);
    constexpr precision_t reproject_error_max(5.0);
    constexpr precision_t expweightdist(0.5);

    ExponentialWeighting(reproject_error_tolerance, reproject_error_max,
                         expweightdist, new_weights);

    // dependency on old weights
    const array_t diff = new_weights - weights;
    weights = weights * diff + new_weights; // dep1
    const precision_t resultmax = weights.maxCoeff();
    weights /= resultmax;
  }
};

struct PC_ReprojectionErrorDep2 : public Pointclassifier {
  std::string getDescription() const { return "Reprojectionerror Dep2"; }

  void classifynext(const Sfmreconstruction &reconstruct,
                    array_t &weights) const override {
    array_t new_weights = reconstruct.reprojection_error;

    // set tolerance
    constexpr precision_t reproject_error_tolerance(0.01);
    constexpr precision_t reproject_error_max(5.0);
    constexpr precision_t expweightdist(0.5);

    ExponentialWeighting(reproject_error_tolerance, reproject_error_max,
                         expweightdist, new_weights);

    // dependency on old weights
    const array_t diff = new_weights - weights;
    weights = new_weights * diff + weights; // dep2
    const precision_t resultmax = weights.maxCoeff();
    weights /= resultmax;
  }
};

} // namespace sfmsimulator::pointclassifier

#endif /* end of include guard: __PC_TRIANGULATIONERROR_HH__ */
