#ifndef __POINTCLASSIFIER_HH__
#define __POINTCLASSIFIER_HH__

#include "cameramodel.hh"
#include "global.hh"
#include "points.hh"

#include <string>
#include <vector>

namespace sfmsimulator {

struct Sfmreconstruction {
  std::vector<std::shared_ptr<mat44_t>> camerapose_estimate_mat;
  std::vector<vec6_t> camerapose_estimate;
  std::shared_ptr<points::Points3d> point3d_estimate;
  array_t reprojection_error;
};

namespace pointclassifier {

enum Pointclassifier_type {
  PC_Noclassifier_t,
  PC_ReprojectionErrorNodep_t,
  PC_ReprojectionErrorDep1_t,
  PC_ReprojectionErrorDep2_t,
  PC_ReprojectionErrorDep3_t,
};

// TODO(dave): create class for clustering!
class Pointclassifier {
public:
  // classifies points into static and dynamic points
  virtual void classifynext(const Sfmreconstruction &reconstruct,
                            array_t &weights) const = 0;
  virtual std::string getDescription() const = 0;
};

} // namespace pointclassifier
} /* sfmsimulator */

#endif /* end of include guard: __POINTCLASSIFIER_HH__ */
