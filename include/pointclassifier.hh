#ifndef __POINTCLASSIFIER_HH__
#define __POINTCLASSIFIER_HH__

#include "cameramodel.hh"
#include "global.hh"
#include "points.hh"

#include <vector>

namespace sfmsimulator {

struct Sfmreconstruction {
  std::vector<std::shared_ptr<mat44_t>> camerapose_estimate_mat;
  std::vector<vec6_t> camerapose_estimate;
  std::shared_ptr<points::Points3d> point3d_estimate;
  array_t reprojection_error;
};

namespace pointclassifier {

enum Pointclassifier_type { PC_Triangulationerror_t };

// TODO(dave): create class for clustering!
class Pointclassifier {
public:
  Pointclassifier(cameramodel::Cameramodel cameramodel)
      : _cameramodel(cameramodel) {}

  // classifies points into static and dynamic points
  virtual const array_t classifynext(Sfmreconstruction reconstruct) const = 0;

  // clusters dynamic points specified in type and creates convex hull
  virtual void cluster(const points::Points2d image_points,
                       const std::vector<bool> type) const = 0;

protected:
  const cameramodel::Cameramodel _cameramodel;
};

} // namespace pointclassifier
} /* sfmsimulator */

#endif /* end of include guard: __POINTCLASSIFIER_HH__ */
