#ifndef __POINTCLASSIFIER_HH__
#define __POINTCLASSIFIER_HH__

#include "cameramodel.hh"
#include "global.hh"
#include "points.hh"

#include <vector>

namespace sfmsimulator::pointclassifier {

enum Pointclassifier_type { PC_Triangulationerror_t };

// TODO(dave): create class for clustering!
class Pointclassifier {
 public:
  Pointclassifier(cameramodel::Cameramodel cameramodel)
      : _cameramodel(cameramodel){};

  // classifies points into static and dynamic points
  virtual const std::vector<bool> classify(
      const std::shared_ptr<points::Points2d> image_points_frame_1,
      const std::shared_ptr<points::Points2d> image_points_frame_2,
      const std::shared_ptr<points::Points3d> world_points_frame_1,
      const std::shared_ptr<points::Points3d> world_points_frame_2) const {}

  // clusters dynamic points specified in type and creates convex hull
  virtual void cluster(const points::Points2d image_points,
                       const std::vector<bool> type) const {}

 private:
  const cameramodel::Cameramodel _cameramodel;
};

}  // namespace sfmsimulator::pointclassifier

#endif /* end of include guard: __POINTCLASSIFIER_HH__ */
