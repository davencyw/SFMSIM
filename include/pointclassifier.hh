#ifndef __POINTCLASSIFIER_HH__
#define __POINTCLASSIFIER_HH__

#include "points.hh"

#include <vector>

namespace sfmsimulator::pointclassifier {

// TODO(dave): create class for clustering!
class Pointclassifier {
 public:
  // classifies points into static and dynamic points
  virtual void classify(points::Points2d image_points_frame_1,
                        points::Points2d image_points_frame_2,
                        points::Points3d world_points_frame_1,
                        points::Points3d world_points_frame_2) = 0;
  // clusters dynamic points specified in type and creates convex hull
  virtual void cluster(points::Points2d image_points,
                       std::vector<bool> type) = 0;

 private:
};

}  // namespace sfmsimulator::pointclassifier

#endif /* end of include guard: __POINTCLASSIFIER_HH__ */
