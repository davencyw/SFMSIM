#ifndef __PC_TRIANGULATIONERROR_HH__
#define __PC_TRIANGULATIONERROR_HH__

#include "pointclassifier.hh"

namespace sfmsimulator::pointclassifier {

class PC_Triangulationerror : Pointclassifier {
 public:
  void classify(points::Points2d image_points_frame_1,
                points::Points2d image_points_frame_2,
                points::Points3d world_points_frame_1,
                points::Points3d world_points_frame_2);
  void cluster(points::Points2d image_points, std::vector<bool> type);

 private:
};

}  // namespace sfmsimulator::pointclassifier

#endif /* end of include guard: __PC_TRIANGULATIONERROR_HH__ */
