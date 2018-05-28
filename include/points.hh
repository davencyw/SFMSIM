#ifndef __POINTS_HH__
#define __POINTS_HH__

#include "global.hh"

#include <array>
#include <memory>

namespace sfmsimulator::points {

// struct for 2d feature points at timestep
struct Points2d {
  Points2d() = default;
  Points2d(size_t numpoints) : numpoints(numpoints) {
    coord[0] = array_t(numpoints);
    coord[1] = array_t(numpoints);
    visible = std::vector<bool>(numpoints);
  }

  size_t numpoints;
  // coordinates on image plane
  std::array<array_t, 2> coord;

  // true if visible
  std::vector<bool> visible;
};

// struct for synthetic feature points at timestep with additional type info
struct Syntheticpoints : public Points2d {
  Syntheticpoints(size_t numpoints) : Points2d(numpoints) {}

  // specifies if a feature is static (0) or dynamic (1)
  std::vector<bool> feature_type;
};

// struct for 3d feature points at timestep
struct Points3d {
  Points3d() = default;
  Points3d(size_t numpoints) : numpoints(numpoints) {
    coord[0] = array_t(numpoints);
    coord[1] = array_t(numpoints);
    coord[2] = array_t(numpoints);
    visible = std::vector<bool>(numpoints);
  }

  size_t numpoints;
  // coordinates on image plane
  std::array<array_t, 3> coord;

  // true if visible
  std::vector<bool> visible;
};

struct Dynamicpointgroup {
  size_t numpoints;
  // indices of points in image_coord and world_coord
  int_array_t convexhull;
  int_array_t pointsinsidehull;

  std::shared_ptr<Points2d> image_coord;
  std::shared_ptr<Points3d> world_coord;
};
} // namespace sfmsimulator::points
#endif /* end of include guard: __POINTS_HH__ */
