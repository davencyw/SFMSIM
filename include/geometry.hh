#ifndef __GEOMETRY_HH__
#define __GEOMETRY_HH__

#include <numeric>

namespace sfmsimulator::geometry {

// inline does not make a change for this shared_ptr pass
inline precision_t cross(const size_t O, const size_t A, const size_t B,
                         std::shared_ptr<points::Points2d> points) {
  const precision_t OAx(points->coord[0](A) - points->coord[0](O));
  const precision_t OBx(points->coord[0](B) - points->coord[0](O));
  const precision_t OAy(points->coord[1](A) - points->coord[1](O));
  const precision_t OBy(points->coord[1](B) - points->coord[1](O));
  return OAx * OBy - OAy * OBx;
}

// sorts indices from points lexicographically
std::vector<size_t>
sort_indices(const std::shared_ptr<points::Points2d> points) {

  // initialize original index locations
  std::vector<size_t> sorted_indices(points->numpoints);
  iota(sorted_indices.begin(), sorted_indices.end(), 0);

  // sort with lambda
  sort(sorted_indices.begin(), sorted_indices.end(),
       [&points](size_t i1, size_t i2) {
         const precision_t i1x(points->coord[0](i1));
         const precision_t i1y(points->coord[1](i1));
         const precision_t i2x(points->coord[0](i2));
         const precision_t i2y(points->coord[1](i2));
         return i1x < i2x || (i1x == i2x && i1y < i2y);
       });

  return sorted_indices;
}

// implements the monotone chain algorithm
void inline getConvexHull(points::Dynamicpointgroup &pointgroup) {

  size_t numpoints = pointgroup.numpoints;

  if (numpoints <= 3) {
    // hull is equal to all points
    std::swap(pointgroup.pointsinsidehull, pointgroup.convexhull);
    return;
  }
  int_array_t hull(numpoints * 2);
  std::vector<size_t> sorted_indices = sort_indices(pointgroup.image_coord);
  size_t k(0);

  for (size_t point_i(0); point_i < numpoints; ++point_i) {
    while (k >= 2 &&
           cross(hull(k - 2), hull(k - 1), sorted_indices[point_i],
                 pointgroup.image_coord) <= 0) {
      --k;
    }
    hull(++k) = sorted_indices[point_i];
  }

  size_t t(k + 1);
  for (size_t point_i(numpoints - 1); point_i > 0; --point_i) {
    while (k >= t &&
           cross(hull(k - 2), hull(k - 1), sorted_indices[point_i - 1],
                 pointgroup.image_coord) <= 0) {
      --k;
    }
    hull(k++) = sorted_indices[point_i - 1];
  }

  pointgroup.convexhull = hull;
}
bool inline isInside2dPolygon(const precision_t x_coordinate,
                              const precision_t y_coordinate,
                              const size_t height, const size_t width) {
  return x_coordinate > 0.0 && y_coordinate > 0.0 && x_coordinate <= width &&
         y_coordinate <= height;
}

} /* sfmsimulator::geometry */

#endif /* end of include guard: __GEOMETRY_HH__ */
