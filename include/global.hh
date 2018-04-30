#ifndef __GLOBAL_HH__
#define __GLOBAL_HH__

#include <eigen3/Eigen/Dense>

// global typedefs
typedef double precision_t;
typedef Eigen::Array<precision_t, Eigen::Dynamic, 1> array_t;
typedef Eigen::Array<size_t, Eigen::Dynamic, 1> int_array_t;
typedef Eigen::Matrix<precision_t, Eigen::Dynamic, Eigen::Dynamic,
                      Eigen::RowMajor>
    mat_t;
typedef Eigen::Matrix<precision_t, 3, 3, Eigen::RowMajor> mat33_t;
typedef Eigen::Matrix<precision_t, 4, 4, Eigen::RowMajor> mat44_t;
typedef Eigen::Matrix<precision_t, 3, 1> vec3_t;
typedef Eigen::Matrix<precision_t, 4, 1> vec4_t;
typedef Eigen::Matrix<precision_t, 6, 1> vec6_t;

#endif /* end of include guard: __GLOBAL_HH__ */
