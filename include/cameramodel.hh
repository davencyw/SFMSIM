#ifndef __CAMERAMODEL_HH__
#define __CAMERAMODEL_HH__

#include "global.hh"

#include <opencv2/core/core.hpp>

namespace sfmsimulator::cameramodel {

struct Imageplane {
  Imageplane(const size_t width, const size_t height)
      : width(width), height(height) {}

  size_t width;
  size_t height;
};

class Cameramodel {
public:
  Cameramodel(const precision_t f, const precision_t cx, const precision_t cy,
              const size_t image_width, const size_t image_height)
      : _f(f), _cx(cx), _cy(cy),
        _imageplane(Imageplane(image_width, image_height)) {}

  const inline cv::Matx33d getK() const {
    return cv::Matx33d(_f, 0, _cx, 0, _f, _cy, 0, 0, 1);
  }

  const Imageplane getImageplane() const { return _imageplane; }

private:
  const precision_t _f;
  const precision_t _cx;
  const precision_t _cy;

  const Imageplane _imageplane;
};

} // namespace sfmsimulator::cameramodel

#endif /* end of include guard: __CAMERAMODEL_HH__ */
