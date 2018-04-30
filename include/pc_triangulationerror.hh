#ifndef __PC_TRIANGULATIONERROR_HH__
#define __PC_TRIANGULATIONERROR_HH__

#include "pointclassifier.hh"

#include <iostream>

namespace sfmsimulator::pointclassifier {

class PC_Triangulationerror : public Pointclassifier {
public:
  PC_Triangulationerror(cameramodel::Cameramodel camera)
      : Pointclassifier(camera) {}

  const array_t classifynext(Sfmreconstruction reconstruct) const override;

  void cluster(const points::Points2d image_points,
               const std::vector<bool> type) const override;

private:
};

} // namespace sfmsimulator::pointclassifier

#endif /* end of include guard: __PC_TRIANGULATIONERROR_HH__ */
