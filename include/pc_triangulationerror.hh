#ifndef __PC_TRIANGULATIONERROR_HH__
#define __PC_TRIANGULATIONERROR_HH__

#include "pointclassifier.hh"

namespace sfmsimulator::pointclassifier {

class PC_Triangulationerror : Pointclassifier {
 public:
  void classify();
  void cluster();

 private:
};

}  // namespace sfmsimulator::pointclassifier

#endif /* end of include guard: __PC_TRIANGULATIONERROR_HH__ */
