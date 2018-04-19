#ifndef __VISUALIZER_HH__
#define __VISUALIZER_HH__

namespace sfmsimulator::visualizer {

class Visualizer {
 public:
  void updateCamera();
  void updateDynamicLandmarks();
  void changeView();

 private:
};

}  // namespace sfmsimulator::visualizer

#endif /* end of include guard: __VISUALIZER_HH__ */
