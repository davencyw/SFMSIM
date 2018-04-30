#include "bundleadjustment.hh"
#include "pc_triangulationerror.hh"
#include "sfmsimulator.hh"

#include <opencv2/core/affine.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/core/eigen.hpp>
#include <opencv2/sfm.hpp>

#include <cassert>

namespace sfmsimulator {

Sfmsimulator::Sfmsimulator(Sfmconfig config)
    : _config(config), _cameramodel(config.cameramodel),
      _framesimulator(framesimulator::Framesimulator(config.filepaths,
                                                     config.cameramodel)) {
  using pct = pointclassifier::Pointclassifier_type;

  switch (config.type_pointclassifier) {
  case pct::PC_Triangulationerror_t:
    _pointclassifier = std::make_unique<pointclassifier::PC_Triangulationerror>(
        pointclassifier::PC_Triangulationerror(_cameramodel));
    break;
  default:
    break;
  }

  std::cout
      << "\033[0m\n"
      << "\n\n\n\n"
      << "            __\n"
      << "     ___   / _|  _ __ ___      \n"
      << "    / __| | |_  | '_ ` _ \\     \n"
      << "    \\__ \\ |  _| | | | | | |     \n"
      << "    |___/ |_|   |_| |_| |_|   \n\n"
      << "           _                       _           _\n"
      << "     ___  (_)  _ __ ___    _   _  | |   __ _  | |_    ___    _ __ "
      << "\n    / __| | | | '_ ` _ \\  | | | | | |  / _` | | __|  / _ \\  | "
      << "'__|\n"
      << "    \\__ \\ | | | | | | | | | |_| | | | | (_| | | |_  | (_) | | |  "
      << " \n"
      << "    |___/ |_| |_| |_| |_|  \\__,_| |_|  \\__,_|  \\__|  \\___/  "
      << "|_| \n\n\n\n"
      << "    author: david schmidig [david@davencyw.net]\n"
      << "            davencyw code  [davencyw.net]\n"
      << "            ETH Zurich\n\n"
      << "_____________________________________________________________________"
         "\n\n\n";
}

void Sfmsimulator::run() {
  const size_t steps(_framesimulator.updatesLeft());
  std::cout << "STEPS: " << steps << "\n\n\n";
  doSteps(steps);
}

void Sfmsimulator::doSteps(const size_t steps) {
  for (size_t step_i(0); step_i < steps; step_i++) {
    step();
  }
}

// TODO(dave) add generic depending sliding window to pc_classifier
// TODO(dave) separate initializationsteps (_steps=0,1) from this method
void Sfmsimulator::step() {
  std::cout << " - STEP[ " << _step << " ]\n";

  _framesimulator.step();

  _scene_window_image.push_front(
      std::make_shared<points::Points2d>(_framesimulator.getImagePoints()));
  _scene_window_cameraposes.push_front(_framesimulator.getCameraPose());

  // initialization step
  if (_scene_window_image.size() < 2) {
    assert(_step == 0);
    _weights = array_t::Ones(_scene_window_image[0]->numpoints);
    ++_step;
    return;
  }

  assert(_scene_window_image.size() == 2);
  // newer frame is older (frame2.time > frame1.time) but is placed in the front
  // of the deque!
  std::shared_ptr<points::Points3d> points_world2;
  std::cout << " -    reconstruction \n";

  std::shared_ptr<points::Points3d> world_points(
      std::make_shared<points::Points3d>(_framesimulator.getWorldPoints()));

  std::vector<std::shared_ptr<points::Points2d>> frames;
  std::vector<vec6_t> cameraposes;

  for (auto &frame_i : _scene_window_image) {
    frames.push_back(frame_i);
  }

  for (auto &pose_i : _scene_window_cameraposes) {
    cameraposes.push_back(pose_i);
  }

  Sfmreconstruction reconstruct = bundleadjustment::adjustBundle(
      frames, world_points, cameraposes, _cameramodel, _weights);

  _scene_window_world.push_front(reconstruct.point3d_estimate);
  _scene_window_cameraposes_mat.push_front(reconstruct.camerpose_estimate[1]);

  if (_pointclassifier) {
    // classify and reconstruct with only static points
    _weights = _pointclassifier->classifynext(reconstruct);
    std::cout << " -    classify \n";
  }

  _scene_window_world.pop_back();
  _scene_window_image.pop_back();
  _scene_window_cameraposes.pop_back();
  ++_step;
}

} // namespace sfmsimulator
