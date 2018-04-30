#include "bundleadjustment.hh"
#include "pc_triangulationerror.hh"
#include "sfmsimulator.hh"

#define CERES_FOUND 1
#define CV_OVERRIDE override

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

  _scene_camera_pose.setIdentity();
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

  _scene_window_image.push_front(std::make_shared<points::Points2d>(
      _framesimulator.step_GetImagePoints()));

  // initialization step
  if (_scene_window_image.size() < 2) {
    assert(_step == 0);
    ++_step;
    return;
  }

  assert(_scene_window_image.size() == 2);
  // newer frame is older (frame2.time > frame1.time) but is placed in the front
  // of the deque!
  std::shared_ptr<points::Points3d> points_world2;
  std::cout << " -    reconstruction \n";
  Sfmreconstruction reconstruction(
      reconstruct(_scene_window_image[1], _scene_window_image[0]));
  _scene_window_world.push_front(reconstruction.point3d_estimate);

  _scene_camera_pose = _scene_camera_pose * (*reconstruction.transformation);
  _scene_camera_transforms.push_front(reconstruction.transformation);

  // needs two 3d pointestimate for initialization
  if (_scene_window_world.size() < 2) {
    assert(_step == 1);
    ++_step;
    return;
  }

  if (_pointclassifier) {
    // classify and reconstruct with only static points
    _pointclassifier->classifynext(
        _scene_window_image[1], _scene_window_image[0], _scene_window_world[1],
        _scene_window_world[0]);
    std::cout << " -    classify \n";
  }

  //   _scene_window_world.pop_back();
  _scene_window_image.pop_back();
  ++_step;
}

Sfmreconstruction
Sfmsimulator::reconstruct(std::shared_ptr<points::Points2d> points_frame1,
                          std::shared_ptr<points::Points2d> points_frame2) {

  // TODO(dave): feed cameraposes and 3d poses

  return bundleadjustment::Bundleadjustment::adjustBundle(
      points_frame1, points_frame2, , _cameramodel);
}

} // namespace sfmsimulator
