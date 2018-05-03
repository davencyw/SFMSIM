#include "bundleadjustment.hh"
#include "pc_triangulationerror.hh"
#include "sfmsimulator.hh"

#include <random>

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

  if (config.filepaths.size() > 3) {
    _file_output = config.filepaths[3];
  }
  _fstream_output_weights = std::make_unique<std::ofstream>();
  _fstream_output_camera_trajectory = std::make_unique<std::ofstream>();
  _fstream_output_weights->open(_file_output + "weights.csv");
  _fstream_output_camera_trajectory->open(_file_output + "camera.csv");

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
      << "___________________________________________________________________"
         "__"
         "\n\n\n";
}

void Sfmsimulator::run() {
  const size_t steps(_framesimulator.updatesLeft());
  std::cout << "STEPS: " << steps << "\n\n\n";

  for (size_t weight_i(0); weight_i < _weights.size(); ++weight_i) {
    *_fstream_output_weights << _weights(weight_i) << ",";
  }
  *_fstream_output_weights << "\n";

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

  _scene_window_image.push_back(
      std::make_shared<points::Points2d>(_framesimulator.getImagePoints()));
  _scene_window_cameraposes.push_back(_framesimulator.getCameraPose());

  const size_t numpoints(_scene_window_image[0]->numpoints);

  // initialization step
  if (_scene_window_image.size() < 2) {
    assert(_step == 0);
    _weights = array_t::Ones(numpoints);
    ++_step;
    return;
  }

  assert(_scene_window_image.size() == 2);

  std::cout << " -    reconstruction \n";

  std::shared_ptr<points::Points3d> world_points(
      std::make_shared<points::Points3d>(_framesimulator.getWorldPoints()));

  std::vector<std::shared_ptr<points::Points2d>> frames;
  std::vector<vec6_t> cameraposes;

  // convert from deque to vector
  for (auto &frame_i : _scene_window_image) {
    frames.push_back(frame_i);
  }
  // have to bemutable for BA, so copy into new vector
  for (auto &pose_i : _scene_window_cameraposes) {
    cameraposes.push_back(pose_i);
  }

  // add noise to ground truth
  addNoise(world_points, cameraposes, 0);

  Sfmreconstruction reconstruct = bundleadjustment::adjustBundle(
      frames, world_points, cameraposes, _cameramodel, _weights);

  _scene_window_world.push_front(reconstruct.point3d_estimate);
  _scene_window_cameraposes_mat.push_front(
      reconstruct.camerapose_estimate_mat[1]);

  if (_pointclassifier) {
    // classify and reconstruct with only static points
    _weights = _pointclassifier->classifynext(reconstruct);
    std::cout << " -    classify \n";
  }

  output(reconstruct);

  _scene_window_world.pop_back();
  _scene_window_image.pop_front();
  _scene_window_cameraposes.pop_front();
  ++_step;
}

void Sfmsimulator::addNoise(std::shared_ptr<points::Points3d> points,
                            std::vector<vec6_t> cameraposes,
                            precision_t amount) {

  if (amount == 0) {
    return;
  }

  const size_t numpoints(points->coord.size());
  //  add noise to worldpoints and camerapose

  std::random_device rd{};
  std::mt19937 gen{rd()};
  std::normal_distribution<> d{0, amount};

  for (size_t point_i(0); point_i < numpoints; ++point_i) {
    precision_t random(d(gen));
    points->coord[0] += random;
    random = d(gen);
    points->coord[1] += random;
    random = d(gen);
    points->coord[2] += random;
  }

  // max = _scene_window_cameraposes.front().maxCoeff();
  // for (size_t param_i(0); param_i < 6; ++param_i) {
  //   const precision_t random(d(gen));
  //   _scene_window_cameraposes.front()(param_i) += max * random;
  // }
}

void Sfmsimulator::output(const Sfmreconstruction &reconstruct) const {
  for (size_t weight_i(0); weight_i < _weights.size(); ++weight_i) {
    *_fstream_output_weights << _weights(weight_i) << ",";
  }
  *_fstream_output_weights << "\n";

  for (size_t coeff_i(0); coeff_i < 6; ++coeff_i) {
    // ground truth
    *_fstream_output_camera_trajectory
        << (_scene_window_cameraposes.back())(coeff_i) << " ";
  }
  *_fstream_output_camera_trajectory << "\n";

  auto camerapose =
      reconstruct
          .camerapose_estimate[reconstruct.camerapose_estimate.size() - 1];
  for (size_t coeff_i(0); coeff_i < 6; ++coeff_i) {
    // estimate
    *_fstream_output_camera_trajectory << camerapose(coeff_i) << " ";
  }
  *_fstream_output_camera_trajectory << "\n";
}

} // namespace sfmsimulator
