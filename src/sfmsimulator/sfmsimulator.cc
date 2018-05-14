#include "bundleadjustment.hh"
#include "pc_reprojectionerror.hh"
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

  using pct = pointclassifier::Pointclassifier_type;

  switch (config.type_pointclassifier) {
  case pct::PC_ReprojectionErrorDep1_t:
    _pointclassifier =
        std::make_unique<pointclassifier::PC_ReprojectionErrorDep1>(
            pointclassifier::PC_ReprojectionErrorDep1());
    break;
  case pct::PC_ReprojectionErrorDep2_t:
    _pointclassifier =
        std::make_unique<pointclassifier::PC_ReprojectionErrorDep2>(
            pointclassifier::PC_ReprojectionErrorDep2());
    break;
  case pct::PC_ReprojectionErrorNodep_t:
    _pointclassifier =
        std::make_unique<pointclassifier::PC_ReprojectionErrorNodep>(
            pointclassifier::PC_ReprojectionErrorNodep());
    break;
  default:
    std::cout << "No classifier defined!!\n\n";
    break;
  }

  if (_pointclassifier) {
    std::cout << "classifiertype: " << _pointclassifier->getDescription()
              << "\n\n";
  }

  if (config.filepaths.size() > 3) {
    _file_output = config.filepaths[3];
  }

  // start output filestreams
  _fstream_output_weights = std::make_unique<std::ofstream>();
  _fstream_output_camera_trajectory = std::make_unique<std::ofstream>();
  _fstream_output_camera_trajectory_groundtruth =
      std::make_unique<std::ofstream>();
  _fstream_output_weights->open(_file_output + "_weights.csv");
  _fstream_output_camera_trajectory->open(_file_output + "_camera.csv");
  _fstream_output_camera_trajectory_groundtruth->open(_file_output +
                                                      "_camera_gt.csv");

  _weights = array_t::Ones(_framesimulator.getNumPoints());
}

void Sfmsimulator::updateSlidingWindow() {
  _framesimulator.step();
  _scene_window_image.push_back(
      std::make_shared<points::Points2d>(_framesimulator.getImagePoints()));
  _scene_window_cameraposes.push_back(_framesimulator.getCameraPose());
  _world_points =
      std::make_shared<points::Points3d>(_framesimulator.getWorldPoints());

  // write ground truth to output
  vec6_t *camera_ground_truth = &(_scene_window_cameraposes.back());
  for (size_t camera_i(0); camera_i < 6; camera_i++) {
    *_fstream_output_camera_trajectory_groundtruth
        << (*camera_ground_truth)[camera_i] << " ";
  }
  *_fstream_output_camera_trajectory_groundtruth << "\n";

  // handle noise
  addNoise(_world_points, &(_scene_window_cameraposes.back()), 0.3);

  if (_scene_window_image.size() > _config.slidingwindow_size) {
    _scene_window_image.pop_front();
    _scene_window_cameraposes.pop_front();
  }
}

void Sfmsimulator::run() {
  const size_t steps(_framesimulator.updatesLeft());
  std::cout << "STEPS: " << steps << "\n\n\n";

  // initializationstep because we need atleast two frames to reconstruct
  updateSlidingWindow();

  for (size_t weight_i(0); weight_i < static_cast<size_t>(_weights.size());
       ++weight_i) {
    *_fstream_output_weights << _weights(weight_i) << ",";
  }
  *_fstream_output_weights << "\n";

  for (size_t step_i(0); step_i < steps; step_i++) {
    step();
  }
}

// TODO(dave) add generic depending sliding window to pc_classifier
// TODO(dave) separate initializationsteps (_steps=0,1) from this method
void Sfmsimulator::step() {
  std::cout << " - STEP[ " << _step << " ]\n";

  _framesimulator.step();
  updateSlidingWindow();

  // convert from deque to vector
  std::vector<std::shared_ptr<points::Points2d>> frames;
  std::vector<vec6_t> cameraposes;
  for (auto &frame_i : _scene_window_image) {
    frames.push_back(frame_i);
  }
  for (auto &pose_i : _scene_window_cameraposes) {
    cameraposes.push_back(pose_i);
  }

  Sfmreconstruction reconstruct = bundleadjustment::adjustBundle(
      frames, _world_points, cameraposes, _cameramodel, _weights);

  // push back estimates
  _scene_full_camera_estimate.push_back(reconstruct.camerapose_estimate);

  // classify points
  if (_pointclassifier) {
    // classify and reconstruct with only static points
    _pointclassifier->classifynext(reconstruct, _weights);
  }

  // write camera estimates to output
  for (auto &camerapose_i : reconstruct.camerapose_estimate) {
    for (size_t coeff_i(0); coeff_i < 6; ++coeff_i) {
      *_fstream_output_camera_trajectory << camerapose_i[coeff_i] << " ";
    }
    *_fstream_output_camera_trajectory << "\n";
  }

  // write weights to output
  for (size_t weight_i(0); weight_i < static_cast<size_t>(_weights.size());
       ++weight_i) {
    *_fstream_output_weights << _weights(weight_i) << ",";
  }
  *_fstream_output_weights << "\n";

  ++_step;
}

void Sfmsimulator::addNoise(std::shared_ptr<points::Points3d> points,
                            vec6_t *cameraposes, precision_t amount) {

  //  add noise to worldpoints and camerapose
  if (amount == 0) {
    return;
  }

  std::random_device rd{};
  std::mt19937 gen{rd()};
  std::normal_distribution<> d{0, amount};

  if (_config.noise_3dposition) {
    const size_t numpoints(points->coord.size());
    for (size_t point_i(0); point_i < numpoints; ++point_i) {
      precision_t random(d(gen));
      points->coord[0] += random;
      random = d(gen);
      points->coord[1] += random;
      // no z noise for the boyz
      // random = d(gen);
      // points->coord[2] += random;
    }
  }
  if (_config.noise_camera) {
    for (size_t param_i(0); param_i < 6; ++param_i) {
      const precision_t random(d(gen));
      (*cameraposes)(param_i) += random;
    }
  }
}

void Sfmsimulator::output(const Sfmreconstruction &reconstruct) const {
  for (size_t weight_i(0); weight_i < static_cast<size_t>(_weights.size());
       ++weight_i) {
    *_fstream_output_weights << _weights(weight_i) << ",";
  }
  *_fstream_output_weights << "\n";
}

} // namespace sfmsimulator
