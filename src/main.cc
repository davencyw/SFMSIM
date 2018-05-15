#include <sfmsimulator.hh>

#include <cstdlib>
#include <fstream>
#include <iostream>
#include <string>
#include <vector>

#include "boost/program_options.hpp"

// namespace po = boost::program_options;
//
// // TODO(dave): move this to cmdparser!
// void read_settings(std::string filepath, po::options_description &desc,
//                    po::variables_map &vm) {
//
//   std::ifstream settings_file(filepath);
//   vm = po::variables_map();
//   po::store(po::parse_config_file(settings_file, desc), vm);
//   po::notify(vm);
// }

int main(int argc, char const *argv[]) {
  // TODO(dave): add boost program options to properly parse the input
  // input files

  // std::string folder("../data/test_large0");
  std::string testset("Tlarge1");
  std::string folder("../data/" + testset);
  std::string outputfolder("../results/s2_1t3/");
  std::string camera_poses(folder + "/camera_poses.csv");
  std::string dynamic_landmarks(folder + "/landmark_dynamic_3d.csv");
  std::string static_landmarks(folder + "/landmark_static_3d.csv");

  std::vector<std::string> classifier_names{"noclassifier", "nodep", "dep1",
                                            "dep2"};
  std::vector<int> classifier_to_test{2};
  // std::vector<int> classifier_to_test{0, 1, 2, 3};

  sfmsimulator::cameramodel::Cameramodel camera(1.0, 620, 480);
  sfmsimulator::Sfmconfig config(camera);
  config.type_pointclassifier = sfmsimulator::pointclassifier::
      Pointclassifier_type::PC_ReprojectionErrorNodep_t;
  config.filepaths = {camera_poses, static_landmarks, dynamic_landmarks};

  config.camera_noise_amount = 0.1;
  config.world_position_noise_amount = 0.3;
  config.image_detection_noise_amount = -1;
  config.slidingwindow_size = 30;

  for (int classifier_i : classifier_to_test) {
    config.filepaths[3] =
        outputfolder + testset + "_" + classifier_names[classifier_i];
    config.type_pointclassifier =
        sfmsimulator::pointclassifier::Pointclassifier_type(classifier_i);
    sfmsimulator::Sfmsimulator sfmsim(config);
    sfmsim.run();
  }

  return 0;
}
