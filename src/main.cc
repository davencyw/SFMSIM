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
  std::string testset("Tlarge3");
  std::string folder("../data/" + testset);
  std::string camera_poses(folder + "/camera_poses.csv");
  std::string dynamic_landmarks(folder + "/landmark_dynamic_3d.csv");
  std::string static_landmarks(folder + "/landmark_static_3d.csv");

  sfmsimulator::cameramodel::Cameramodel camera(1.0, 620, 480);
  sfmsimulator::Sfmconfig config(camera);
  config.type_pointclassifier = sfmsimulator::pointclassifier::
      // Pointclassifier_type::PC_ReprojectionErrorDep1_t;
      Pointclassifier_type::PC_ReprojectionErrorNodep_t;
  config.filepaths = {camera_poses, static_landmarks, dynamic_landmarks,
                      testset + "_nodep"};

  sfmsimulator::Sfmsimulator sfmsim(config);

  sfmsim.run();

  return 0;
}
