#include <sfmsimulator.hh>

#include <cstdlib>
#include <fstream>
#include <iostream>
#include <string>
#include <vector>

#include "boost/program_options.hpp"

int main(int argc, char const *argv[]) {
  // TODO(dave): add boost program options to properly parse the input

  std::vector<std::string> classifier_names{"noclassifier", "nodep", "dep3"};
  std::vector<std::string> testsets{"Tlarge11", "Tlarge14"};
  std::string outputfolder("../results/s11/");
  std::vector<int> classifier_to_test{0, 1, 2};

  for (std::string testset : testsets) {

    std::string folder("../data/" + testset);
    std::string camera_poses(folder + "/camera_poses.csv");
    std::string dynamic_landmarks(folder + "/landmark_dynamic_3d.csv");
    std::string static_landmarks(folder + "/landmark_static_3d.csv");

    sfmsimulator::cameramodel::Cameramodel camera(1.0, 620, 480);
    sfmsimulator::Sfmconfig config(camera);
    config.type_pointclassifier = sfmsimulator::pointclassifier::
        Pointclassifier_type::PC_ReprojectionErrorNodep_t;
    config.filepaths = {camera_poses, static_landmarks, dynamic_landmarks};

    config.image_detection_noise_amount = 0.005;
    config.camera_noise_amount = 0.3;
    config.world_position_noise_amount = 0.3;
    config.slidingwindow_size = 50;

    // iterate over classifiers
    for (int classifier_i : classifier_to_test) {
      config.filepaths[3] =
          outputfolder + testset + "_" + classifier_names[classifier_i];
      config.type_pointclassifier =
          sfmsimulator::pointclassifier::Pointclassifier_type(classifier_i);
      sfmsimulator::Sfmsimulator sfmsim(config);
      sfmsim.run();
    }
  }

  return 0;
}
