#include <sfmsimulator.hh>

#include <vector>

int main(int argc, char const *argv[]) {
  // TODO(dave): add boost program options to properly parse the input
  // input files
  std::string camera_poses("../data/test_small/camera_poses.csv");
  std::string dynamic_landmarks("../data/test_small/landmark_dynamic_3d.csv");
  std::string static_landmarks("../data/test_small/landmark_static_3d.csv");

  sfmsimulator::cameramodel::Cameramodel camera(1.0, 1.0, 1.0, 620, 480);
  sfmsimulator::Sfmconfig config(camera);
  config.filepaths = {camera_poses, static_landmarks, dynamic_landmarks};

  sfmsimulator::Sfmsimulator sfmsim(config);

  sfmsim.run();

  return 0;
}
