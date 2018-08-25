#include <string>

#include "boost/program_options.hpp"

struct InputConfig {
  std::vector<std::string> testsets;
  std::string outputfolder;
  std::vector<int> classifier_to_test;
  // std::vector<std::string> testsets{"Tlarge15", "Tlarge16"};
  // std::string outputfolder("../results/");
  // std::vector<int> classifier_to_test{0, 1, 2};
};

InputConfig parseOptions(int argc, char const *argv[]) {
  namespace po = boost::program_options;

  InputConfig config;
  po::options_description desc("Options");
  // clang-format off
  desc.add_options()
    ("help", "Print help messages")
    ("outputfolder,o",po::value<std::string>(&(config.outputfolder))->required(),"outputfolder")
    ("classifiers,c",po::value<std::vector<int>>(&(config.classifier_to_test))->required(),"classifiers")
    ("testsets,t",po::value<std::vector<std::string>>(&(config.testsets))->required(),"sets to test");
  // clang-format on

  po::variables_map vm;
  try {
    po::store(po::parse_command_line(argc, argv, desc),
              vm); // can throw

    if (vm.count("help")) {
      std::cout << "Basic Command Line Parameter" << std::endl
                << desc << std::endl;
      return config;
    }

    po::notify(vm); // throws on error, so do after help in case
                    // there are any problems
  } catch (po::error &e) {
    std::cerr << "ERROR: " << e.what() << std::endl << std::endl;
    std::cerr << desc << std::endl;
    return config;
  }

  return config;
}
