#include <string>

#include "boost/program_options.hpp"

void parseOptions(int argc, char const *argv[]) {}

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
