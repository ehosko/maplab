#include <iostream>
#include <experimental/filesystem>

#include <console-common/console.h>
#include <gflags/gflags.h>
#include <glog/logging.h>
#include <map-manager/map-manager.h>
#include <maplab-common/file-system-tools.h>
#include <maplab-common/string-tools.h>
#include <vi-map/vi-map.h>
#include <visualization/viwls-graph-plotter.h>
#include <yaml-cpp/yaml.h>
#include <std_msgs/String.h>

#include "maplab-console/maplab-console.h"

// This executable reads yaml files and executes the commands from it on all
// maps in the list. The files have the following format:
// The template string "<CURRENT_VIMAP_FOLDER>" is replaced by the currently
// processed map. This can be used e.g. to save the resulting map to a
// different folder:
//    save -map_folder=<CURRENT_VIMAP_FOLDER>_result
//
// Yaml-format:
//   vi_map_folder_paths:
//     - /some/path/map1
//     - /some/path/map2
//   commands:
//     - command1
//     - command2
//     - command3

namespace fs = std::experimental::filesystem;

const std::string kMapFolderTemplate("<CURRENT_VIMAP_FOLDER>");
const std::string kMapMergeFolderTemplate("<MERGE_MAP_FOLDER>");
const std::string kConsoleName = "maplab-re-localization-eval";

struct BatchControlInformation {
  std::vector<std::string> vi_map_folder_paths;
  std::vector<std::string> built_maps;
  std::vector<std::string> command_flags;
  std::vector<std::string> commands;
};

namespace YAML {
template <>
struct convert<BatchControlInformation> {
  static Node encode(const BatchControlInformation& rhs) {
    Node node;
    node["vi_map_folder_paths"] = rhs.vi_map_folder_paths;
    node["built_maps"] = rhs.built_maps;
    node["command_flags"] = rhs.command_flags;
    node["commands"] = rhs.commands;
    return node;
  }
  static bool decode(
      const Node& node, BatchControlInformation& rhs) {  // NOLINT
    rhs.vi_map_folder_paths =
        node["vi_map_folder_paths"].as<std::vector<std::string>>();
    rhs.built_maps = node["built_maps"].as<std::vector<std::string>>();
    rhs.command_flags = node["command_flags"].as<std::vector<std::string>>();
    rhs.commands = node["commands"].as<std::vector<std::string>>();
    return true;
  }
};
}  // namespace YAML

DEFINE_string(
    batch_control_file, "",
    "Filename of the yaml file that "
    "contains the batch processing information.");

int main(int argc, char** argv) {
//   ros::init(argc, argv, "talker");
  google::InitGoogleLogging(argv[0]);
  google::ParseCommandLineFlags(&argc, &argv, true);
  google::InstallFailureSignalHandler();
  FLAGS_alsologtostderr = true;
  FLAGS_colorlogtostderr = true;


  CHECK_NE(FLAGS_batch_control_file, "")
      << "You have to provide the path to the batch control yaml-file.";

  visualization::ViwlsGraphRvizPlotter::Ptr plotter(
      new visualization::ViwlsGraphRvizPlotter());

  BatchControlInformation control_information;
  if (!YAML::Load(FLAGS_batch_control_file, &control_information)) {
    LOG(FATAL) << "Failed to read batch control file: "
               << FLAGS_batch_control_file;
  }

  std::vector<std::string> map_folders = control_information.vi_map_folder_paths;

  // for (const auto& entry : fs::directory_iterator(control_information.vi_map_folder_paths[0])) {
  //   if (fs::is_directory(entry.status())) {
  //       //std::cout << entry.path() << std::endl;
  //       map_folders.push_back(entry.path());
  //       LOG(INFO) << "Found map folder: " << entry.path();
  //   }
  // }

  std::vector<std::string> built_maps = control_information.built_maps;
  // std::vector<std::string> built_maps;

  // for (const auto& entry : fs::directory_iterator(control_information.built_maps[0])) {
  //   if (fs::is_directory(entry.status())) {
  //       //std::cout << entry.path() << std::endl;
  //       built_maps.push_back(entry.path());
  //       LOG(INFO) << "Found map folder: " << entry.path();
  //   }
  // }

  std::vector<std::string> command_flags = control_information.command_flags;
  std::cout << "command_flags: " << command_flags[0] << std::endl;


  const size_t num_maps = map_folders.size();
  const size_t num_cmds = control_information.commands.size();
  LOG_IF(FATAL, num_maps == 0u) << "No maps supplied with file: "
                                << FLAGS_batch_control_file;
  LOG_IF(FATAL, num_cmds == 0u) << "No commands supplied with file: "
                                << FLAGS_batch_control_file;

  LOG(INFO) << "Got " << num_cmds << " commands to apply on " << num_maps
            << " maps.";

  //std::string built_map = control_information.built_maps[0];

  for (const std::string& map_folder :
       map_folders) {
    CHECK(common::pathExists(map_folder)) << "Map folder path " << map_folder
                                          << " does not exist.";
  }

  // Process all commands for all maps.
  maplab::MapLabConsole console(kConsoleName, argc, argv);

  size_t map_idx = 1u;
  size_t built_map_idx = 1u;
  size_t num_built_maps = built_maps.size();
  size_t num_failed_commands = 0u;
  for(const std::string& built_map :built_maps){
    map_idx = 1u;
    LOG(INFO) << "Running on built map"  << built_map;
    for (const std::string& map_folder :map_folders)
    {
      LOG(INFO) << "Running on built map ("<< built_map_idx << "/"<<  num_built_maps << ") map (" << map_idx << " / " << num_maps
                << "): " << map_folder;
      // LOG(INFO) << "Running on built map (" << map_idx << " / " << num_maps
      //           << "): " << map_folder;

      // Release all maps from memory.
      vi_map::VIMapManager map_manager;
      std::unordered_set<std::string> all_map_keys;
      map_manager.getAllMapKeys(&all_map_keys);
      for (const std::string& key : all_map_keys) {
        map_manager.deleteMap(key);
      }
      const std::string kNoMapSelected = "";
      console.setSelectedMapKey(kNoMapSelected);



      // Run all commands on this map.
      size_t cmd_idx = 1u;
      for (const std::string& command : control_information.commands) {
        // Replace the map_folder template string for the current command.
        std::string actual_command = command;

        if(command == "rlae")
        {
          actual_command += " " + command_flags[built_map_idx-1];
        }

        common::replaceSubstring(kMapFolderTemplate, map_folder, &actual_command);
        common::replaceSubstring(kMapMergeFolderTemplate, built_map, &actual_command);

        LOG(INFO) << "\t Running command (" << cmd_idx << " / " << num_cmds
                  << "): " << actual_command;

        // Run the command.
        if (console.RunCommand(actual_command) != common::kSuccess) {
          LOG(ERROR) << "\t Command failed!";
          ++num_failed_commands;
        } else {
          LOG(INFO) << "\t Command successful.";
        }
        ++cmd_idx;
      }

      LOG(INFO) << "Done running map.";
      ++map_idx;
    }
    ++built_map_idx;
  }

  LOG(INFO) << "Done. Processed " << num_cmds << " commands for " << num_maps
            << " maps.";
  if (num_failed_commands != 0u) {
    LOG(ERROR) << num_failed_commands << " commands failed.";
    return common::kUnknownError;
  }
  return common::kSuccess;
}
