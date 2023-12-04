#include "re-localization-evaluation-plugin/re-localization-evaluation-plugin.h"
#include "re-localization-evaluation-plugin/vi-map-localizer.h"

#include <console-common/console.h>
#include <map-manager/map-manager.h>
#include <vi-map/vi-map.h>

// TODO: (michbaum) Check what is needed
#include <descriptor-projection/train-projection-matrix.h>
#include <posegraph/pose-graph.h>
#include <posegraph/unique-id.h>
#include <vi-map/vi-map-serialization.h>
#include <visualization/viwls-graph-plotter.h>

#include <loop-closure-handler/loop-detector-node.h>

DECLARE_string(map_mission);

namespace re_localization_evaluation_plugin {

ReLocalizationEvaluationPlugin::ReLocalizationEvaluationPlugin(
    common::Console* console)
    : common::ConsolePluginBase(console) {
  CHECK_NOTNULL(console);

  addCommand(
      {"rle", "re_localization_evaluation_all_query_missions"},
      [this]() -> int { return evaluateReLocalizationForAllBenchmarkMissions(); },
      "Evaluate the re-localization accuracy between all query missions and the to be evaluated mission.", common::Processing::Sync);

  addCommand(
      {"rleom", "re_localization_evaluation_one_query_mission"},
      [this]() -> int { return evaluateReLocalizationForOneBenchmarkMission(); },
      "Evaluate the re-localization accuracy between one query missions and the to be evaluated mission.", common::Processing::Sync);

}

bool areQualitiesOfAllLandmarksSet(const vi_map::VIMap& map) {
  vi_map::LandmarkIdList landmark_ids;
  map.getAllLandmarkIds(&landmark_ids);
  // Check all landmarks to ensure that their quality is not unknown.
  for (const vi_map::LandmarkId& landmark_id : landmark_ids) {
    if (map.getLandmark(landmark_id).getQuality() ==
        vi_map::Landmark::Quality::kUnknown) {
      return false;
    }
  }
  return true;
}


int ReLocalizationEvaluationPlugin::evaluateReLocalizationForAllBenchmarkMissions() const {
  std::string selected_map_key;
  if (!getSelectedMapKeyIfSet(&selected_map_key)) {
    return common::kStupidUserError;
  }
  vi_map::VIMapManager map_manager;
  vi_map::VIMapManager::MapWriteAccess map =
      map_manager.getMapWriteAccess(selected_map_key);

  if (!areQualitiesOfAllLandmarksSet(*map)) {
    LOG(ERROR) << "Some landmarks are of unknown quality. Update them with the "
               << "elq command.";
    return common::kStupidUserError;
  }


  VIMapLocalizer localizer(map.get());
  return localizer.reLocalizeAllMissions(selected_map_key);
}

int ReLocalizationEvaluationPlugin::evaluateReLocalizationForOneBenchmarkMission() const {
  std::string selected_map_key;
  if (!getSelectedMapKeyIfSet(&selected_map_key)) {
    return common::kStupidUserError;
  }
  vi_map::VIMapManager map_manager;
  vi_map::VIMapManager::MapWriteAccess map =
      map_manager.getMapWriteAccess(selected_map_key);

  if (!areQualitiesOfAllLandmarksSet(*map)) {
    LOG(ERROR) << "Some landmarks are of unknown quality. Update them with the "
               << "elq command.";
    return common::kStupidUserError;
  }

  if (FLAGS_map_mission.empty()) {
    LOG(ERROR) << "Specify a valid mission with -map_mission.";
    return common::kStupidUserError;
  }

  vi_map::MissionIdList mission_ids;
  // Get ID of the benchmark map
  vi_map::MissionId mission_id;
  map->ensureMissionIdValid(FLAGS_map_mission, &mission_id);
  mission_ids.emplace_back(mission_id);
  VLOG(1) << "Benchmark mission: " << mission_id;
  VLOG(1) << "Map key: " << selected_map_key;

  VIMapLocalizer localizer(map.get());
  return localizer.reLocalizeOneMission(mission_ids, selected_map_key);
}

}  // namespace re_localization_evaluation_plugin

MAPLAB_CREATE_CONSOLE_PLUGIN(
    re_localization_evaluation_plugin::ReLocalizationEvaluationPlugin);
