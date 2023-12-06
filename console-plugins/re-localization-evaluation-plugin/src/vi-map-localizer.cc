#include "re-localization-evaluation-plugin/vi-map-localizer.h"

#include <loop-closure-handler/loop-detector-node.h>
#include <maplab-common/file-system-tools.h>
#include <vi-map/vi-map.h>

DEFINE_string(evaluate_mission, "", "Mission to be evaluated.");

namespace re_localization_evaluation_plugin {

VIMapLocalizer::VIMapLocalizer(
    vi_map::VIMap* map)
    : map_(map) {
  CHECK_NOTNULL(map_);
}

int VIMapLocalizer::reLocalizeAllMissions(const std::string selected_map_key) {
  if (map_->numMissions() == 0u) {
    LOG(ERROR) << "No missions in database.";
    return kNoData;
  }

  vi_map::MissionIdList mission_ids;
  // Make sure that the to-be-evaluated-map is the first here
  map_->getAllMissionIds(&mission_ids);

  // TODO: Probably have to emplace the to-be-evaluated mission at the front of the list

  return reLocalizeOneMission(mission_ids, selected_map_key);
}

int VIMapLocalizer::reLocalizeOneMission(
    const vi_map::MissionIdList& mission_ids, const std::string selected_map_key) {
  // Relocalizes one mission at a time against the first mission
  // in the mission_ids list, which should be the mission to be evaluated

  if (mission_ids.empty()) {
    LOG(ERROR) << "There are no missions in the loaded map. Aborting.";
    return common::kUnknownError;
  }

  if (FLAGS_evaluate_mission.empty()) {
    LOG(ERROR) << "Specify a valid mission with -evaluate_mission.";
    return common::kStupidUserError;
  }

  // Get ID of the to be evaluated map
  vi_map::MissionId evaluate_mission_id;
  map_->ensureMissionIdValid(FLAGS_evaluate_mission, &evaluate_mission_id);
  VLOG(1) << "To be evaluated mission: " << evaluate_mission_id;

  // We don't want to merge landmarks or add loop closure edges
  const bool kAddLoopClosureEdges = false;
  const bool kMergeLandmarks = false;

  // We only want to localize all benchmark submaps w.r.t. the main map
  
  loop_detector_node::LoopDetectorNode loop_detector;
  loop_detector.addMissionToDatabase(evaluate_mission_id, *map_);
  for (vi_map::MissionIdList::const_iterator jt = mission_ids.begin();
        jt != mission_ids.end(); ++jt) {
    if (*jt == evaluate_mission_id) {
      VLOG(1) << "Skipping mission " << *jt << " because it is the same as the "
              << "to-be-evaluated mission. Also, something went wrong here!";
      continue;
    }
    pose::Transformation T_G_M2;
    vi_map::LoopClosureConstraintVector inlier_constraints;
    // loop_detector.detectLoopClosuresAndMergeLandmarks(*jt, map_);
    VLOG(1) << "Localizing mission " << *jt << " w.r.t. mission " << evaluate_mission_id << ".";
    loop_detector.detectLocalizationMissionToDatabase(
        *jt, kMergeLandmarks, kAddLoopClosureEdges, map_, &T_G_M2,
        &inlier_constraints, selected_map_key);
  }
  
  

  return common::kSuccess;
}

}  // namespace loop_closure_plugin
