#include "loop-closure-plugin/vi-map-merger.h"

#include <loop-closure-handler/loop-detector-node.h>
#include <maplab-common/file-system-tools.h>
#include <vi-map/vi-map.h>

namespace loop_closure_plugin {

DEFINE_bool(
    lc_only_against_other_missions, false,
    "If true, no inter-mission loop-closures are sought.");

DEFINE_bool(
    lc_against_cumulative_map, false,
    "Cumulate all landmarks simultaneously into the database and query "
    "against that. Only to be used as a refinement step at the end, because "
    "otherwise any misalignments will introduce bad associations.");

VIMapMerger::VIMapMerger(
    vi_map::VIMap* map, const visualization::ViwlsGraphRvizPlotter* plotter)
    : map_(map), plotter_(plotter) {
  CHECK_NOTNULL(map_);
}

int VIMapMerger::findLoopClosuresBetweenAllMissions() {
  if (map_->numMissions() == 0u) {
    LOG(ERROR) << "No missions in database.";
    return kNoData;
  }

  vi_map::MissionIdList mission_ids;
  map_->getAllMissionIds(&mission_ids);

  return findLoopClosuresBetweenMissions(mission_ids);
}

int VIMapMerger::findLoopClosuresBetweenFirstAndOtherMissions() {
  if (map_->numMissions() == 0u) {
    LOG(ERROR) << "No missions in database.";
    return kNoData;
  }

  vi_map::MissionIdList mission_ids;
  map_->getAllMissionIds(&mission_ids);

  return findLoopClosuresBetweenFirstAndOtherMissions(mission_ids);
}

int VIMapMerger::findLoopClosuresBetweenMissions(
    const vi_map::MissionIdList& mission_ids) {
  VLOG(1) << "Trying to find loop-closures in and between "
          << mission_ids.size() << " missions.";

  if (mission_ids.empty()) {
    LOG(ERROR) << "There are no missions in the loaded map. Aborting.";
    return common::kUnknownError;
  }

  // TODO: (michbaum) We don't want to do this
  if (FLAGS_lc_against_cumulative_map) {
    CHECK(!FLAGS_lc_only_against_other_missions)
        << "When using a cumulative map all the missions are included in the "
           "query database and queried against. They can not be separated.";

    VLOG(1) << "Loop closing against cumulative map" << std::endl;

    // Create joint global map of landmarks for queries
    loop_detector_node::LoopDetectorNode loop_detector;
    if (plotter_ != nullptr) {
      loop_detector.instantiateVisualizer();
    }

    for (vi_map::MissionIdList::const_iterator it = mission_ids.begin();
         it != mission_ids.end(); ++it) {
      CHECK(it->isValid());
      loop_detector.addMissionToDatabase(*it, *map_);
    }

    // Attempt to merge landmarks from each mission against the entire database
    for (vi_map::MissionIdList::const_iterator it = mission_ids.begin();
         it != mission_ids.end(); ++it) {
      loop_detector.detectLoopClosuresAndMergeLandmarks(*it, map_);
    }
  } else {
    // TODO: (michbaum) We want to use this branch, but need to change the logic to only loop close the benchmark missions against the to-be-evaluated mission
    // We want to try to loop close every mission pair
    // This is the basic way for a single map lc
    VLOG(1) << "Loop closing between all missions" << std::endl;
    // TODO: (ehosko) Outer loop not needed - only loop against first map
    for (vi_map::MissionIdList::const_iterator it = mission_ids.begin();
         it != mission_ids.end(); ++it) {
      CHECK(it->isValid());
      loop_detector_node::LoopDetectorNode loop_detector;
      if (plotter_ != nullptr) {
        loop_detector.instantiateVisualizer();
      }
      loop_detector.addMissionToDatabase(*it, *map_);
      for (vi_map::MissionIdList::const_iterator jt = mission_ids.begin();
           jt != mission_ids.end(); ++jt) {
        if (FLAGS_lc_only_against_other_missions && *jt == *it) {
          continue;
        }
        // TODO: (michbaum) might need to not merge landmarks for the eval pipeline
        loop_detector.detectLoopClosuresAndMergeLandmarks(*jt, map_);
      }
    }
  }

  return common::kSuccess;
}

int VIMapMerger::findLoopClosuresBetweenFirstAndOtherMissions(
    const vi_map::MissionIdList& mission_ids) {
  VLOG(1) << "Trying to find loop-closures in and between "
          << mission_ids.size() << " missions.";

  if (mission_ids.empty()) {
    LOG(ERROR) << "There are no missions in the loaded map. Aborting.";
    return common::kUnknownError;
  }

  VLOG(1) << "Loop closing between first against other missions" << std::endl;
  // TODO: (ehosko) Outer loop not needed - only loop against first map
  vi_map::MissionIdList::const_iterator it = mission_ids.begin();
  CHECK(it->isValid());
  loop_detector_node::LoopDetectorNode loop_detector;
  if (plotter_ != nullptr) {
    loop_detector.instantiateVisualizer();
  }
  loop_detector.addMissionToDatabase(*it, *map_);
  loop_detector.outputFile = "/home/michbaum/Projects/maplab/data/loopclosure/test3.csv";
  // Create and open csv file for output
  std::ofstream outputStream;
  //std::string driftlogfile_ = "/home/michbaum/Projects/maplab/data/loopclosure/test2.csv";
  outputStream.open(loop_detector.outputFile.c_str());
  if (!outputStream.is_open())
  {
    LOG(INFO) << "Failed to open log file";
  }
  else
  {
    outputStream << "ratio\n";
    outputStream.close();
  }
  for (vi_map::MissionIdList::const_iterator jt = mission_ids.begin() + 1;
        jt != mission_ids.end(); ++jt) {
    if (FLAGS_lc_only_against_other_missions && *jt == *it) {
      continue;
    }
    // TODO: (michbaum) might need to not merge landmarks for the eval pipeline
    loop_detector.detectLoopClosuresAndMergeLandmarks(*jt, map_);
  }


  return common::kSuccess;
}

}  // namespace loop_closure_plugin
