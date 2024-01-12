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

int VIMapLocalizer::reLocalizeAccuracyAllMissions(const std::string selected_map_key){

  if (map_->numMissions() == 0u) {
  LOG(ERROR) << "No missions in database.";
  return kNoData;
  }

  vi_map::MissionIdList mission_ids;
  // Make sure that the to-be-evaluated-map is the first here
  map_->getAllMissionIds(&mission_ids);

  return reLocalizeAccuracyOneMission(mission_ids, selected_map_key);
}

int VIMapLocalizer::reLocalizeAccuracyOneMission(
  const vi_map::MissionIdList& mission_ids, const std::string selected_map_key) {

  if (mission_ids.empty()) {
  LOG(ERROR) << "There are no missions in the loaded map. Aborting.";
  return common::kUnknownError;
  }

  if (FLAGS_evaluate_mission.empty()) {
    LOG(ERROR) << "Specify a valid mission with -evaluate_mission.";
    return common::kStupidUserError;
  }

  if(FLAGS_log_dir.empty()) {
    LOG(ERROR) << "Specify a valid log directory with -log_dir.";
    return common::kStupidUserError;
  }
  
  std::string filename = FLAGS_log_dir + "/lc_edges.csv";
  std::ofstream logFile(filename, std::ios_base::app);
  if (!logFile.is_open()) {
    LOG(ERROR) << "Could not open file " << filename << " for writing.";
    return common::kUnknownError;
  }

  // Get ID of the to be evaluated map
  vi_map::MissionId evaluate_mission_id;
  map_->ensureMissionIdValid(FLAGS_evaluate_mission, &evaluate_mission_id);
  VLOG(1) << "To be evaluated mission: " << evaluate_mission_id;

  // We don't want to merge landmarks or add loop closure edges
  const bool kAddLoopClosureEdges = true;
  const bool kMergeLandmarks = false;

  // Temp threshold parameter
  double thresh = 1.0;

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

    std::vector<vi_map::Edge::ConstPtr> loop_closure_edges;

    std::vector<std::pair<Eigen::Vector3d, Eigen::Quaterniond>> evaluate_mission_vertices;
    std::vector<std::pair<Eigen::Vector3d, Eigen::Quaterniond>> sample_mission_vertices;

    // loop_detector.detectLoopClosuresAndMergeLandmarks(*jt, map_);
    VLOG(1) << "Localizing mission " << *jt << " w.r.t. mission " << evaluate_mission_id << ".";
    loop_detector.detectLocalizationMissionToDatabase(
      *jt, kMergeLandmarks, kAddLoopClosureEdges, map_, &T_G_M2,
      &inlier_constraints, selected_map_key, &loop_closure_edges);

    int num_loop_closure_edges = loop_closure_edges.size();
    VLOG(1) << "Number of loop closure edges: " << loop_closure_edges.size();

    // TODO (ehosko) : need to add aam computation here to get more results for transformation matrix (or reset threshold if possible)
    VLOG(1) << "Transformation Matrix: " << T_G_M2;
    // Align missions
    // List of keyframes in the to-be-evaluated mission (as vertex)
    // List of keyframes in the sample mission (as vertex)
    for(int i = 0; i <num_loop_closure_edges; ++i) {

      const vi_map::Edge::ConstPtr& loop_closure_edge = loop_closure_edges[i];

      // Get the two vertices of the loop closure edge
      const pose_graph::VertexId& vertex_id_1 = loop_closure_edge->from();
      const pose_graph::VertexId& vertex_id_2 = loop_closure_edge->to();

      // Get the two vertices of the loop closure edge
      const vi_map::Vertex& vertex_1 = map_->getVertex(vertex_id_1);
      const vi_map::Vertex& vertex_2 = map_->getVertex(vertex_id_2);

      LOG(INFO) << "Vertex 1: " << vertex_1.getMissionId();
      LOG(INFO) << "Vertex 2: " << vertex_2.getMissionId();

      // Transform into same coordinate system
      Eigen::Vector3d p2_I2 = vertex_2.get_p_M_I();
      Eigen::Quaterniond q2_I2 = vertex_2.get_q_M_I();

      Eigen::Vector3d p2_I1 = T_G_M2 * p2_I2;
      Eigen::Quaterniond q2_I1 = T_G_M2.getEigenQuaternion() * q2_I2;

      Eigen::Vector3d p1_position = vertex_1.get_p_M_I();
      VLOG(1) << "P1: " << p1_position.x() << " " << p1_position.y() << " " << p1_position.z();
      VLOG(1) << "P2 transformed : " << p2_I1.x() << " " << p2_I1.y() << " " << p2_I1.z();
      
      // Compute distance of loop closure edge and remove if above threshold
      double eucl_distance = std::sqrt(std::pow(p1_position.x() - p2_I1.x(), 2) + std::pow(p1_position.y() - p2_I1.y(), 2) + std::pow(p1_position.z() - p2_I1.z(), 2));
      if(eucl_distance > thresh)
      {
          // VLOG(1) << "Removing loop closure edge with distance: " << eucl_distance;
          LOG(INFO) << "Removing loop closure edge with distance: " << eucl_distance;
          loop_closure_edges.erase(loop_closure_edges.begin() + i);

          // Move back one step in the loop to not skip the next edge
          --i;
          --num_loop_closure_edges;
      }
      else
      {
        // Add the vertices to the list of vertices
        evaluate_mission_vertices.push_back(std::make_pair(p1_position, vertex_1.get_q_M_I()));
        sample_mission_vertices.push_back(std::make_pair(p2_I1, q2_I1));

        logFile << *jt << "," << p1_position.x() << "," << p1_position.y() << "," << p1_position.z() << ","  
                              << p2_I1.x() << "," << p2_I1.y() << "," << p2_I1.z() << "\n";
      }
    } // For loop

    logFile.close();

    // Align trajectories with rpg_trajectory_evaluation with remaining edges
    // Create list/vector of poses for the two missions as input for the alignment

    // Compare new poses with ground truth poses

    // Write results to csv file
  }


  return common::kSuccess;
}

}  // namespace loop_closure_plugin
