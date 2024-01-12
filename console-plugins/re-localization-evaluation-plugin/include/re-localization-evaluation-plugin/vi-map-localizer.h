#ifndef RE_LOCALIZATION_EVALUATION_PLUGIN_VI_MAP_LOCALIZER_H_
#define RE_LOCALIZATION_EVALUATION_PLUGIN_VI_MAP_LOCALIZER_H_

#include <string>

#include <console-common/command-registerer.h>
#include <vi-map/unique-id.h>

namespace vi_map {
class VIMap;
}  // namespace vi_map

namespace re_localization_evaluation_plugin {
class VIMapLocalizer {
 public:
  VIMapLocalizer() = delete;
  VIMapLocalizer(
      vi_map::VIMap* map);
  enum ConsistencyStatus {
    kInconsistent = common::kCustomStatusOffset,
    kNoData,
  };

  int reLocalizeAllMissions(const std::string selected_map_key);
  int reLocalizeOneMission(const vi_map::MissionIdList& mission_ids, const std::string selected_map_key);
  int reLocalizeAccuracyAllMissions(const std::string selected_map_key);
  int reLocalizeAccuracyOneMission(const vi_map::MissionIdList& mission_ids, const std::string selected_map_key);
  

 private:
  vi_map::VIMap* map_;
};
}  // namespace re_localization_evaluation_plugin

#endif  // RE_LOCALIZATION_EVALUATION_PLUGIN_VI_MAP_LOCALIZER_H_
