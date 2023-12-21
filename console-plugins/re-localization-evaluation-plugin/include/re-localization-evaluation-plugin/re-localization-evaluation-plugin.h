#ifndef RE_LOCALIZATION_EVALUATION_PLUGIN_H_
#define RE_LOCALIZATION_EVALUATION_PLUGIN_H_

#include <string>

#include <console-common/console.h>

namespace common {
class Console;
}  // namespace common

namespace re_localization_evaluation_plugin {
class ReLocalizationEvaluationPlugin : public common::ConsolePluginBase {
 public:
  ReLocalizationEvaluationPlugin(
      common::Console* console);

  virtual std::string getPluginId() const {
    return "relocalization-evaluation";
  }

 private:
  int evaluateReLocalizationForAllBenchmarkMissions() const;
  int evaluateReLocalizationForOneBenchmarkMission() const;
  int evaluateReLocalizationAccuracyForAllBenchmarkMissions() const;
};
}  // namespace re_localization_evaluation_plugin

#endif  // RE_LOCALIZATION_EVALUATION_PLUGIN_H_
