#ifndef RE_LOCALIZATION_EVALUATION_PLUGIN_H_
#define RE_LOCALIZATION_EVALUATION_PLUGIN_H_

#include <string>

#include <console-common/console-plugin-base-with-plotter.h>

namespace common {
class Console;
}  // namespace common

namespace visualization {
class ViwlsGraphRvizPlotter;
}  // namespace visualization

namespace re_localization_evaluation_plugin {
class ReLocalizationEvaluationPlugin : public common::ConsolePluginBaseWithPlotter {
 public:
  ReLocalizationEvaluationPlugin(
      common::Console* console, visualization::ViwlsGraphRvizPlotter* plotter);

  virtual std::string getPluginId() const {
    return "relocalization-evaluation";
  }

 private:
  int evaluateReLocalizationForAllBenchmarkMissions() const;
  int evaluateReLocalizationForOneBenchmarkMission() const;
};
}  // namespace re_localization_evaluation_plugin

#endif  // RE_LOCALIZATION_EVALUATION_PLUGIN_H_
