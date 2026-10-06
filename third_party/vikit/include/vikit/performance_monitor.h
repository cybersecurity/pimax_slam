// Reconstructed from pimax_slam.pi.dll (chunk c18_tail_vikit).
// Upstream: rpg_svo_pro_open/vikit/vikit_common/include/vikit/performance_monitor.h
//
// Pimax changes:
//  * std::map -> std::unordered_map (std::hash<std::string> = FNV-1a, 0xCBF29CE484222325 /
//    0x100000001B3 inlined in log/startTimer/stopTimer; node sizes 0x48 (Timer) / 0x40 (LogItem)).
//  * init()/trace()/traceHeader() no longer throw when the trace file cannot be opened
//    (strings "Could not open tracefile." and "Performance monitor not correctly initialized" are
//    absent from the image).
//
// Layout, sizeof(PerformanceMonitor) == 0x1C8 (operator new(0x1C8) at 0x180015DC0 / 0x1800DFDF0):
//   +0    std::unordered_map<std::string, Timer>   timers_   (64 B: maxload f32 @0, list @8/16,
//                                                              buckets vector @24, mask @48, maxidx @56)
//   +64   std::unordered_map<std::string, LogItem> logs_
//   +128  std::string trace_name_
//   +160  std::string trace_dir_
//   +192  std::ofstream ofs_   (filebuf @+200, _Myfile @+328; ios_base fmtflags @ +192+vbase+24,
//                               precision @ +192+vbase+32)
// Users: global g_permon (frontend, qword_18047DDA0) and Estimator member @+1224 (backend).
#ifndef VIKIT_PERFORMANCE_MONITOR_H
#define VIKIT_PERFORMANCE_MONITOR_H

#include <unordered_map>
#include <string>
#include <iostream>
#include <fstream>
#include <vikit/timer.h>

namespace vk
{

struct LogItem
{
  double data;   // +0
  bool   set;    // +8
};

class PerformanceMonitor
{
public:
  PerformanceMonitor();
  ~PerformanceMonitor();
  void init(const std::string& trace_name, const std::string& trace_dir);
  void addTimer(const std::string& name);
  void addLog(const std::string& name);
  void writeToFile();
  void startTimer(const std::string& name);
  void stopTimer(const std::string& name);
  double getTime(const std::string& name) const;
  void log(const std::string& name, double data);

private:
  std::unordered_map<std::string, Timer>      timers_;
  std::unordered_map<std::string, LogItem>    logs_;
  std::string                       trace_name_;        //<! name of the thread that started the performance monitor
  std::string                       trace_dir_;         //<! directory where the logfiles are saved
  std::ofstream                     ofs_;

  void trace();
  void traceHeader();
};

} // namespace vk

#endif // VIKIT_PERFORMANCE_MONITOR_H
