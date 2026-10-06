// Reconstructed from pimax_slam.pi.dll (chunk c18_tail_vikit), object performance_monitor.obj
// (0x1801B4200 .. 0x1801B5A77). Upstream: rpg_svo_pro_open/vikit/vikit_common/src/performance_monitor.cpp
//
// Library template instantiations emitted in this object (not reconstructed):
//   0x1801B4200 / 0x1801B42A0  std::list<pair<const string,LogItem/Timer>>::_Free_non_head
//   0x1801B4340                _Hash<..., string, LogItem>::emplace(pair<string,LogItem>&&)  (node 0x40)
//   0x1801B4650                _Hash<..., string, Timer>::emplace(pair<string,Timer>&&)      (node 0x48)
//   0x1801B4B30 / 0x1801B4BB0  _Hash<...LogItem/Timer>::~_Hash (bucket vector + list)
//   0x1801B4C30 / 0x1801B4CC0  _List_node_emplace_op2<...>::~_List_node_emplace_op2
//   0x1801B4D50 / 0x1801B4D80  std::list<...>::~list
//   0x1801B4DB0 / 0x1801B4DC0  unordered_map<...LogItem/Timer>::~unordered_map (thunks)
//   0x1801B5070                std::string::c_str()
// Timer::start()/stop() use std::chrono::steady_clock::now() (QueryPerformanceCounter with the
// MSVC 10 MHz fast path: 100*ctr, else 1e9*(ctr/freq) + 1e9*(ctr%freq)/freq).
#include <vikit/performance_monitor.h>
#include <stdio.h>
#include <stdexcept>

namespace vk
{
using namespace std;

// 0x1801B4970  -- upstream-modified only through the container type (unordered_map default ctors:
// max_load_factor 1.0f, 8 buckets / mask 7)
PerformanceMonitor::PerformanceMonitor()
{}

// 0x1801B4DD0  -- upstream-identical
PerformanceMonitor::~PerformanceMonitor()
{
  ofs_.flush();
  ofs_.close();
}

// 0x1801B5080  -- upstream-modified: no throw when the file cannot be opened; only the printf.
// traceHeader() is inlined (only reached when the file is open).
void PerformanceMonitor::init(
    const string& trace_name,
    const string& trace_dir)
{
  trace_name_ = trace_name;
  trace_dir_ = trace_dir;
  string filename(trace_dir + "/" + trace_name + ".csv");
  ofs_.open(filename.c_str());
  if(!ofs_.is_open())
  {
    printf("Tracefile = %s\n", filename.c_str());
    return;   // Pimax: upstream throws runtime_error("Could not open tracefile.")
  }
  traceHeader();
}

// 0x1801B4F90  -- upstream-identical (Timer() captures steady_clock::now(), duration/accumulated 0)
void PerformanceMonitor::addTimer(const string& name)
{
  timers_.insert(make_pair(name, Timer()));
}

// 0x1801B4EF0  -- upstream-identical (LogItem() value-initialised: data = 0.0, set = false)
void PerformanceMonitor::addLog(const string& name)
{
  logs_.insert(make_pair(name, LogItem()));
}

// 0x1801B58E0  -- upstream-identical apart from the inlined trace() not throwing
void PerformanceMonitor::writeToFile()
{
  trace();

  for(auto it = timers_.begin(); it!=timers_.end(); ++it)
    it->second.reset();
  for(auto it=logs_.begin(); it!=logs_.end(); ++it)
  {
    it->second.set = false;
    it->second.data = -1;
  }
}

// 0x1801B56D0  -- upstream-identical
void PerformanceMonitor::startTimer(const string& name)
{
  auto t = timers_.find(name);
  if(t == timers_.end()) {
    printf("Timer = %s\n", name.c_str());
    throw std::runtime_error("startTimer: Timer not registered");
  }
  t->second.start();
}

// 0x1801B57D0  -- upstream-identical
void PerformanceMonitor::stopTimer(const string& name)
{
  auto t = timers_.find(name);
  if(t == timers_.end()) {
    printf("Timer = %s\n", name.c_str());
    throw std::runtime_error("stopTimer: Timer not registered");
  }
  t->second.stop();
}

// not present in the image (unreferenced -> removed by /OPT:REF). Upstream body kept.
double PerformanceMonitor::getTime(const string& name) const
{
  auto t = timers_.find(name);
  if(t == timers_.end()) {
    printf("Timer = %s\n", name.c_str());
    throw std::runtime_error("Timer not registered");
  }
  return t->second.getTime();
}

// 0x1801B55F0  -- upstream-identical
void PerformanceMonitor::log(const string& name, double data)
{
  auto l = logs_.find(name);
  if(l == logs_.end()) {
    printf("Logger = %s\n", name.c_str());
    throw std::runtime_error("Logger not registered");
  }
  l->second.data = data;
  l->second.set = true;
}

// inlined into writeToFile (0x1801B58E0) -- upstream-modified: silently returns when not open.
// Note first_value is shared between the timer and log loops (as upstream).
void PerformanceMonitor::trace()
{
  //char buffer[128];
  bool first_value = true;
  if(!ofs_.is_open())
    return;   // Pimax: upstream throws "Performance monitor not correctly initialized"
  ofs_.precision(15);
  ofs_.setf(std::ios::fixed, std::ios::floatfield );
  for(auto it = timers_.begin(); it!=timers_.end(); ++it)
  {
    if(first_value) {
      ofs_ << it->second.getTime();
      first_value = false;
    }
    else
      ofs_ << "," << it->second.getTime();
  }
  for(auto it=logs_.begin(); it!=logs_.end(); ++it)
  {
    if(first_value) {
      ofs_ << it->second.data;
      first_value = false;
    }
    else
      ofs_ << "," << it->second.data;
  }
  ofs_ << "\n";
}

// inlined into init (0x1801B5080) -- upstream-modified: silently returns when not open
// (that path is unreachable from init(), so the check folds away).
void PerformanceMonitor::traceHeader()
{
  if(!ofs_.is_open())
    return;   // Pimax: upstream throws "Performance monitor not correctly initialized"
  bool first_value = true;
  for(auto it = timers_.begin(); it!=timers_.end(); ++it)
  {
    if(first_value) {
      ofs_ << it->first;
      first_value = false;
    }
    else
      ofs_ << "," << it->first;
  }
  for(auto it=logs_.begin(); it!=logs_.end(); ++it)
  {
    if(first_value) {
      ofs_ << it->first;
      first_value = false;
    }
    else
      ofs_ << "," << it->first;
  }
  ofs_ << "\n";
}

} // namespace vk
