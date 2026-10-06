// pimax_slam.pi.dll -- src/frontend/global.h
//
// Upstream svo/global.h (the part that survives): the global performance monitor and the
// SVO_*_TIMER / SVO_LOG convenience macros.  (Drafts c07/c09/c10 referenced "frontend/global.h" or
// declared g_permon locally.)
//   g_permon: std::shared_ptr<vk::PerformanceMonitor> at 0x18047DDA0 (ctrl 0x18047DDA8),
//   constant-initialised, defined in frame_processor_base.cpp (draft c00
//   frame_processor_base_globals.cpp); created in the FrameProcessorBase ctor when
//   options_.trace_statistics is set.
//   vk::PerformanceMonitor: startTimer 0x1801B56D0, stopTimer 0x1801B57D0, log 0x1801B55F0.
#pragma once

#include <memory>

#include <vikit/performance_monitor.h>

namespace pimax {
namespace totem {

using PerformanceMonitorPtr = std::shared_ptr<vk::PerformanceMonitor>;
extern PerformanceMonitorPtr g_permon;

}  // namespace totem
}  // namespace pimax

#define SVO_START_TIMER(name) ::pimax::totem::g_permon->startTimer((name))
#define SVO_STOP_TIMER(name)  ::pimax::totem::g_permon->stopTimer((name))
#define SVO_LOG(name, value)  ::pimax::totem::g_permon->log((name), (value))
