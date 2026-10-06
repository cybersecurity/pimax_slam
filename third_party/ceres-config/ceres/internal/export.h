// Static Ceres build (compiled into pimax_slam.pi.dll).
#ifndef CERES_EXPORT_H
#define CERES_EXPORT_H
#define CERES_EXPORT
#define CERES_NO_EXPORT
#define CERES_DEPRECATED __declspec(deprecated)
#define CERES_DEPRECATED_EXPORT CERES_EXPORT CERES_DEPRECATED
#define CERES_DEPRECATED_NO_EXPORT CERES_NO_EXPORT CERES_DEPRECATED
#endif
