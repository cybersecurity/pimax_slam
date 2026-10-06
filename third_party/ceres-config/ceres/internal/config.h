// Ceres 2.1.0 configuration matching the copy compiled into pimax_slam.pi.dll.
// Solver VersionString() in the binary: "2.1.0-eigen-(3.4.0)-no_lapack-eigensparse-no_openmp";
// RTTI shows the full set of SchurEliminator/PartitionedMatrixView specializations; the parallel_for_cxx /
// ThreadTokenProvider code is present (C++ threads); ceres is linked statically into the DLL.
#ifndef CERES_PUBLIC_INTERNAL_CONFIG_H_
#define CERES_PUBLIC_INTERNAL_CONFIG_H_

#define CERES_USE_EIGEN_SPARSE
#define CERES_NO_LAPACK
#define CERES_NO_SUITESPARSE
#define CERES_NO_CXSPARSE
#define CERES_NO_CUDA
#define CERES_NO_ACCELERATE_SPARSE

#if defined(CERES_NO_SUITESPARSE) && defined(CERES_NO_ACCELERATE_SPARSE) && \
    defined(CERES_NO_CXSPARSE) && !defined(CERES_USE_EIGEN_SPARSE)
#define CERES_NO_SPARSE
#endif

#define CERES_USE_CXX_THREADS

#endif  // CERES_PUBLIC_INTERNAL_CONFIG_H_
