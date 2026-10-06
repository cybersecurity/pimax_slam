// pimax_slam.pi.dll -- src/ceres_backend/ceres_map.hpp  (drafts c02 + c01)
//
// Pimax fork of svo_ceres_backend/include/svo/ceres_backend/map.hpp
// (OKVIS-derived wrapper around ceres::Problem), namespace pimax::totem::ceres_backend.
//
// Layout verified from Map::Map(bool) @0x180018E90 (chunk c01), the implicit Map::~Map()
// @0x180024050 and make_shared<Map>() @0x180021FE0 (operator new(0x590) -> sizeof(Map)==0x580).
// Members marked (sure) are confirmed by accesses inside this chunk.
//
// Differences to upstream (see notes/c02_ceres_map.md):
//  * Parameterization enum gained `Gravity` (=2) and `Trivial` moved to 3.
//  * extra GravityLocalParameterization member (+0x538).
//  * every local parameterization carries an extra bool (+16) set from the Map(bool) ctor arg.
//  * two extra std::vector members (+0x550 new residual-block ids, +0x568 ?).
//  * extra bool at +0x3E8 = the map-wide "minimal Jacobians" flag (Map(bool) / applyFlag
//    0x18001A980 propagate it to every ErrorInterface and LocalParamizationAdditionalInterfaces).
//  * residuals() has an out-parameter overload; parametersPtr()/parameterBlockIdOfResidual() added.
//  * no DEBUG_CHECKs anywhere (all upstream DEBUG_CHECK strings are absent from the binary).
#pragma once

#include <cstdint>
#include <memory>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

#include <Eigen/Core>
#include <ceres/ceres.h>

#include "ceres_backend/error_interface.hpp"          // ErrorInterface, ErrorType, kErrorToStr
#include "ceres_backend/parameter_block.hpp"          // ParameterBlock
#include "ceres_backend/homogeneous_point_local_parameterization.hpp"
#include "ceres_backend/pose_local_parameterization.hpp"
#include "ceres_backend/gravity_local_parameterization.hpp"

namespace pimax {
namespace totem {
namespace ceres_backend {

class Map
{
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  // 0x180018E90 (chunk c01). The flag is copied into the bool at +16 of the three local
  // parameterization members.  make_shared<Map>() passes 0.  TODO(verify) name/meaning.
  explicit Map(bool flag = false);

  /// [pimax-new] 0x18001A980: propagate `flag` to every ErrorInterface::use_minimal_jacobians_
  /// and LocalParamizationAdditionalInterfaces::use_minimal_jacobians_; when the flag did not
  /// change only the blocks added since the last call (new_*_ids_) are updated.  Name TODO(verify).
  void applyFlag(bool flag);
  /// [pimax-new] 0x18001A950: forwards to applyFlag(flag); the vector is ignored.  Only caller:
  /// Estimator::optimize (empty local vector, true) right before ceres::Solve.  The argument is a
  /// vector of vectors (B1: Estimator::optimize destroys the local with a loop over 24-byte
  /// elements calling ~vector 0x180019E70).  Name/element type TODO(verify).
  void setFlag(const std::vector<std::vector<uint64_t> >& unused, bool flag);

  /// 32 bytes: id(+0) loss(+8) error_interface_ptr(+16/+24)   (sure)
  struct ResidualBlockSpec
  {
    ResidualBlockSpec()
        : residual_block_id(0),
          loss_function_ptr(0),
          error_interface_ptr(std::shared_ptr<ErrorInterface>())
    {}

    ResidualBlockSpec(ceres::ResidualBlockId residual_block_id,
                      ceres::LossFunction* loss_function_ptr,
                      std::shared_ptr<ErrorInterface> error_interface_ptr)
        : residual_block_id(residual_block_id),
          loss_function_ptr(loss_function_ptr),
          error_interface_ptr(error_interface_ptr)
    {}

    ceres::ResidualBlockId residual_block_id;
    ceres::LossFunction* loss_function_ptr;
    std::shared_ptr<ErrorInterface> error_interface_ptr;
  };

  /// 24 bytes: id(+0) shared_ptr(+8/+16)   (sure)
  typedef std::pair<uint64_t, std::shared_ptr<ceres_backend::ParameterBlock> > ParameterBlockSpec;
  typedef std::vector<ResidualBlockSpec> ResidualBlockCollection;
  typedef std::vector<ParameterBlockSpec> ParameterBlockCollection;

  /// Values verified from the switch in addParameterBlock (0x18001B4D0) and its callers
  /// (addStates passes 1 for poses, 2 for gravity, 3 for speed&bias / General3D landmarks).
  enum Parameterization
  {
    HomogeneousPoint = 0,  ///< homogeneous_point_local_parameterization_ (+0x508)
    Pose6d = 1,            ///< pose_local_parameterization_ (+0x520)
    Gravity = 2,           ///< gravity_local_parameterization_ (+0x538)   [pimax-new]
    Trivial = 3            ///< no local parameterization
  };

  // ---- this chunk ---------------------------------------------------------------------------
  bool parameterBlockExists(uint64_t parameter_block_id) const;               // 0x18001D360
  void printParameterBlockInfo(uint64_t parameter_block_id) const;            // 0x18001D940
  // Pimax takes the block by const reference (upstream: by value): the callee never releases
  // it, callers build a converted temporary and destroy it after the call.
  bool addParameterBlock(const std::shared_ptr<ceres_backend::ParameterBlock>& parameter_block,
                         int parameterization = Parameterization::Trivial,
                         const int group = -1);                                 // 0x18001B4D0
  bool removeParameterBlock(uint64_t parameter_block_id);                     // 0x18001E1D0
  bool removeParameterBlock(std::shared_ptr<ceres_backend::ParameterBlock> parameter_block)
  {
    return removeParameterBlock(parameter_block->id());
  }
  ceres::ResidualBlockId addResidualBlock(
      std::shared_ptr<ceres::CostFunction> cost_function,
      ceres::LossFunction* loss_function,
      std::vector<std::shared_ptr<ceres_backend::ParameterBlock> >& parameter_block_ptrs);  // 0x18001B840
  ceres::ResidualBlockId addResidualBlock(
      std::shared_ptr<ceres::CostFunction> cost_function,
      ceres::LossFunction* loss_function,
      std::shared_ptr<ceres_backend::ParameterBlock> x0,
      std::shared_ptr<ceres_backend::ParameterBlock> x1 = std::shared_ptr<ceres_backend::ParameterBlock>(),
      std::shared_ptr<ceres_backend::ParameterBlock> x2 = std::shared_ptr<ceres_backend::ParameterBlock>(),
      std::shared_ptr<ceres_backend::ParameterBlock> x3 = std::shared_ptr<ceres_backend::ParameterBlock>(),
      std::shared_ptr<ceres_backend::ParameterBlock> x4 = std::shared_ptr<ceres_backend::ParameterBlock>(),
      std::shared_ptr<ceres_backend::ParameterBlock> x5 = std::shared_ptr<ceres_backend::ParameterBlock>(),
      std::shared_ptr<ceres_backend::ParameterBlock> x6 = std::shared_ptr<ceres_backend::ParameterBlock>(),
      std::shared_ptr<ceres_backend::ParameterBlock> x7 = std::shared_ptr<ceres_backend::ParameterBlock>(),
      std::shared_ptr<ceres_backend::ParameterBlock> x8 = std::shared_ptr<ceres_backend::ParameterBlock>(),
      std::shared_ptr<ceres_backend::ParameterBlock> x9 = std::shared_ptr<ceres_backend::ParameterBlock>());  // 0x18001C370
  bool removeResidualBlock(ceres::ResidualBlockId id);                        // 0x18001E4C0
  bool setParameterBlockConstant(uint64_t parameter_block_id);                // 0x1800202C0
  bool isParameterBlockConstant(uint64_t parameter_block_id);                 // 0x18001CEC0
  bool setParameterBlockVariable(uint64_t parameter_block_id);                // 0x180020380
  bool setParameterBlockConstant(std::shared_ptr<ceres_backend::ParameterBlock> parameter_block)
  {
    return setParameterBlockConstant(parameter_block->id());
  }
  bool isParameterBlockConstant(std::shared_ptr<ceres_backend::ParameterBlock> parameter_block)
  {
    return isParameterBlockConstant(parameter_block->id());
  }
  bool setParameterBlockVariable(std::shared_ptr<ceres_backend::ParameterBlock> parameter_block)
  {
    return setParameterBlockVariable(parameter_block->id());
  }

  std::shared_ptr<ceres_backend::ParameterBlock> parameterBlockPtr(
      uint64_t parameter_block_id);                                           // 0x18001D4A0
  std::shared_ptr<const ceres_backend::ParameterBlock> parameterBlockPtr(
      uint64_t parameter_block_id) const;                                     // inlined only
  std::shared_ptr<const ceres_backend::ErrorInterface> errorInterfacePtr(
      ceres::ResidualBlockId residual_block_id) const;                        // 0x18001CD70

  ResidualBlockCollection residuals(uint64_t parameter_block_id) const;      // 0x18001EA00
  void residuals(uint64_t parameter_block_id,
                 ResidualBlockCollection& residual_collection) const;       // 0x18001EA40 [pimax-new]

  /// [pimax-new] nullptr if the residual block is unknown.   0x18001D310
  /// TODO(verify) real name (used by MarginalizationError @0x180079D90:
  ///   CHECK(parameters != nullptr) << "Residual block not found").
  const ParameterBlockCollection* parametersPtr(ceres::ResidualBlockId residual_block_id) const;
  /// [pimax-new] id of the parameter_index-th parameter block of a residual.  0x18001D3A0
  /// TODO(verify) real name.
  uint64_t parameterBlockIdOfResidual(ceres::ResidualBlockId residual_block_id,
                                      size_t parameter_index) const;

  // ---- other chunks (declared for completeness; upstream API) ------------------------------
  ParameterBlockCollection parameters(ceres::ResidualBlockId residual_block_id) const;
  std::shared_ptr<ceres_backend::ErrorInterface> errorInterfacePtr(
      ceres::ResidualBlockId residual_block_id);

  typedef std::unordered_map<uint64_t, std::shared_ptr<ceres_backend::ParameterBlock> >
      IdToParameterBlockMap;
  typedef std::unordered_map<ceres::ResidualBlockId, ResidualBlockSpec>
      ResidualBlockIdToResidualBlockSpecMap;

  const IdToParameterBlockMap& idToParameterBlockMap() const
  {
    return id_to_parameter_block_map_;
  }
  const ResidualBlockIdToResidualBlockSpecMap& residualBlockIdToResidualBlockSpecMap() const
  {
    return residual_block_id_to_residual_block_spec_map_;
  }

  // public, as upstream
  ceres::Solver::Options options;            // +0x000 (dtor 0x18001A0B0 called on +0)
  ceres::Solver::Summary summary;            // +0x1F0 (dtor 0x18001A2B0 called on +0x1F0)
  bool use_minimal_jacobians_;               // +0x3E8 [pimax-new] ctor arg; see applyFlag  TODO(verify) name

  void solve()
  {
    Solve(options, problem_.get(), &summary);
  }

 public:  // Pimax: Estimator::optimize reads id_to_residual_block_multimap_.count(id) directly (c03)
  uint64_t residual_counter_;                // +0x3F0 (by elimination)
  std::shared_ptr<ceres::Problem> problem_;  // +0x3F8 (sure)

  typedef std::unordered_multimap<uint64_t, ResidualBlockSpec> IdToResidualBlockMultimap;
  typedef std::unordered_map<ceres::ResidualBlockId, ParameterBlockCollection>
      ResidualBlockIdToParameterBlockCollectionMap;

  IdToParameterBlockMap id_to_parameter_block_map_;                                  // +0x408 (sure)
  ResidualBlockIdToResidualBlockSpecMap residual_block_id_to_residual_block_spec_map_; // +0x448 (sure)
  IdToResidualBlockMultimap id_to_residual_block_multimap_;                          // +0x488 (sure)
  ResidualBlockIdToParameterBlockCollectionMap
      residual_block_id_to_parameter_block_collection_map_;                          // +0x4C8 (sure)

  ceres_backend::HomogeneousPointLocalParameterization
      homogeneous_point_local_parameterization_;                                     // +0x508 (sure)
  ceres_backend::PoseLocalParameterization pose_local_parameterization_;             // +0x520 (sure)
  ceres_backend::GravityLocalParameterization gravity_local_parameterization_;       // +0x538 (sure)

  /// [pimax-new] every id returned by addResidualBlock is appended here (0x18001B840);
  /// consumed/cleared by 0x18001A980 (c01).  TODO(verify) name.
  std::vector<ceres::ResidualBlockId> new_residual_block_ids_;                       // +0x550 (sure)
  /// [pimax-new] 8-byte elements, consumed/cleared by 0x18001A980 together with +0x550
  /// (looked up in id_to_parameter_block_map_ -> parameter block ids).  Not written in this chunk.
  std::vector<uint64_t> new_parameter_block_ids_;                                    // +0x568

  friend struct MapLayoutCheck;
};

struct MapLayoutCheck
{
  static_assert(sizeof(Map) == 0x580, "");
  static_assert(offsetof(Map, options) == 0x000, "");
  static_assert(offsetof(Map, summary) == 0x1F0, "");
  static_assert(offsetof(Map, use_minimal_jacobians_) == 0x3E8, "");
  static_assert(offsetof(Map, residual_counter_) == 0x3F0, "");
  static_assert(offsetof(Map, problem_) == 0x3F8, "");
  static_assert(offsetof(Map, id_to_parameter_block_map_) == 0x408, "");
  static_assert(offsetof(Map, residual_block_id_to_residual_block_spec_map_) == 0x448, "");
  static_assert(offsetof(Map, id_to_residual_block_multimap_) == 0x488, "");
  static_assert(offsetof(Map, residual_block_id_to_parameter_block_collection_map_) == 0x4C8, "");
  static_assert(offsetof(Map, homogeneous_point_local_parameterization_) == 0x508, "");
  static_assert(offsetof(Map, pose_local_parameterization_) == 0x520, "");
  static_assert(offsetof(Map, gravity_local_parameterization_) == 0x538, "");
  static_assert(offsetof(Map, new_residual_block_ids_) == 0x550, "");
  static_assert(offsetof(Map, new_parameter_block_ids_) == 0x568, "");
};

}  // namespace ceres_backend
}  // namespace totem
}  // namespace pimax
