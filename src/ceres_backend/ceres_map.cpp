// pimax_slam.pi.dll -- src/ceres_backend/ceres_map.cpp  (drafts c02 + c01)
//
// Pimax fork of svo_ceres_backend/src/map.cpp (OKVIS map.cpp).
// Original __FILE__: E:\code_codex\pimax_slam\beta111_5a7902_dll\src\ceres_backend\ceres_map.cpp
// Object #2 in link order (TU2).  Map::Map(bool) 0x180018E90, Map::setFlag 0x18001A950 and
// Map::applyFlag 0x18001A980 (draft c01) precede the first __FILE__ user 0x18001B4D0; the rest is
// draft c02.  The GravityLocalParameterization statics plus 0x18001D5D0 / minus 0x18001CF70 and
// TangentBasis 0x18001ACE0 are header-inline (gravity_local_parameterization.hpp) and first emitted
// here.
//
// glog prints the source line of every LOG/VLOG/CHECK; the original line numbers are reproduced
// with #line directives placed directly in front of each logging statement:
//   94/100/104 printParameterBlockInfo, 164/169/201 addParameterBlock, 224 removeParameterBlock,
//   268 addResidualBlock, 382 removeResidualBlock, 511 parameterBlockPtr, 588 parameterBlockIdOfResidual.
//
// Build note: the binary evaluates the right-hand operands of several `LOG(..) << a << f()` chains
// before constructing the LogMessage (C++14 "unsequenced" operand order, MSVC default /std:c++14).
// This has no observable effect except the order of side-effect-free lookups.

#include "ceres_backend/ceres_map.hpp"

#include <glog/logging.h>

#include <memory>
#include <sstream>
#include <tuple>

#include "ceres_backend/estimator_types.hpp"   // BackendId (operator<< prints std::hex id std::dec)

namespace pimax {
namespace totem {
namespace ceres_backend {

// 0x180018E90  upstream-modified: flag argument (copied into the three local parameterizations,
// stores in the order pose +0x530, gravity +0x548, homogeneous point +0x518 after the Problem is
// created), disable_all_safety_checks = true, gravity parameterization, two new-id vectors.
// The binary builds the shared_ptr<Problem> from a unique_ptr (control block
// _Ref_count_resource<ceres::Problem*, default_delete<ceres::Problem>>, null test before allocating
// it), not with shared_ptr::reset(p) (which would use _Ref_count<Problem>).
Map::Map(bool flag)
    : use_minimal_jacobians_(flag), residual_counter_(0)
{
  ceres::Problem::Options problemOptions;
  problemOptions.local_parameterization_ownership =
      ceres::Ownership::DO_NOT_TAKE_OWNERSHIP;
  problemOptions.loss_function_ownership =
      ceres::Ownership::DO_NOT_TAKE_OWNERSHIP;
  problemOptions.cost_function_ownership =
      ceres::Ownership::DO_NOT_TAKE_OWNERSHIP;
  problemOptions.disable_all_safety_checks = true;  // pimax-new (stack byte +0x11 = 1)
  //problemOptions.enable_fast_parameter_block_removal = true;
  problem_ = std::unique_ptr<ceres::Problem>(new ceres::Problem(problemOptions));
  //options.linear_solver_ordering = new ceres::ParameterBlockOrdering;

  pose_local_parameterization_.use_minimal_jacobians_ = flag;
  gravity_local_parameterization_.use_minimal_jacobians_ = flag;
  homogeneous_point_local_parameterization_.use_minimal_jacobians_ = flag;
}

// 0x18001A950  [pimax-new]: forwards to applyFlag(flag); the vector is ignored.
// Only caller: Estimator::optimize 0x18002A6D0 (empty local vector, true), immediately before
// ceres::Solve(options, problem_.get(), &summary).  TODO(verify) name/parameter type.
void Map::setFlag(const std::vector<std::vector<uint64_t> >& /*unused*/, bool flag)
{
  applyFlag(flag);
}

// 0x18001A980  [pimax-new] (name unknown)
// Propagate the flag to every LocalParamizationAdditionalInterfaces of the registered parameter
// blocks and to every ErrorInterface of the registered residual blocks.  If the flag did not
// change, only the blocks added since the previous call are updated.
void Map::applyFlag(bool flag)
{
  if (use_minimal_jacobians_ == flag)
  {
    for (ceres::ResidualBlockId id : new_residual_block_ids_)
    {
      auto it = residual_block_id_to_residual_block_spec_map_.find(id);
      if (it != residual_block_id_to_residual_block_spec_map_.end())
      {
        if (it->second.error_interface_ptr)
        {
          it->second.error_interface_ptr->use_minimal_jacobians_ = flag;  // ErrorInterface +8
        }
      }
    }
    for (uint64_t id : new_parameter_block_ids_)
    {
      auto it = id_to_parameter_block_map_.find(id);
      if (it != id_to_parameter_block_map_.end())
      {
        if (it->second->localParameterizationPtr())  // ParameterBlock vtable slot 10
        {
          auto* p = dynamic_cast<const LocalParamizationAdditionalInterfaces*>(
              it->second->localParameterizationPtr());
          if (p)
          {
            const_cast<LocalParamizationAdditionalInterfaces*>(p)->use_minimal_jacobians_ = flag;
          }
        }
      }
    }
  }
  else
  {
    use_minimal_jacobians_ = flag;
    pose_local_parameterization_.use_minimal_jacobians_ = flag;
    gravity_local_parameterization_.use_minimal_jacobians_ = flag;
    homogeneous_point_local_parameterization_.use_minimal_jacobians_ = flag;
    for (auto& kv : id_to_parameter_block_map_)
    {
      if (kv.second->localParameterizationPtr())
      {
        auto* p = dynamic_cast<const LocalParamizationAdditionalInterfaces*>(
            kv.second->localParameterizationPtr());
        if (p)
        {
          const_cast<LocalParamizationAdditionalInterfaces*>(p)->use_minimal_jacobians_ = flag;
        }
      }
    }
    for (auto& kv : residual_block_id_to_residual_block_spec_map_)
    {
      if (kv.second.error_interface_ptr)
      {
        kv.second.error_interface_ptr->use_minimal_jacobians_ = flag;
      }
    }
  }
  new_residual_block_ids_.clear();
  new_parameter_block_ids_.clear();
}

// 0x18001D360  -- upstream-identical
// Check whether a certain parameter block is part of the map.
bool Map::parameterBlockExists(uint64_t parameter_block_id) const
{
  if (id_to_parameter_block_map_.find(parameter_block_id)
      == id_to_parameter_block_map_.end())
  {
    return false;
  }
  return true;
}

// 0x18001D940  -- upstream-identical (residuals() inlined, kErrorToStr is an unordered_map)
// Log information on a parameter block.
void Map::printParameterBlockInfo(uint64_t parameter_block_id) const
{
  ResidualBlockCollection residualCollection = residuals(parameter_block_id);
#line 94
  LOG(INFO) << "parameter info" << std::endl << "----------------------------"
            << std::endl << " - block Id: " << parameter_block_id << std::endl
            << " - type: " << parameterBlockPtr(parameter_block_id)->typeInfo()
            << std::endl << " - residuals (" << residualCollection.size()
            << "):";
  for (size_t i = 0; i < residualCollection.size(); ++i) {
#line 100
    LOG(INFO)
        << "   - id: "
        << residualCollection.at(i).residual_block_id
        << std::endl
        << "   - type: "
        << kErrorToStr.at(errorInterfacePtr(residualCollection.at(i).residual_block_id)->typeInfo());
  }
#line 104
  LOG(INFO) << "============================";
}

// 0x18001B4D0  -- upstream-modified: extra `Gravity` parameterization (enum value 2, Trivial=3),
//                 parameter taken by const reference, no DEBUG_CHECK(parameter_block != nullptr).
// Add a parameter block to the map
bool Map::addParameterBlock(
    const std::shared_ptr<ceres_backend::ParameterBlock>& parameter_block,
    int parameterization, const int /*group*/)
{
#line 164
  VLOG(200) << "Adding parameter block with parameterization "
            << parameterization << " and id " << BackendId(parameter_block->id());

  // check Id availability
  if (parameterBlockExists(parameter_block->id()))
  {
#line 169
    LOG(ERROR) << "Parameter block with id " << BackendId(parameter_block->id())
               << " exists already!";
    return false;
  }

  id_to_parameter_block_map_.insert(
      std::pair<uint64_t, std::shared_ptr<ceres_backend::ParameterBlock> >(
          parameter_block->id(), parameter_block));

  // also add to ceres problem
  switch (parameterization)
  {
    case Parameterization::Trivial:
    {
      problem_->AddParameterBlock(parameter_block->parameters(),
                                  parameter_block->dimension());
      break;
    }
    case Parameterization::HomogeneousPoint:
    {
      problem_->AddParameterBlock(parameter_block->parameters(),
                                  parameter_block->dimension(),
                                  &homogeneous_point_local_parameterization_);
      parameter_block->setLocalParameterizationPtr(
          &homogeneous_point_local_parameterization_);
      break;
    }
    case Parameterization::Pose6d:
    {
      problem_->AddParameterBlock(parameter_block->parameters(),
                                  parameter_block->dimension(),
                                  &pose_local_parameterization_);
      parameter_block->setLocalParameterizationPtr(&pose_local_parameterization_);
      break;
    }
    case Parameterization::Gravity:
    {
      problem_->AddParameterBlock(parameter_block->parameters(),
                                  parameter_block->dimension(),
                                  &gravity_local_parameterization_);
      parameter_block->setLocalParameterizationPtr(&gravity_local_parameterization_);
      break;
    }
    default:
    {
#line 201
      LOG(ERROR) << "Unknown parameterization!";
      return false;
      break;  // just for consistency...
    }
  }

  return true;
}

// 0x18001E1D0  -- upstream-modified: single find(); the iterator is reused for
//                 RemoveParameterBlock and erase(iterator) (upstream: parameterBlockPtr()+erase(key)).
// Remove a parameter block from the map.
bool Map::removeParameterBlock(uint64_t parameter_block_id)
{
  IdToParameterBlockMap::iterator it = id_to_parameter_block_map_.find(parameter_block_id);
  if (it == id_to_parameter_block_map_.end())
  {
    return false;
  }
#line 224
  VLOG(200) << "Removing paramter block with ID " << BackendId(parameter_block_id);

  // remove all connected residuals
  const ResidualBlockCollection res = residuals(parameter_block_id);
  for (size_t i = 0; i < res.size(); ++i)
  {
    removeResidualBlock(res[i].residual_block_id);  // remove in ceres and book-keeping
  }
  problem_->RemoveParameterBlock(it->second->parameters());  // remove parameter block
  id_to_parameter_block_map_.erase(it);  // remove book-keeping
  return true;
}

// 0x18001B840  -- upstream-modified: reserve() of both temporaries, const-ref range-for in the
//                 VLOG block, no DEBUG_CHECK, and every new id is appended to
//                 new_residual_block_ids_ (+0x550).
// Adds a residual block.
ceres::ResidualBlockId Map::addResidualBlock(
    std::shared_ptr< ceres::CostFunction> cost_function,
    ceres::LossFunction* loss_function,
    std::vector<std::shared_ptr<ceres_backend::ParameterBlock> >& parameter_block_ptrs)
{
  ceres::ResidualBlockId return_id;
  std::vector<double*> parameter_blocks;
  ParameterBlockCollection parameter_block_collection;
  parameter_blocks.reserve(parameter_block_ptrs.size());
  parameter_block_collection.reserve(parameter_block_ptrs.size());
  for (const std::shared_ptr<ceres_backend::ParameterBlock>& parameter_block_ptr :
       parameter_block_ptrs)
  {
    parameter_blocks.push_back(parameter_block_ptr->parameters());
    parameter_block_collection.emplace_back(parameter_block_ptr->id(), parameter_block_ptr);
  }

  // add in ceres
  return_id = problem_->AddResidualBlock(cost_function.get(), loss_function,
                                         parameter_blocks);

  if (FLAGS_v >= 200)
  {
    std::stringstream s;
    s << "Adding residual block: "
      << kErrorToStr.at(std::dynamic_pointer_cast<ceres_backend::ErrorInterface>(cost_function)->typeInfo())
      << " with id " << return_id
      << " connected to the following parameter blocks:\n";
    for (const auto& block : parameter_block_ptrs)
    {
      s << BackendId(block->id()) << "\n";
    }
#line 268
    VLOG(200) << s.str();
  }

  // add in book-keeping
  std::shared_ptr<ErrorInterface> error_interface_ptr =
      std::dynamic_pointer_cast<ErrorInterface>(cost_function);
  residual_block_id_to_residual_block_spec_map_.insert(
      std::pair< ceres::ResidualBlockId, ResidualBlockSpec>(
          return_id,
          ResidualBlockSpec(return_id, loss_function, error_interface_ptr)));

  // update book-keeping
  bool insertion_success;
  std::tie(std::ignore, insertion_success) =
      residual_block_id_to_parameter_block_collection_map_.insert(
          std::make_pair(return_id, parameter_block_collection));
  if (insertion_success == false)
  {
    return ceres::ResidualBlockId(0);
  }

  // update ResidualBlock pointers on involved ParameterBlocks
  for (uint64_t parameter_id = 0;
      parameter_id < parameter_block_collection.size(); ++parameter_id)
  {
    id_to_residual_block_multimap_.insert(
        std::pair<uint64_t, ResidualBlockSpec>(
            parameter_block_collection[parameter_id].first,
            ResidualBlockSpec(return_id, loss_function, error_interface_ptr)));
  }

  new_residual_block_ids_.push_back(return_id);  // [pimax-new]

  return return_id;
}

// 0x18001C370  -- upstream-modified: parameter_block_ptrs.reserve(10); no DEBUG_CHECK.
// Add a residual block. See respective ceres docu. If more are needed, see other interface.
ceres::ResidualBlockId Map::addResidualBlock(
    std::shared_ptr< ceres::CostFunction> cost_function,
    ceres::LossFunction* loss_function,
    std::shared_ptr<ceres_backend::ParameterBlock> x0,
    std::shared_ptr<ceres_backend::ParameterBlock> x1,
    std::shared_ptr<ceres_backend::ParameterBlock> x2,
    std::shared_ptr<ceres_backend::ParameterBlock> x3,
    std::shared_ptr<ceres_backend::ParameterBlock> x4,
    std::shared_ptr<ceres_backend::ParameterBlock> x5,
    std::shared_ptr<ceres_backend::ParameterBlock> x6,
    std::shared_ptr<ceres_backend::ParameterBlock> x7,
    std::shared_ptr<ceres_backend::ParameterBlock> x8,
    std::shared_ptr<ceres_backend::ParameterBlock> x9)
{
  std::vector<std::shared_ptr<ceres_backend::ParameterBlock> > parameter_block_ptrs;
  parameter_block_ptrs.reserve(10);
  if (x0 != 0)
  {
    parameter_block_ptrs.push_back(x0);
  }
  if (x1 != 0)
  {
    parameter_block_ptrs.push_back(x1);
  }
  if (x2 != 0)
  {
    parameter_block_ptrs.push_back(x2);
  }
  if (x3 != 0)
  {
    parameter_block_ptrs.push_back(x3);
  }
  if (x4 != 0)
  {
    parameter_block_ptrs.push_back(x4);
  }
  if (x5 != 0)
  {
    parameter_block_ptrs.push_back(x5);
  }
  if (x6 != 0)
  {
    parameter_block_ptrs.push_back(x6);
  }
  if (x7 != 0)
  {
    parameter_block_ptrs.push_back(x7);
  }
  if (x8 != 0)
  {
    parameter_block_ptrs.push_back(x8);
  }
  if (x9 != 0)
  {
    parameter_block_ptrs.push_back(x9);
  }

  return Map::addResidualBlock(cost_function, loss_function, parameter_block_ptrs);
}

// 0x18001E4C0  -- upstream-modified:
//   * parameter id is taken from the collection entry's .first (upstream: ->second->id()),
//   * a parameter block that occurs k times in the residual is handled once (at its first
//     occurrence) and at most k matching multimap entries are erased,
//   * no DEBUG_CHECK.
// Remove a residual block.
bool Map::removeResidualBlock(ceres::ResidualBlockId residual_block_id)
{
#line 382
  VLOG(200) << "Removing residual block with ID " << residual_block_id;
  problem_->RemoveResidualBlock(residual_block_id);  // remove in ceres

  ResidualBlockIdToParameterBlockCollectionMap::iterator it =
      residual_block_id_to_parameter_block_collection_map_.find(residual_block_id);
  if (it == residual_block_id_to_parameter_block_collection_map_.end())
  {
    return false;
  }

  for (size_t i = 0; i < it->second.size(); ++i)
  {
    const uint64_t parameter_id = it->second[i].first;

    // how often is this parameter block connected to the residual, and did we see it before?
    size_t num_occurrences = 0;
    bool handled_before = false;
    for (size_t j = 0; j < it->second.size(); ++j)
    {
      if (it->second[j].first == parameter_id)
      {
        ++num_occurrences;
        if (j < i)
        {
          handled_before = true;
        }
      }
    }
    if (handled_before)
    {
      continue;
    }

    std::pair<IdToResidualBlockMultimap::iterator,
        IdToResidualBlockMultimap::iterator> range = id_to_residual_block_multimap_
        .equal_range(parameter_id);
    size_t num_erased = 0;
    for (IdToResidualBlockMultimap::iterator it2 = range.first;
        it2 != range.second;)
    {
      if (residual_block_id == it2->second.residual_block_id)
      {
        it2 = id_to_residual_block_multimap_.erase(it2);  // remove book-keeping
        if (++num_erased == num_occurrences)
        {
          break;
        }
      }
      else
      {
        it2++;
      }
    }
  }
  residual_block_id_to_parameter_block_collection_map_.erase(it);  // remove book-keeping
  residual_block_id_to_residual_block_spec_map_.erase(residual_block_id);  // remove book-keeping
  return true;
}

// 0x1800202C0  -- upstream-modified: single find() instead of parameterBlockExists()+find().
// Do not optimise a certain parameter block.
bool Map::setParameterBlockConstant(uint64_t parameter_block_id)
{
  IdToParameterBlockMap::iterator it = id_to_parameter_block_map_.find(parameter_block_id);
  if (it == id_to_parameter_block_map_.end())
  {
    return false;
  }
  std::shared_ptr<ParameterBlock> parameter_block = it->second;
  parameter_block->setFixed(true);
  problem_->SetParameterBlockConstant(parameter_block->parameters());
  return true;
}

// 0x18001CEC0  -- upstream-modified: single find(), the
//                 CHECK_EQ(problem_->IsParameterBlockConstant(..), fixed()) is gone.
bool Map::isParameterBlockConstant(uint64_t parameter_block_id)
{
  IdToParameterBlockMap::iterator it = id_to_parameter_block_map_.find(parameter_block_id);
  if (it == id_to_parameter_block_map_.end())
  {
    return false;
  }
  std::shared_ptr<ParameterBlock> parameter_block = it->second;
  return parameter_block->fixed();
}

// 0x180020380  -- upstream-modified: single find().
// Optimise a certain parameter block (this is the default).
bool Map::setParameterBlockVariable(uint64_t parameter_block_id)
{
  IdToParameterBlockMap::iterator it = id_to_parameter_block_map_.find(parameter_block_id);
  if (it == id_to_parameter_block_map_.end())
  {
    return false;
  }
  std::shared_ptr<ParameterBlock> parameter_block = it->second;
  parameter_block->setFixed(false);
  problem_->SetParameterBlockVariable(parameter_block->parameters());
  return true;
}

// getters
// 0x18001D4A0  -- upstream-modified: single find() + CHECK on the iterator (upstream:
//                 CHECK(parameterBlockExists()) + if(parameterBlockExists()) find()).
// Get a shared pointer to a parameter block.
std::shared_ptr<ceres_backend::ParameterBlock> Map::parameterBlockPtr(
    uint64_t parameter_block_id)
{
  // get a parameterBlock
  IdToParameterBlockMap::iterator it = id_to_parameter_block_map_.find(parameter_block_id);
#line 511
  CHECK(it != id_to_parameter_block_map_.end()) << "parameterBlock with id " << BackendId(parameter_block_id) << " does not exist";
  return it->second;
}

// Not emitted out-of-line (only inlined into printParameterBlockInfo).  upstream-identical.
// Get a shared pointer to a parameter block.
std::shared_ptr<const ceres_backend::ParameterBlock> Map::parameterBlockPtr(
    uint64_t parameter_block_id) const
{
  // get a parameterBlock
  if (parameterBlockExists(parameter_block_id))
  {
    return id_to_parameter_block_map_.find(parameter_block_id)->second;
  }
  return std::shared_ptr<const ceres_backend::ParameterBlock>();  // NULL
}

// 0x18001EA00  -- upstream-modified: implemented through the out-parameter overload.
// Get the residual blocks of a parameter block.
Map::ResidualBlockCollection Map::residuals(uint64_t parameter_block_id) const
{
  ResidualBlockCollection residual_collection;
  residuals(parameter_block_id, residual_collection);
  return residual_collection;
}

// 0x18001EA40  -- pimax-new (body = upstream residuals() without the leading find()).
void Map::residuals(uint64_t parameter_block_id,
                    ResidualBlockCollection& residual_collection) const
{
  residual_collection.clear();
  std::pair<IdToResidualBlockMultimap::const_iterator,
      IdToResidualBlockMultimap::const_iterator> range =
      id_to_residual_block_multimap_.equal_range(parameter_block_id);
  for (IdToResidualBlockMultimap::const_iterator it = range.first;
      it != range.second; ++it)
  {
    residual_collection.push_back(it->second);
  }
}

// 0x18001CD70  -- upstream-identical
// Get a shared pointer to an error term.
std::shared_ptr<const ceres_backend::ErrorInterface> Map::errorInterfacePtr(
    ceres::ResidualBlockId residual_block_id) const
{  // get a vertex
  ResidualBlockIdToResidualBlockSpecMap::const_iterator it =
      residual_block_id_to_residual_block_spec_map_.find(residual_block_id);
  if (it == residual_block_id_to_residual_block_spec_map_.end())
  {
    return std::shared_ptr<ceres_backend::ErrorInterface>();  // NULL
  }
  return it->second.error_interface_ptr;
}

// 0x18001D310  -- pimax-new
const Map::ParameterBlockCollection* Map::parametersPtr(
    ceres::ResidualBlockId residual_block_id) const
{
  ResidualBlockIdToParameterBlockCollectionMap::const_iterator it =
      residual_block_id_to_parameter_block_collection_map_.find(residual_block_id);
  if (it == residual_block_id_to_parameter_block_collection_map_.end())
  {
    return nullptr;
  }
  return &it->second;
}

// 0x18001D3A0  -- pimax-new (parametersPtr() inlined)
// Quirk: on CHECK failure the binary dereferences the (null) collection pointer before
// ~LogMessageFatal runs, i.e. it crashes with an access violation instead of a glog FATAL.
uint64_t Map::parameterBlockIdOfResidual(ceres::ResidualBlockId residual_block_id,
                                         size_t parameter_index) const
{
  const ParameterBlockCollection* parameters = parametersPtr(residual_block_id);
#line 588
  CHECK(parameters != nullptr && parameter_index < parameters->size()) << "Invalid residual parameter index " << parameter_index;
  return (*parameters)[parameter_index].first;
}

}  // namespace ceres_backend
}  // namespace totem
}  // namespace pimax
