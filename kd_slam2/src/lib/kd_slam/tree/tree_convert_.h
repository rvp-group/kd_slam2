#pragma once
#include <memory>
namespace kd_slam{
  
  // This is for the dynamic overriding
  // of the conversions see {d2,d3}/tree.{cpp,cu} depending on the loaded .so
  // they are cakked in the derived classes by the copy constructor from base
  // and redirect the necessary conversions based on the installed compute
  // managed in d2/d3 via indirection pointers.
  
  template <typename KDTreeBase_>
  std::shared_ptr<KDTreeBase_> Tree_toCompute(std::shared_ptr<KDTreeBase_>& src, bool is_gpu);

  template <typename KDTreeBase_>
  std::shared_ptr<KDTreeBase_> Tree_toCompute_(std::shared_ptr<KDTreeBase_>& src, bool is_gpu);

  template<typename KDTreeBase_>
  using ToComputeFn_ = std::shared_ptr<KDTreeBase_>(*)(std::shared_ptr<KDTreeBase_>&, bool);

  // rewritten inthe base
  template<typename KDTreeBase_>
  void Tree_copyTo(KDTreeBase_& dest, const KDTreeBase_& src);

  template<typename KDTreeBase_>
  void Tree_copyTo_(KDTreeBase_& dest_, const KDTreeBase_& src_);

  template<typename KDTreeBase_>
  using CopyToFn_ = void(*)(KDTreeBase_&, const KDTreeBase_&);

  
} // namespace kd_slam
