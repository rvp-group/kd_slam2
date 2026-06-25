#include "icp_cuda_.h"
#include "cuda/reduce_impl_.cuh"
#include "icp_ws_item_.h"
#include <iostream>


namespace kd_slam {
  namespace icp {

    using namespace std;
    using namespace kd_slam::cuda;
    // ---- CUDA workspace --------------------------------------------------------
    // ICP variant: stats + diag_pose arrays
    template<typename ICP_BaseType_>
    struct ICP_CUDAWorkspace_ {
      using StatsType    = typename ICP_BaseType_::StatsType;
      using DiagPoseType = typename ICP_BaseType_::DiagPoseType;
      using DestView     = typename ICP_BaseType_::DestView;

      StatsType*    _stats     = nullptr;
      DiagPoseType* _diag_pose = nullptr;
      size_t _capacity = 0;

      Reduce_<StatsType,    2, ICP_BaseType_::Traits::StatsReduceMaxThreadsPerBlock>    _stats_adder;
      Reduce_<DiagPoseType, 2, ICP_BaseType_::Traits::DiagPoseReduceMaxThreadsPerBlock> _diag_pose_adder;

      DestView makeView() { return DestView{_stats, _diag_pose}; }

      void resize(int n) {
        if (n > (int)_capacity) {
          if (_stats) { CUDA_CHECK(cudaFree(_stats)); CUDA_CHECK(cudaFree(_diag_pose)); }
          _capacity = 2*n;
          CUDA_CHECK(cudaMalloc((void**)&_stats,     sizeof(StatsType)    * _capacity));
          CUDA_CHECK(cudaMalloc((void**)&_diag_pose, sizeof(DiagPoseType) * _capacity));
          _stats_adder.init(_capacity);
          _diag_pose_adder.init(_capacity);
        }
      }

      ~ICP_CUDAWorkspace_() {
        if (_stats) { CUDA_CHECK(cudaFree(_stats)); CUDA_CHECK(cudaFree(_diag_pose)); }
      }
    };

    // ---- boilerplate -----------------------------------------------------------
    template <typename ICP_BaseType_>
    ICP_CUDA_<ICP_BaseType_>::ICP_CUDA_() {
      _workspace = new WorkspaceType;
    }

    template <typename ICP_BaseType_>
    ICP_CUDA_<ICP_BaseType_>::~ICP_CUDA_() {
      if (_workspace) delete _workspace;
    }

    template <typename ICP_BaseType_>
    void ICP_CUDA_<ICP_BaseType_>::setMoving(const TreeBaseType& moving) {
      this->_moving = &moving;
      _workspace->resize(moving._num_leaves);
    }

    template <typename ICP_BaseType_>
    void ICP_CUDA_<ICP_BaseType_>::setFixed(const TreeBaseType& fixed) {
      this->_fixed = &fixed;
    }

    // ---- Jacobian kernel -------------------------------------------------------
    template <typename ICP_BaseType_>
    __global__
    __launch_bounds__(ICP_BaseType_::Traits::DiagPoseReduceMaxThreadsPerBlock)
      void ICP_kernel(
                      typename ICP_BaseType_::DestView ws,

                      const typename ICP_BaseType_::NodeType* fixed_nodes_ptr,
                      size_t fixed_num_nodes,
                      const typename ICP_BaseType_::NodeType* moving_nodes_ptr,
                      const size_t * moving_leaves_indices_ptr,
                      const size_t moving_num_leaves,
    
                      const typename ICP_BaseType_::State fixed_state,
                      const typename ICP_BaseType_::State moving_state,
                      const typename ICP_BaseType_::QuadraticTermCache cache,
                      const typename ICP_BaseType_::ParamsType  params) {
      int tid = threadIdx.x + blockIdx.x * blockDim.x;
      if (fixed_num_nodes <= 0)          return;
      if (tid >= moving_num_leaves) return;
      int leaf_idx = moving_leaves_indices_ptr[tid];
      const typename ICP_BaseType_::NodeType& moving_leaf = moving_nodes_ptr[leaf_idx];
      ICP_BaseType_::quadraticTerm(ws, tid, fixed_state, moving_state, cache, fixed_nodes_ptr, moving_leaf, params);
    }

    // ---- buildQuadraticForm ----------------------------------------------------
    template <typename ICP_BaseType_>
    void ICP_CUDA_<ICP_BaseType_>::_buildQuadraticForm() {
      using StatsType    = typename ICP_BaseType_::StatsType;
      using DiagPoseType = typename ICP_BaseType_::DiagPoseType;

      const int n         = this->_moving->_num_leaves;
      const int n_threads = ICP_BaseType_::Traits::DiagPoseReduceMaxThreadsPerBlock;
      const int n_blocks  = (n + n_threads - 1) / n_threads;
      const KDTreeType* fixed=reinterpret_cast<const KDTreeType*>(this->_fixed);
      const KDTreeType* moving=reinterpret_cast<const KDTreeType*>(this->_moving);
  
      ICP_kernel<ICP_BaseType_><<<n_blocks, n_threads>>>(
                                                         _workspace->makeView(),
                                                         fixed->_nodes_ptr,
                                                         fixed->_num_nodes,
                                                         moving->_nodes_ptr,
                                                         moving->_leaves_indices_ptr,
                                                         moving->_num_leaves,
                                                         this->fixed_state,
                                                         this->moving_state,
                                                         this->_cache,
                                                         this->params);
      // reduces run on the same stream: ordered after Jacobian kernel automatically;
      // the cudaMemcpy(D2H) inside getDeferredResult is the effective sync point
      //CUDA_CHECK(cudaDeviceSynchronize());
      //CUDA_CHECK(cudaGetLastError());

      _workspace->_stats_adder.template reduceDeferred<OpSum<StatsType>>(_workspace->_stats, n);
      _workspace->_diag_pose_adder.template reduceDeferred<OpSum<DiagPoseType>>(_workspace->_diag_pose, n);
      //CUDA_CHECK(cudaDeviceSynchronize());
      //CUDA_CHECK(cudaGetLastError());


      auto diag_pose = _workspace->_diag_pose_adder.getDeferredResult();
      this->stats=_workspace->_stats_adder.getDeferredResult();
  
      diag_pose.unpack(this->H, this->b);
    }

  }
} // namespace kd_slam::icp
