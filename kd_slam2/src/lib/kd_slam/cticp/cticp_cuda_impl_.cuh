#include "cticp_cuda_.h"
#include "cuda/reduce_impl_.cuh"
#include "kd_slam/icp/icp_ws_item_.h"
#include <iostream>


namespace kd_slam {
  namespace cticp {
    using namespace kd_slam::icp;
    using namespace kd_slam::cuda;
    using namespace std;

    // ---- CUDA workspace --------------------------------------------------------
    // CTICP variant: stats + diag_pose + diag_vel + cross arrays
    template<typename CTICP_BaseType_>
    struct CTICP_CUDAWorkspace_ {
      using StatsType         = typename CTICP_BaseType_::StatsType;
      using DiagPoseType      = typename CTICP_BaseType_::DiagPoseType;
      using DiagVelType       = typename CTICP_BaseType_::DiagVelType;
      using CrossType         = typename CTICP_BaseType_::CrossType;
      using CrossVelAVelBType = typename CTICP_BaseType_::CrossVelAVelBType;
      using DestView          = typename CTICP_BaseType_::DestView;
      using DestViewDual      = typename CTICP_BaseType_::DestViewDual;

      // non-dual arrays
      StatsType*    _stats     = nullptr;
      DiagPoseType* _diag_pose = nullptr;
      DiagVelType*  _diag_vel  = nullptr;
      CrossType*    _cross     = nullptr;
      // dual-only arrays (allocated lazily in resizeDual)
      DiagVelType*       _diag_vel_B       = nullptr;
      CrossType*         _cross_pose_vel_B = nullptr;
      CrossVelAVelBType* _cross_vel_AB     = nullptr;

      size_t _capacity      = 0;
      size_t _capacity_dual = 0;

      Reduce_<StatsType,    2, CTICP_BaseType_::Traits::StatsReduceMaxThreadsPerBlock>    _stats_adder;
      Reduce_<DiagPoseType, 2, CTICP_BaseType_::Traits::DiagPoseReduceMaxThreadsPerBlock> _diag_pose_adder;
      Reduce_<DiagVelType,  2, CTICP_BaseType_::Traits::DiagVelReduceMaxThreadsPerBlock>  _diag_vel_adder;
      Reduce_<CrossType,    2, CTICP_BaseType_::Traits::CrossReduceMaxThreadsPerBlock>    _cross_adder;
      // dual reducers (same thread-count limits as their same-type counterparts)
      Reduce_<DiagVelType,       2, CTICP_BaseType_::Traits::DiagVelReduceMaxThreadsPerBlock>  _diag_vel_B_adder;
      Reduce_<CrossType,         2, CTICP_BaseType_::Traits::CrossReduceMaxThreadsPerBlock>    _cross_pose_vel_B_adder;
      Reduce_<CrossVelAVelBType, 2, CTICP_BaseType_::Traits::CrossReduceMaxThreadsPerBlock>    _cross_vel_AB_adder;

      DestView makeView() {
        return DestView{_stats, _diag_pose, _diag_vel, _cross};
      }
      DestViewDual makeViewDual() {
        return DestViewDual{_stats, _diag_pose, _diag_vel, _cross,
                            _diag_vel_B, _cross_pose_vel_B, _cross_vel_AB};
      }

      void resize(int n) {
        if (n > (int)_capacity) {
          if (_stats) {
            CUDA_CHECK(cudaFree(_stats));
            CUDA_CHECK(cudaFree(_diag_pose));
            CUDA_CHECK(cudaFree(_diag_vel));
            CUDA_CHECK(cudaFree(_cross));
          }
          _capacity = 2*n;
          CUDA_CHECK(cudaMalloc((void**)&_stats,     sizeof(StatsType)    * _capacity));
          CUDA_CHECK(cudaMalloc((void**)&_diag_pose, sizeof(DiagPoseType) * _capacity));
          CUDA_CHECK(cudaMalloc((void**)&_diag_vel,  sizeof(DiagVelType)  * _capacity));
          CUDA_CHECK(cudaMalloc((void**)&_cross,     sizeof(CrossType)    * _capacity));
          _stats_adder.init(_capacity);
          _diag_pose_adder.init(_capacity);
          _diag_vel_adder.init(_capacity);
          _cross_adder.init(_capacity);
        }
      }

      void resizeDual(int n) {
        resize(n);  // ensure non-dual arrays are allocated
        if (n > (int)_capacity_dual) {
          if (_diag_vel_B) {
            CUDA_CHECK(cudaFree(_diag_vel_B));
            CUDA_CHECK(cudaFree(_cross_pose_vel_B));
            CUDA_CHECK(cudaFree(_cross_vel_AB));
          }
          _capacity_dual = 2*n;
          CUDA_CHECK(cudaMalloc((void**)&_diag_vel_B,       sizeof(DiagVelType)       * _capacity_dual));
          CUDA_CHECK(cudaMalloc((void**)&_cross_pose_vel_B, sizeof(CrossType)         * _capacity_dual));
          CUDA_CHECK(cudaMalloc((void**)&_cross_vel_AB,     sizeof(CrossVelAVelBType) * _capacity_dual));
          _diag_vel_B_adder.init(_capacity_dual);
          _cross_pose_vel_B_adder.init(_capacity_dual);
          _cross_vel_AB_adder.init(_capacity_dual);
        }
      }

      ~CTICP_CUDAWorkspace_() {
        if (_stats) {
          CUDA_CHECK(cudaFree(_stats));
          CUDA_CHECK(cudaFree(_diag_pose));
          CUDA_CHECK(cudaFree(_diag_vel));
          CUDA_CHECK(cudaFree(_cross));
        }
        if (_diag_vel_B) {
          CUDA_CHECK(cudaFree(_diag_vel_B));
          CUDA_CHECK(cudaFree(_cross_pose_vel_B));
          CUDA_CHECK(cudaFree(_cross_vel_AB));
        }
      }
    };

    // ---- boilerplate -----------------------------------------------------------
    template <typename CTICP_BaseType_>
    CTICP_CUDA_<CTICP_BaseType_>::CTICP_CUDA_() {
      _workspace = new WorkspaceType;
    }

    template <typename CTICP_BaseType_>
    CTICP_CUDA_<CTICP_BaseType_>::~CTICP_CUDA_() {
      if (_workspace) delete _workspace;
    }

    template <typename CTICP_BaseType_>
    void CTICP_CUDA_<CTICP_BaseType_>::setMoving(const TreeBaseType& moving) {
      //ask if the base type has cuda storage. If not throw an exception
      if (! isGPUPtr(moving._nodes_ptr)) {
        throw std::runtime_error("applyVelocitiesInPlace not on device. This is bad");
      }
      this->_moving = &moving;
      _workspace->resize(moving._num_leaves);
    }

    template <typename CTICP_BaseType_>
    void CTICP_CUDA_<CTICP_BaseType_>::setFixed(const TreeBaseType& fixed) {
      //ask if the base type has cuda storage. If not throw an exception
      if (! isGPUPtr(fixed._nodes_ptr)) {
        throw std::runtime_error("applyVelocitiesInPlace not on device. This is bad");
      }
      this->_fixed = &fixed;
    }

    // ---- Jacobian kernel -------------------------------------------------------
    template <typename CTICP_BaseType_>
    __global__
    __launch_bounds__(CTICP_BaseType_::Traits::JacobianMaxThreadsPerBlock)
      void CTICP_kernel(typename CTICP_BaseType_::DestView ws,
                        const typename CTICP_BaseType_::NodeType* fixed_nodes_ptr,
                        size_t fixed_num_nodes,
                        const typename CTICP_BaseType_::NodeType* moving_nodes_ptr,
                        const size_t * moving_leaves_indices_ptr,
                        const size_t moving_num_leaves,
                        const typename CTICP_BaseType_::State fixed_state,
                        const typename CTICP_BaseType_::State moving_state,
                        const typename CTICP_BaseType_::QuadraticTermCache cache,
                        const typename CTICP_BaseType_::ParamsType params,
                        bool stats_mode) {
      int tid = threadIdx.x + blockIdx.x * blockDim.x;
      if (fixed_num_nodes <= 0)          return;
      if (tid >= moving_num_leaves) return;
      int leaf_idx = moving_leaves_indices_ptr[tid];
      const typename CTICP_BaseType_::NodeType& moving_leaf = moving_nodes_ptr[leaf_idx];
      CTICP_BaseType_::quadraticTerm(ws, tid, fixed_state, moving_state, cache,
                                     fixed_nodes_ptr, moving_leaf, params, stats_mode);
    }

    // ---- buildQuadraticForm ----------------------------------------------------
    template <typename CTICP_BaseType_>
    void CTICP_CUDA_<CTICP_BaseType_>::_buildQuadraticForm(bool stats_mode) {
      using StatsType    = typename CTICP_BaseType_::StatsType;
      using DiagPoseType = typename CTICP_BaseType_::DiagPoseType;
      using DiagVelType  = typename CTICP_BaseType_::DiagVelType;
      using CrossType    = typename CTICP_BaseType_::CrossType;
      static constexpr int PDim = CTICP_BaseType_::PerturbationPoseDim;
      static constexpr int VDim = CTICP_BaseType_::VelocityDim;

      const int n         = this->_moving->_num_leaves;
      const int n_threads = CTICP_BaseType_::Traits::JacobianMaxThreadsPerBlock;
      const int n_blocks  = (n + n_threads - 1) / n_threads;
      const KDTreeType* fixed=reinterpret_cast<const KDTreeType*>(this->_fixed);
      const KDTreeType* moving=reinterpret_cast<const KDTreeType*>(this->_moving);

      CTICP_kernel<CTICP_BaseType_><<<n_blocks, n_threads>>>(
                                                             _workspace->makeView(),
                                                             fixed->_nodes_ptr,
                                                             fixed->_num_nodes,
                                                             moving->_nodes_ptr,
                                                             moving->_leaves_indices_ptr,
                                                             moving->_num_leaves,
                                                             this->fixed_state,
                                                             this->moving_state,
                                                             this->_cache,
                                                             this->params,
                                                             stats_mode);
      // reduces run on the same stream: ordered after Jacobian kernel automatically;
      // the cudaMemcpy(D2H) inside the last reduce is the effective sync point
      //  CUDA_CHECK(cudaDeviceSynchronize());
      // CUDA_CHECK(cudaGetLastError());

      _workspace->_stats_adder.template reduceDeferred<OpSum<StatsType>>(_workspace->_stats, n);

      _workspace->_diag_pose_adder.template reduceDeferred<OpSum<DiagPoseType>>(_workspace->_diag_pose, n);

      _workspace->_diag_vel_adder.template reduceDeferred<OpSum<DiagVelType>>(_workspace->_diag_vel, n);
      _workspace->_cross_adder.template reduceDeferred<OpSum<CrossType>>(_workspace->_cross, n);
      //CUDA_CHECK(cudaDeviceSynchronize());
      //CUDA_CHECK(cudaGetLastError());

      this-> stats = _workspace->_stats_adder.getDeferredResult();
      auto diag_pose = _workspace->_diag_pose_adder.getDeferredResult();
      auto diag_vel  = _workspace->_diag_vel_adder.getDeferredResult();
      auto cross     = _workspace->_cross_adder.getDeferredResult();
  
      typename DiagPoseType::HessianType     H_pose; typename DiagPoseType::CoefficientType b_pose;
      typename DiagVelType::HessianType      H_vel;  typename DiagVelType::CoefficientType  b_vel;
      typename CrossType::HessianType        H_cross;

      diag_pose.unpack(H_pose, b_pose);
      diag_vel.unpack(H_vel,   b_vel);
      cross.unpack(H_cross);

      this->H.template block<PDim, PDim>(0,    0)    = H_pose;
      this->H.template block<PDim, VDim>(0,    PDim) = H_cross;
      this->H.template block<VDim, PDim>(PDim, 0)    = H_cross.transpose();
      this->H.template block<VDim, VDim>(PDim, PDim) = H_vel;
      this->b.template head<PDim>() = b_pose;
      this->b.template tail<VDim>() = b_vel;
    }

    // ---- Dual Jacobian kernel --------------------------------------------------
    template <typename CTICP_BaseType_>
    __global__
    __launch_bounds__(CTICP_BaseType_::Traits::JacobianMaxThreadsPerBlock)
      void CTICP_kernel_dual(
                             typename CTICP_BaseType_::DestViewDual ws,
                             const typename CTICP_BaseType_::NodeType* fixed_nodes_ptr,
                             size_t fixed_num_nodes,
                             const typename CTICP_BaseType_::NodeType* moving_nodes_ptr,
                             const size_t * moving_leaves_indices_ptr,
                             const size_t moving_num_leaves,
                             const typename CTICP_BaseType_::State fixed_state,
                             const typename CTICP_BaseType_::State moving_state,
                             const typename CTICP_BaseType_::QuadraticTermCache cache,
                             const typename CTICP_BaseType_::ParamsType params,
                        bool stats_mode) {
      int tid = threadIdx.x + blockIdx.x * blockDim.x;
      if (fixed_num_nodes <= 0)          return;
      if (tid >= moving_num_leaves) return;
      int leaf_idx = moving_leaves_indices_ptr[tid];
      const typename CTICP_BaseType_::NodeType& moving_leaf = moving_nodes_ptr[leaf_idx];
      CTICP_BaseType_::quadraticTermDual(ws, tid, fixed_state, moving_state, cache,
                                         fixed_nodes_ptr, moving_leaf, params, stats_mode);
    }

    // ---- buildQuadraticFormDual ------------------------------------------------
    template <typename CTICP_BaseType_>
    void CTICP_CUDA_<CTICP_BaseType_>::_buildQuadraticFormDual(bool stats_mode) {
      using StatsType         = typename CTICP_BaseType_::StatsType;
      using DiagPoseType      = typename CTICP_BaseType_::DiagPoseType;
      using DiagVelType       = typename CTICP_BaseType_::DiagVelType;
      using CrossType         = typename CTICP_BaseType_::CrossType;
      using CrossVelAVelBType = typename CTICP_BaseType_::CrossVelAVelBType;
      static constexpr int PDim = CTICP_BaseType_::PerturbationPoseDim;
      static constexpr int VDim = CTICP_BaseType_::VelocityDim;
      const KDTreeType* fixed=reinterpret_cast<const KDTreeType*>(this->_fixed);
      const KDTreeType* moving=reinterpret_cast<const KDTreeType*>(this->_moving);

      const int n         = this->_moving->_num_leaves;
      const int n_threads = CTICP_BaseType_::Traits::JacobianMaxThreadsPerBlock;
      const int n_blocks  = (n + n_threads - 1) / n_threads;

      _workspace->resizeDual(n);

      CTICP_kernel_dual<CTICP_BaseType_><<<n_blocks, n_threads>>>(
                                                                  _workspace->makeViewDual(),
                                                                  fixed->_nodes_ptr,
                                                                  fixed->_num_nodes,
                                                                  moving->_nodes_ptr,
                                                                  moving->_leaves_indices_ptr,
                                                                  moving->_num_leaves,
                                                                  this->fixed_state,
                                                                  this->moving_state,
                                                                  this->_cache,
                                                                  this->params,
                                                             stats_mode);

      _workspace->_stats_adder.template             reduceDeferred<OpSum<StatsType>>         (_workspace->_stats,            n);
      _workspace->_diag_pose_adder.template         reduceDeferred<OpSum<DiagPoseType>>      (_workspace->_diag_pose,        n);
      _workspace->_diag_vel_adder.template          reduceDeferred<OpSum<DiagVelType>>        (_workspace->_diag_vel,         n);
      _workspace->_cross_adder.template             reduceDeferred<OpSum<CrossType>>          (_workspace->_cross,            n);
      _workspace->_diag_vel_B_adder.template        reduceDeferred<OpSum<DiagVelType>>        (_workspace->_diag_vel_B,       n);
      _workspace->_cross_pose_vel_B_adder.template  reduceDeferred<OpSum<CrossType>>          (_workspace->_cross_pose_vel_B, n);
      _workspace->_cross_vel_AB_adder.template      reduceDeferred<OpSum<CrossVelAVelBType>>  (_workspace->_cross_vel_AB,     n);

      this->stats = _workspace->_stats_adder.getDeferredResult();
      auto diag_pose        = _workspace->_diag_pose_adder.getDeferredResult();
      auto diag_vel         = _workspace->_diag_vel_adder.getDeferredResult();
      auto cross            = _workspace->_cross_adder.getDeferredResult();
      auto diag_vel_B       = _workspace->_diag_vel_B_adder.getDeferredResult();
      auto cross_pose_vel_B = _workspace->_cross_pose_vel_B_adder.getDeferredResult();
      auto cross_vel_AB     = _workspace->_cross_vel_AB_adder.getDeferredResult();

      typename DiagPoseType::HessianType  H_pose; typename DiagPoseType::CoefficientType b_pose;
      typename DiagVelType::HessianType   H_vel;  typename DiagVelType::CoefficientType  b_vel;
      typename CrossType::HessianType     H_cross_pv;

      diag_pose.unpack(H_pose, b_pose);
      diag_vel.unpack(H_vel, b_vel);
      cross.unpack(H_cross_pv);

      this->H.template block<PDim, PDim>(0,    0)    = H_pose;
      this->H.template block<PDim, VDim>(0,    PDim) = H_cross_pv;
      this->H.template block<VDim, PDim>(PDim, 0)    = H_cross_pv.transpose();
      this->H.template block<VDim, VDim>(PDim, PDim) = H_vel;
      this->b.template head<PDim>() = b_pose;
      this->b.template tail<VDim>() = b_vel;

      diag_vel_B.unpack(this->H_dual_vel_B, this->b_dual_vel_B);
      cross_pose_vel_B.unpack(this->H_dual_cross_pose_vel_B);
      cross_vel_AB.unpack(this->H_dual_cross_vel_AB);
    }

    template <typename CTICP_BaseType_>
    void CTICP_CUDA_<CTICP_BaseType_>::applyVelocitiesInPlace(typename CTICP_BaseType_::TreeBaseType& target,
                                                              const State& s, bool skip_leaves) {
      target.applyVelocitiesInPlace(s.velocities, skip_leaves);
    }


  }
} // namespace kd_slam::cticp
