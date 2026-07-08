#include <Eigen/Cholesky>
#include <iterator>
#include <iostream>
#include <sstream>
#include "cticp_.h"
#include "utils/geometry_.h"



namespace kd_slam {
  namespace cticp {
    using namespace kd_slam::icp;

    template <typename Traits_>
    void CTICP_<Traits_>::updateCache() {
      _cache.Xm  = moving_state.X;
      _cache.Xf  = fixed_state.X;
      _cache.iXf = _cache.Xf.inverse();
      _cache.Xmf = _cache.iXf * _cache.Xm;
    }

    template <typename Traits_>
    void CTICP_<Traits_>::buildQuadraticForm(bool stats_mode) {
      updateCache();
      _buildQuadraticForm(stats_mode);
    }

    template <typename Base_>
    void CTICP_<Base_>::buildQuadraticForm(FixedEntryBase& fixed_, bool stats_mode) {
      FixedEntry& fixed=static_cast<FixedEntry&>(fixed_);
      this->_fixed=fixed.fixed_tree;
      fixed_state=fixed.fixed_state;
      updateCache();
      _buildQuadraticForm(stats_mode);
      fixed.stats=this->stats;
      fixed.H=H;
      fixed.b=b;
    }


    template <typename Traits_>
    void CTICP_<Traits_>::buildQuadraticFormDual(bool stats_mode) {
      updateCache();
      _buildQuadraticFormDual(stats_mode);
    }

    template <typename Traits_>
    bool CTICP_<Traits_>::oneRound(bool stats_mode) {
      using namespace std;
      if (_fixed_forest.empty()) {
        buildQuadraticForm(stats_mode);
      } else {
        HessianType temp_H=HessianType::Zero();
        CoefficientType temp_b=CoefficientType::Zero();
        StatsType temp_stats=StatsType::Zero();
        for(auto [ref, e]: _fixed_forest) {
          FixedEntry& entry = static_cast<FixedEntry&>(*e);
          buildQuadraticForm(entry, stats_mode);
          temp_H+=entry.H;
          temp_b+=entry.b;
          temp_stats+=entry.stats;
        }
        H=temp_H;
        b=temp_b;
        this->stats=temp_stats;
      }
      AlignerBaseType::posePriorBuildQuadraticForm();
      H.template block<PerturbationPoseDim,PerturbationPoseDim>(0,0)+=AlignerBaseType::_pose_prior_H;
      b.template block<PerturbationPoseDim, 1>(0,0)+=AlignerBaseType::_pose_prior_b;
      int good_measurements=this->stats.num_outliers+this->stats.num_inliers;
      if (good_measurements<this->params.min_good_measurements) {
        //cerr << "good: " << good_measurements << endl;
        return false;
      }
 
      
      HessianType damping=HessianType::Zero();
      int k=0;
      for (int i=0; i<PerturbationPoseDim; ++i, ++k)
        damping(k,k)=this->params.damping;
      for (int i=0; i<VelocityDim; ++i, ++k)
        damping(k,k)=this->params.velocity_damping;

      dx = (H + damping).ldlt().solve(-b);
      this->stats.pert_pose_norm=dx.template head<PerturbationPoseDim>().norm();
      this->stats.pert_vel_norm=dx.template tail<VelocityDim>().norm();
      if (stats_mode)
        return true;
      moving_state.X           = Traits::expmap(dx.template head<PerturbationPoseDim>()) * moving_state.X;
      moving_state.velocities += dx.template tail<VelocityDim>();
      return true;
    }

      template <typename Traits_>
        std::ostream& CTICP_<Traits_>::printStatus(std::ostream& os) {
        using namespace std;
        os << this->stats << endl;
        os << "H:\n" << H << endl;
        os << "b:   " << b.transpose() << endl;
        os << "dx:  " << dx.transpose() << endl;
        os << "vel: " << moving_state.velocities.transpose() << endl;
        return os;
      }

      template <typename Traits_>
        __CUDA_EXPORT_INLINE__
        void CTICP_<Traits_>::quadraticTerm(
                                            typename CTICP_<Traits_>::DestView& dest,
                                            int tid,
                                            const typename CTICP_<Traits_>::State& fixed_state,
                                            const typename CTICP_<Traits_>::State& moving_state,
                                            const typename CTICP_<Traits_>::QuadraticTermCache& cache,
                                            const typename CTICP_<Traits_>::NodeType* fixed_nodes_ptr,
                                            const typename CTICP_<Traits_>::NodeType& moving_leaf,
                                            const typename CTICP_<Traits_>::ParamsType& params,
                                            bool stats_mode) {

        dest.clear(tid);

        // Motion-compensate the moving leaf (in robot frame)
        const NodeType warped_leaf = Traits_::applyVelocities(moving_state.velocities, moving_leaf);

        // Transform warped point to fixed frame for KD-tree search
        const auto p_mif = applyIsometry_(cache.Xmf, warped_leaf._mean);

        int aux_idx = 0;
        while (aux_idx >= 0 && !fixed_nodes_ptr[aux_idx].isLeaf()) {
          const auto& aux = fixed_nodes_ptr[aux_idx];
          aux_idx = aux.whichSide(p_mif) ? aux._idx_left : aux._idx_right;
        }
        if (aux_idx < 0) { dest.stats[tid].num_bad = 1; return; }

        // Fixed leaf coordinates in fixed-sensor-local frame
        const auto& pf_local = fixed_nodes_ptr[aux_idx]._mean;
        const auto& nf_local = fixed_nodes_ptr[aux_idx]._direction;

        // Bring both clouds to global (world) frame for the error computation.
        const auto p_moving = applyIsometry_(cache.Xm, warped_leaf._mean);
        const auto n_moving = applyRotation_ (cache.Xm, warped_leaf._direction);
        const auto p_fixed  = applyIsometry_(cache.Xf, pf_local);
        const auto n_fixed  = applyRotation_ (cache.Xf, nf_local);

        // Pose Jacobian w.r.t. moving robot-pose perturbation v2t(dx)*X_m.
        typename Traits_::Scalar e;
        typename Traits_::JacobianPoseType J_pose;
        int result = Traits_::errorAndJacobian(e, J_pose,
                                               p_fixed, n_fixed,
                                               p_moving, n_moving,
                                               params.min_normal_cos,
                                               params.max_error);
        switch (result) {
        case -1: dest.stats[tid].num_err_normal = 1; return;
        case -2: dest.stats[tid].num_err_fn     = 1; return;
        default: break;
        }

        // Velocity Jacobian. Velocity acts in robot frame; Ta = Xm = X_m.
        const Scalar dt = moving_leaf._dts_mean;
        typename Traits_::Scalar e_vel;
        typename Traits_::JacobianVelocityType J_vel;
        J_vel.setZero();
        Traits_::errorAndJacobianVelocityA(e_vel, J_vel,
                                           cache.Xm,
                                           moving_leaf._mean, moving_leaf._direction,
                                           p_fixed, n_fixed,
                                           moving_state.velocities, dt);

        Scalar chi2  = e * e;
        Scalar gamma = 1;
        if (chi2 > params.kernel_threshold) {
          dest.stats[tid].num_outliers = 1;
          gamma = stats_mode ? 0.f : sqrt(params.kernel_threshold/chi2);
        } else {
          dest.stats[tid].num_inliers = 1;
        }

        dest.stats[tid].pack(chi2, gamma);
        dest.diag_pose[tid].pack(e, J_pose, gamma);
        dest.diag_vel[tid].pack(e, J_vel, gamma);
        dest.cross[tid].pack(J_pose, J_vel, gamma);
      }

      template <typename Traits_>
        __CUDA_EXPORT_INLINE__
        void CTICP_<Traits_>::quadraticTermDual(
                                                typename CTICP_<Traits_>::DestViewDual& dest,
                                                int tid,
                                                const typename CTICP_<Traits_>::State& fixed_state,
                                                const typename CTICP_<Traits_>::State& moving_state,
                                                const typename CTICP_<Traits_>::QuadraticTermCache& cache,
                                                const typename CTICP_<Traits_>::NodeType* fixed_nodes_ptr,
                                                const typename CTICP_<Traits_>::NodeType& moving_leaf,
                                                const typename CTICP_<Traits_>::ParamsType& params,
                                            bool stats_mode) {

        dest.clear(tid);

        // -- same tree-search and error computation as quadraticTerm -------------
        const NodeType warped_leaf = Traits_::applyVelocities(moving_state.velocities, moving_leaf);
        const auto p_mif = applyIsometry_(cache.Xmf, warped_leaf._mean);

        int aux_idx = 0;
        while (aux_idx >= 0 && !fixed_nodes_ptr[aux_idx].isLeaf()) {
          const auto& aux = fixed_nodes_ptr[aux_idx];
          aux_idx = aux.whichSide(p_mif) ? aux._idx_left : aux._idx_right;
        }
        if (aux_idx < 0) { dest.stats[tid].num_bad = 1; return; }

        const auto& pf_local = fixed_nodes_ptr[aux_idx]._mean;
        const auto& nf_local = fixed_nodes_ptr[aux_idx]._direction;

        const auto p_moving = applyIsometry_(cache.Xm, warped_leaf._mean);
        const auto n_moving = applyRotation_ (cache.Xm, warped_leaf._direction);
        const auto p_fixed  = applyIsometry_(cache.Xf, pf_local);
        const auto n_fixed  = applyRotation_ (cache.Xf, nf_local);

        typename Traits_::Scalar e;
        typename Traits_::JacobianPoseType J_pose;
        int result = Traits_::errorAndJacobian(e, J_pose,
                                               p_fixed, n_fixed,
                                               p_moving, n_moving,
                                               params.min_normal_cos,
                                               params.max_error);
        switch (result) {
        case -1: dest.stats[tid].num_err_normal = 1; return;
        case -2: dest.stats[tid].num_err_fn     = 1; return;
        default: break;
        }

        // A-velocity Jacobian (w.r.t. moving velocity, same as quadraticTerm)
        const Scalar dt_A = moving_leaf._dts_mean;
        typename Traits_::Scalar e_vel;
        typename Traits_::JacobianVelocityType J_vel;
        J_vel.setZero();
        Traits_::errorAndJacobianVelocityA(e_vel, J_vel,
                                           cache.Xm,
                                           moving_leaf._mean, moving_leaf._direction,
                                           p_fixed, n_fixed,
                                           moving_state.velocities, dt_A);

        Scalar chi2  = e * e;
        Scalar gamma = stats_mode ? 0 : 1;
        if (chi2 > params.kernel_threshold) {
          dest.stats[tid].num_outliers = 1;
          gamma = sqrt(params.kernel_threshold / chi2);
        } else {
          dest.stats[tid].num_inliers = 1;
        }

        /*
          H Matrix simmetries

          Pa   Va   Pb   Vb
          Pa   A   B    -A    D  
          Va       C    -B'   E
          Pb             A   -D
          Vb                  F

          With:
          A= J_ap'Jap, 
          B= J_ap'Jav
          C= J_av'Jav
          D= J_ap'Jbv
          E= J_av'Jbv
       


        */
  
        // -- standard A-side packing (identical to quadraticTerm) ----------------
        dest.stats[tid].pack(chi2, gamma);                     
        dest.diag_pose[tid].pack(e, J_pose, gamma);                   //J_A_pose^T * J_A_pose
        dest.diag_vel[tid].pack(e, J_vel, gamma);                     //J_A_vel^T *J_A_vel
        dest.cross[tid].pack(J_pose, J_vel, gamma);                   //J_A_pose^T *J_B_vel

        // -- B-velocity Jacobian --------------------------------------------------
        // J_B_pose = -J_A_pose (symmetric error) -> B-pose blocks are derivable,
        // no separate workspace needed.
        // J_B_vel via errorAndJacobianVelocityB: Tb = Xf = X_f.
        const Scalar dt_B = fixed_nodes_ptr[aux_idx]._dts_mean;
        typename Traits_::Scalar e_vel_B;
        typename Traits_::JacobianVelocityType J_vel_B;
        J_vel_B.setZero();
        Traits_::errorAndJacobianVelocityB(e_vel_B, J_vel_B,
                                           cache.Xf,
                                           p_moving, n_moving,
                                           pf_local, nf_local,
                                           fixed_state.velocities, dt_B);

        dest.diag_vel_B[tid].pack(e, J_vel_B, gamma);
        dest.cross_pose_vel_B[tid].pack(J_pose, J_vel_B, gamma);  // J_A_pose^T J_B_vel
        dest.cross_vel_AB[tid].pack(J_vel, J_vel_B, gamma);        // J_A_vel^T  J_B_vel
      }

    template <typename Traits_>
    std::string  CTICP_<Traits_>::iterationLogHeader() const  {
      using namespace std;
      std::ostringstream os;
      os <<"CTICP" << Traits::Dim << "D| START "
         << " fp: " << Traits_::GeometryTraits::logmap(_saved_start_fixed_state.X).transpose()
         << " fv: " << _saved_start_fixed_state.velocities.transpose()
         << " mp: " << Traits_::GeometryTraits::logmap(_saved_start_moving_state.X).transpose()
         << " mv: " << _saved_start_moving_state.velocities.transpose()
         << " | END "
         << " fp: " << Traits_::GeometryTraits::logmap(fixed_state.X).transpose()
         << " fv: " << fixed_state.velocities.transpose()
         << " mp: " << Traits_::GeometryTraits::logmap(moving_state.X).transpose()
         << " mv: " << moving_state.velocities.transpose();
        return os.str();
    }

    }
  } // namespace kd_slam::cticp
