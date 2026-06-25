#include "srrg_boss/serializable.h"
#include "slam.h"
#include "kd_slam/slam/slam_proc_impl_.h"
#include <iostream>
#include "srrg_solver/solver_core/instances.h"
#include "srrg_solver/variables_and_factors/types_2d/instances.h"
#include "srrg_solver/variables_and_factors/types_common/instances.h"
#include "srrg_solver/solver_core/internals/linear_solvers/instances.h"

#include <srrg_solver/solver_core/error_factor_impl.cpp>
#include <srrg_solver/solver_core/measurement_owner_impl.cpp>
#include "kd_slam/map/velocity_prior_factor_impl_.h"
#include "kd_slam/map/multiview_icp_factor_impl_.h"
#include "kd_slam/map/multiview_cticp_factor_impl_.h"
#include "kd_slam/localizer/localizer_proc_impl_.h"
#include "kd_slam/bundler/bundler_proc_impl_.h"
#include "kd_slam/optimizer/optimizer_proc_impl_.h"
#include "kd_slam/motion_model/imu_motion_model_impl_.h"

using namespace std;

namespace kd_slam{
  using namespace srrg2_solver;
  namespace slam {
    template struct BundlerProc_<kd_slam::map::MapOwner_<kd_slam::d2::NodeType>>;
    template struct BundlerProc_<TrackerProc_<kd_slam::map::MapOwner_<kd_slam::d2::NodeType>>>;
    template struct SLAMProc_<BundlerProc_<TrackerProc_<kd_slam::map::MapOwner_<kd_slam::d2::NodeType>>>>;
    template struct Localizer_<kd_slam::d2::NodeType>;
  }

  
  using MultiViewICPFactor2D   = map::MultiViewICPFactor_<d2::NodeType>;
  using MultiViewCTICPFactor2D = map::MultiViewCTICPFactor_<d2::NodeType>;
  using VelocityPriorFactor2D  = slam::SLAM2D::SolverVelocityPriorFactor;
  using ConstVelMotionModel2D  = slam::ConstVelMotionModel_<slam::TrackerProc_<map::MapOwner_<d2::NodeType>>>;
  using IMUMotionModel2D       = slam::IMUMotionModel_<slam::TrackerProc_<map::MapOwner_<d2::NodeType>>>;
  template struct slam::ConstVelMotionModel_<slam::TrackerProc_<map::MapOwner_<d2::NodeType>>>;
  template struct slam::IMUMotionModel_<slam::TrackerProc_<map::MapOwner_<d2::NodeType>>>;

  void kd_slam_registerSLAMTypes2D() {
    using namespace slam;
    solver_registerTypes();
    linear_solver_registerTypes();
    variables_and_factors_2d_registerTypes();
    
    BOSS_REGISTER_CLASS(SLAM2D);
    BOSS_REGISTER_CLASS(Localizer2D);
    BOSS_REGISTER_CLASS(Bundler2D);
    BOSS_REGISTER_CLASS(MultiViewICPFactor2D);
    BOSS_REGISTER_CLASS(MultiViewCTICPFactor2D);
    BOSS_REGISTER_CLASS(VelocityPriorFactor2D);
    BOSS_REGISTER_CLASS(ConstVelMotionModel2D);
    BOSS_REGISTER_CLASS(IMUMotionModel2D);
  }
}
