#include "srrg_boss/serializable.h"
#include "slam.h"
#include "kd_slam/slam/slam_proc_impl_.h"
#include "slam_gravity_prior_factor.h"
#include <iostream>

#include "srrg_solver/solver_core/instances.h"
#include "srrg_solver/variables_and_factors/types_3d/instances.h"
#include "srrg_solver/variables_and_factors/types_common/instances.h"
#include "srrg_solver/solver_core/internals/linear_solvers/instances.h"
#include "typedefs.h"


#include <srrg_solver/solver_core/error_factor_impl.cpp>
#include <srrg_solver/solver_core/measurement_owner_impl.cpp>
#include "kd_slam/map/velocity_prior_factor_impl_.h"
#include "kd_slam/map/multiview_icp_factor_impl_.h"
#include "kd_slam/map/multiview_cticp_factor_impl_.h"
#include "kd_slam/localizer/localizer_proc_impl_.h"
#include "kd_slam/bundler/bundler_proc_impl_.h"
#include "kd_slam/optimizer/optimizer_proc_impl_.h"
#include "kd_slam/motion_model/imu_motion_model_impl_.h"
#include "kd_slam/motion_model/odometry_motion_model_.h"

using namespace std;

namespace kd_slam{
  using namespace srrg2_solver;

  namespace slam {
    template struct BundlerProc_<kd_slam::map::MapOwner_<kd_slam::d3::NodeType>>;
    template struct SLAMProc_<OptimizerProc_<TrackerProc_<kd_slam::map::MapOwner_<kd_slam::d3::NodeType>>>>;
    template struct Localizer_<kd_slam::d3::NodeType>;
  }

  using MultiViewICPFactor3D   = map::MultiViewICPFactor_<d3::NodeType>;
  using MultiViewCTICPFactor3D = map::MultiViewCTICPFactor_<d3::NodeType>;
  using VelocityPriorFactor3D  = slam::SLAM3D::SolverVelocityPriorFactor;
  using ConstVelMotionModel3D  = slam::ConstVelMotionModel_<slam::TrackerProc_<map::MapOwner_<d3::NodeType>>>;
  using IMUMotionModel3D        = slam::IMUMotionModel_<slam::TrackerProc_<map::MapOwner_<d3::NodeType>>>;
  using OdometryMotionModel3D  = slam::OdometryMotionModel_<slam::TrackerProc_<map::MapOwner_<d3::NodeType>>>;
  template struct slam::ConstVelMotionModel_<slam::TrackerProc_<map::MapOwner_<d3::NodeType>>>;
  template struct slam::IMUMotionModel_<slam::TrackerProc_<map::MapOwner_<d3::NodeType>>>;
  template struct slam::OdometryMotionModel_<slam::TrackerProc_<map::MapOwner_<d3::NodeType>>>;
    
  void kd_slam_registerSLAMTypes3D() {
    using namespace slam;
    solver_registerTypes();
    linear_solver_registerTypes();
    variables_and_factors_3d_registerTypes();

    BOSS_REGISTER_CLASS(SLAM3D);
    BOSS_REGISTER_CLASS(Localizer3D);
    BOSS_REGISTER_CLASS(Bundler3D);
    BOSS_REGISTER_CLASS(MultiViewICPFactor3D);
    BOSS_REGISTER_CLASS(MultiViewCTICPFactor3D);
    BOSS_REGISTER_CLASS(VelocityPriorFactor3D);
    BOSS_REGISTER_CLASS(GravityPriorFactor);
    BOSS_REGISTER_CLASS(ConstVelMotionModel3D);
    BOSS_REGISTER_CLASS(IMUMotionModel3D);
    BOSS_REGISTER_CLASS(OdometryMotionModel3D);

  }
}
