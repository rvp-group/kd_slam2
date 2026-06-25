#pragma once
#include "kd_slam/map/multiview_icp_factor_.h"
#include "kd_slam/map/multiview_cticp_factor_.h"
#include "kd_slam/map/map_impl_.h"
namespace kd_slam {
  namespace slam {
    using namespace kd_slam::frame;

    //
    template <typename T_>
    typename SLAMProc_<T_>::PGOFactorPtr
    SLAMProc_<T_>::makeFactor(FramePtr& from, FramePtr& to, KDFactorType ft) {
      using namespace std;
      PGOFactor f;
      f.from_ref=from->ref();
      f.to_ref=to->ref();
      f.type =ft;
      auto dX=from->pose_in_world.inverse()*to->pose_in_world;
      int retval = alignFrames(*_local_aligner, from, to, dX, ft);
      f.stats=_local_aligner->stats;
      //cerr << "ADDFACTOR, stats: " << f.stats << endl;
      if (retval<0) {
        cerr << "makeFactor broke TYPE: " << KDFactorTypeStr[ft] << endl;
        return nullptr;
      }
      f.Z_from_to=_local_aligner->getX();
      if (!_local_aligner->getX().matrix().allFinite()){
        cerr << "makeFactor not finite TYPE: " << KDFactorTypeStr[ft] << endl;
        return nullptr;
      }
      f.Z_to_from=f.Z_from_to.inverse();
      f.omega_from_to=_local_aligner->getPoseHessian();
      f.omega_to_from = GeometryTraits::transformOmega(f.omega_from_to, f.Z_from_to);
      f.det_from_to=f.omega_from_to.determinant();
      f.det_to_from=f.omega_to_from.determinant();
      return std::make_shared<PGOFactor>(f);
    }

    template <typename T_>
    typename SLAMProc_<T_>::PGOFactorPtr
    SLAMProc_<T_>::makeFactor(const Match& match, KDFactorType ft) {
      using namespace std;
      // sanity check
      auto f_from=_map->frame(match.fixed_frame->ref());
      auto f_to=_map->frame(match.moving_frame->ref());
      if (! f_from) {
        cerr<< __PRETTY_FUNCTION__ << "frame " << match.fixed_frame->ref() << " not in graph" << endl;
        throw std::runtime_error("male");
      }
      if (! f_to) {
        cerr<< __PRETTY_FUNCTION__ << "frame " << match.moving_frame->ref() << " not in graph" << endl;
        throw std::runtime_error("male");
      }
      PGOFactor f;
      f.from_ref      = match.fixed_frame->ref();
      f.to_ref        = match.moving_frame->ref();
      f.Z_from_to     = match.pose;
      f.Z_to_from     = f.Z_from_to.inverse();
      f.omega_from_to = match.omega;
      f.omega_to_from = GeometryTraits::transformOmega(f.omega_from_to, f.Z_from_to);
      f.det_from_to   = f.omega_from_to.determinant();
      f.det_to_from   = f.omega_to_from.determinant();
      f.stats         = match.stats;
      f.type          = ft;
      return std::make_shared<PGOFactor>(f);
    }

    
    template <typename T_>
    void SLAMProc_<T_>::addFactor(PGOFactorPtr f) {
      bool result = _map->addFactor(f);
      if (! result)
        return;
      if (_solver) {
        f->solver_factor->compute(true, true);
        _pending_chi2+=f->solver_factor->stats().chi;
        _need_optimize |= (f->type != Odometry);
      }
      pushEvent(std::make_shared<EventFactorAdded>(_floating_frame->ts,
                                                   f->from_ref,
                                                   f->to_ref,
                                                   f->Z_from_to,
                                                   f->omega_from_to,
                                                   f->stats,
                                                   f->type,
                                                   (size_t)f.get()));
    }
    

  }
}// kd_slam::slam
