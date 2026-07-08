#pragma once
#include "map_common.h"
#include "map_owner_.h"
namespace kd_slam {
  namespace map {
    using namespace kd_slam::frame;
    using namespace std;

    template <typename AlignerBase_, typename FramePtr_, typename IsometryType_>
    int alignFrames(AlignerBase_& aligner,
                    FramePtr_& fixed,
                    FramePtr_& moving,
                    const IsometryType_& X0,
                    KDFactorType type,
                    bool stats_mode=false) {
      aligner.setFixed(*fixed->tree);
      aligner.setMoving(*moving->tree);
      aligner.setFixedX(IsometryType_::Identity());
      aligner.setX(X0);
      aligner.setVelocities(moving->velocity);
      int retval=aligner.compute(stats_mode);
      if (aligner.logStream()){
        *aligner.logStream()
          << "SLAM_ALIGN| " << KDFactorTypeStr[type]
          << " fts: " << std::fixed << std::setprecision(5) << fixed->ts
          << " fcnt: " << fixed->frame_count
          << " mts: " << moving->ts
          << " mcnt: " << moving->frame_count
          << " result: " << retval << endl;
      }
      return retval;
    }

    template <typename VectorType_>
    VectorType_ applySlack(const VectorType_& v,
                           typename VectorType_::Scalar slack) {
      using Scalar= typename VectorType_::Scalar;
      Scalar n=v.norm();
      if (n<slack) {
        return VectorType_::Zero();
      }
      return v*(Scalar(1.)-slack/n);
    }
  }
}
