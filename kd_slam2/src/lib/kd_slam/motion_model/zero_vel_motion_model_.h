#pragma once
#include <fstream>
namespace kd_slam{
  namespace slam {
    template <typename TrackerType_>
    struct ZeroVelMotionModel_: public srrg2_core::Configurable {
      using TrackerType = TrackerType_;
      using VelocityVectorType = typename TrackerType::VelocityVectorType;
      using PoseHessianType = typename TrackerType::PoseHessianType;
      using Scalar      = typename TrackerType::Scalar;
      using IsometryType = typename TrackerType::IsometryType;
      using Frame        = typename TrackerType::Frame;
      static constexpr int Dim = TrackerType::NodeType::Dim;
      PARAM(srrg2_core::PropertyString, log_file, "file where to log the ",  "",  &_param_changed);
      
      virtual IsometryType prediction()   {return IsometryType::Identity();}
      virtual PoseHessianType predictionCovariance() { return PoseHessianType::Zero();}

      // called by the owner each time a scan is integrated
      // the delta is the motion of floating (before - now). just 1 scan
      virtual void doUpdate(const IsometryType& delta,
                            const PoseHessianType& sigma=PoseHessianType::Zero()) {
        handleLog();
        if (_log_stream) {
          using namespace std;
          VelocityVectorType dv=TrackerType::GeometryTraits::logmap(delta);
          *_log_stream << "UPDATE " << std::fixed << std::setprecision(9) << _last_ts << " "
                       << std::setprecision(4) << dv.transpose() << endl;
        }
      }

      virtual void doPredict(const Frame& frame) {
        if(_last_ts<0) {
          _frame_duration=0;
        } else {
          _frame_duration=frame.ts-_last_ts;
        }
        handleLog();
        if (_log_stream) {
          for (const auto& [ts,imu_entry]: frame.imus) {
            using namespace std;
            *_log_stream << "IMU    " << std::fixed << std::setprecision(9) << ts << " "
                         << std::setprecision(4)
                         << imu_entry.angular_velocity.transpose() << " "
                         << imu_entry.linear_acceleration.transpose() << endl;
          }
        }
        _last_ts=frame.ts;
      }
      virtual void reset() {
        _last_ts=-1;
        _frame_duration=0;
      }
      // called by the owner when  a new keyframe is created
      virtual void onKeyframe(const IsometryType& pose_in_kf = IsometryType::Identity()){} 
      
      // called by the owner when a keyframe is switched
      // that includes
      // - creation
      // - relocalization
      // - loop closure
      virtual void onOriginReset(const IsometryType& pose_in_kf = IsometryType::Identity()){}
      virtual IsometryType    priorXRef()        const { return IsometryType::Identity(); }
      virtual IsometryType    priorZ()           const { return IsometryType::Identity(); }
      virtual PoseHessianType priorOmega()       const { return PoseHessianType::Zero(); }
     
      void setTracker(TrackerType* t) {_tracker=t;}
      double frameDuration()          const { return _frame_duration; }
      virtual VelocityVectorType velocityLog() const {return VelocityVectorType::Zero();}
    protected:
      void handleLog() {
        using namespace std;
        if (! _param_changed)
          return;
        if (_prev_log_stream_fname==param_log_file.value())
          return;
        _log_stream.reset(nullptr);
        _prev_log_stream_fname=param_log_file.value();
        if (_prev_log_stream_fname.empty())
          return;
        _log_stream.reset(new std::ofstream(_prev_log_stream_fname));
        *_log_stream << "#TAG ts <data>" << endl;
        *_log_stream << "#IMU    ts gx gy gz ax ay az " << endl;
        *_log_stream << "#UPDATE ts dx dy dz wx wy wz " << " # this is the delta since last update in logmap" << endl;
      }
      bool _param_changed=false;
      std::string _prev_log_stream_fname="";
      double _frame_duration=0;
      double _last_ts=-1;
      TrackerType* _tracker=nullptr;
      std::unique_ptr<std::ofstream> _log_stream=nullptr;
    };
  }
}
