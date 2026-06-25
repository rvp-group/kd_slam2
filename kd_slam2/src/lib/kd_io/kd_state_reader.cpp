#include "kd_state_reader_.h"
#include "kd_slam/d2/typedefs.h"
#include "kd_slam/d3/typedefs.h"
#include <iostream>

namespace kd_slam {

  // ---- readIsometry --------------------------------------------------------
  template <typename NodeType_>
  typename KDStateReader_<NodeType_>::IsometryType
  KDStateReader_<NodeType_>::_readIsometry(std::istringstream& ss) {
    IsometryType T = IsometryType::Identity();
    if constexpr (Dim == 3) {
      float tx, ty, tz, qx, qy, qz, qw;
      ss >> tx >> ty >> tz >> qx >> qy >> qz >> qw;
      T.translation() = Eigen::Vector3f(tx, ty, tz);
      T.linear() = Eigen::Quaternionf(qw, qx, qy, qz).toRotationMatrix();
    } else {
      float tx, ty, theta;
      ss >> tx >> ty >> theta;
      T.translation() = Eigen::Vector2f(tx, ty);
      T.linear() = Eigen::Rotation2Df(theta).toRotationMatrix();
    }
    return T;
  }

  // ---- readVelocity --------------------------------------------------------
  template <typename NodeType_>
  typename KDStateReader_<NodeType_>::VelocityVectorType
  KDStateReader_<NodeType_>::_readVelocity(std::istringstream& ss) {
    VelocityVectorType v = VelocityVectorType::Zero();
    for (int i = 0; i < v.size(); ++i)
      ss >> v[i];
    return v;
  }

  // ---- open ----------------------------------------------------------------
  template <typename NodeType_>
  bool KDStateReader_<NodeType_>::open(const std::string& filename) {
    std::ifstream f(filename);
    if (!f.good()) {
      std::cerr << "KDStateReader: cannot open " << filename << "\n";
      return false;
    }

    const std::string kf_tag  = "KEYFRAME_"   + std::to_string(Dim);
    const std::string fr_tag  = "FRAME_"      + std::to_string(Dim);
    const std::string fac_tag = "PGO_FACTOR_" + std::to_string(Dim);
    const std::string vel_tag = "VEL_" + std::to_string(Dim);

    std::string line;
    while (std::getline(f, line)) {
      if (line.empty() || line[0] == '#') continue;
      std::istringstream ss(line);
      std::string tag;
      ss >> tag;

      if (tag == vel_tag) {
        VelocityRecord vel_rec;
        ss >> vel_rec.ts >> vel_rec.count;
        vel_rec.velocity=_readVelocity(ss);
        
      } else if (tag == kf_tag) {
        KeyframeRecord r;
        ss >> r.count >> r.kf_idx >> r.ts;
        r.pose_in_kf = _readIsometry(ss);
        std::string tok;
        if (ss >> tok && tok == "|") {
          ss >> tok; // try "T:" or "O:"
        }
        // tok may still hold first token after pose if no "|"
        while (!tok.empty()) {
          if (tok == "T:")      { ss >> r.topic;      tok.clear(); ss >> tok; }
          else if (tok == "O:") { ss >> r.bag_offset; tok.clear(); ss >> tok; }
          else if (tok == "N:") { ss >> r.stamp_ns;   tok.clear(); ss >> tok; }
          else break;
        }
        _keyframe_records[r.kf_idx] = r;

      } else if (tag == fr_tag) {
        FrameRecord r;
        int reason;
        ss >> r.count >> r.kf_ref >> r.ts >> reason;
        r.pose_in_kf = _readIsometry(ss);
        r.reason = static_cast<FrameAddedReason>(reason);
        {
          std::string tok;
          while (ss >> tok) {
            if (tok == "T:")      { ss >> r.topic; }
            else if (tok == "O:") { ss >> r.bag_offset; }
            else if (tok == "N:") { ss >> r.stamp_ns; }
            // "|" V: block not expected for frames, skip gracefully
          }
        }
        _frame_records[r.count] = r;

      } else if (tag == fac_tag) {
        FactorRecord r;
        ss >> r.key;
        int type_int; ss >> type_int;
        r.type = static_cast<KDFactorType>(type_int);
        ss >> r.from_ref >> r.to_ref;
        r.Z = _readIsometry(ss);
        for (int row = 0; row < HDim; ++row)
          for (int col = row; col < HDim; ++col) {
            Scalar v; ss >> v;
            r.omega(row, col) = r.omega(col, row) = v;
          }
        _factor_records.push_back(r);
      }
    }

    std::cerr << "KDStateReader: loaded "
              << _keyframe_records.size() << " keyframes, "
              << _frame_records.size()    << " frames, "
              << _factor_records.size()   << " factors\n";
    return true;
  }

  // ---- replay --------------------------------------------------------------
  template <typename NodeType_>
  void KDStateReplay_<NodeType_>::replay() {
    ICPStats empty{};

    // keyframes: fire EvAdded + EvKF (+ EvVel)
    int prev_kf = -1;
    for (auto& [kf_idx, kf] : _keyframe_records) {
      std::vector<VectorType> leaves;
      if (_loader && !kf.topic.empty() && kf.stamp_ns) {
        auto ft = _loader->getFrame(kf.topic, kf.bag_offset, kf.stamp_ns);
        if (ft && ft->tree)
          leaves = ft->tree->extractCompressedLeaves();
      }
      pushEvent(std::make_shared<EvAdded>(
        kf.ts, kf.count, -1,
        IsometryType::Identity(),
        leaves,
        FrameAddedReplay,
        kf.topic, kf.bag_offset, kf.stamp_ns));
      pushEvent(std::make_shared<EvKF>(
        kf.ts, kf_idx, prev_kf, kf.pose_in_kf, empty));
      prev_kf = kf_idx;
    }

    // factors
    for (auto& fac : _factor_records)
      pushEvent(std::make_shared<EvFact>(
        0.0, fac.from_ref, fac.to_ref,
        fac.Z, fac.omega, empty, fac.type, fac.key));

    // non-keyframe frames: fire EvAdded + EvOdom
    for (auto& [count, fr] : _frame_records) {
      if (fr.kf_ref < 0) continue; // keyframes already handled
      IsometryType pose_global = IsometryType::Identity();
      auto it = _keyframe_records.find(fr.kf_ref);
      if (it != _keyframe_records.end())
        pose_global = it->second.pose_in_kf * fr.pose_in_kf;
      pushEvent(std::make_shared<EvAdded>(
        fr.ts, fr.count, fr.kf_ref,
        fr.pose_in_kf,
        std::vector<VectorType>{},
        fr.reason,
        fr.topic, fr.bag_offset, fr.stamp_ns));
      pushEvent(std::make_shared<EvOdom>(
        fr.ts, fr.pose_in_kf, pose_global, empty));
    }

    // one final proc + done
    pushEvent(std::make_shared<EvProc>(Tracking, 0.0, 0.0, 0.0, 0.0));
    setDone();
  }

  template struct KDStateReader_<kd_slam::d2::NodeType>;
  template struct KDStateReader_<kd_slam::d3::NodeType>;
  template struct KDStateReplay_<kd_slam::d2::NodeType>;
  template struct KDStateReplay_<kd_slam::d3::NodeType>;

} // namespace kd_slam
