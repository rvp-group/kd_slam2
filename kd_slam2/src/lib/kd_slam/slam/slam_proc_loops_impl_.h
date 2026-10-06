#pragma once

namespace kd_slam {
  namespace slam {
    using namespace kd_slam::frame;

    template <typename T_>
    typename SLAMProc_<T_>::PGOFactorPtr SLAMProc_<T_>::seekLoops() {
      using namespace std;
      if (!_descriptor_matcher)
        return nullptr;

      // 1st pass: spatial chi2 filter
      unordered_map<int, Match> candidates;
      vector<int> allowed_refs;
      EventLoopSearch ev_loop(_floating_frame->ts);
      filterSpatially(candidates, allowed_refs, _slam_params.loop_thresholds);
      if (candidates.empty()) {
        pushEvent(std::make_shared<EventLoopSearch>(ev_loop));
        return nullptr;
      }
      std::unordered_map <int,int> ref2idx;
      ev_loop.matches.resize(candidates.size());
      for (size_t i=0; i<allowed_refs.size(); ++i) {
        int ref=allowed_refs[i];
        ev_loop.matches[i].ref=ref;
        ev_loop.matches[i].chi2_spatial=candidates[ref].chi2_spatial;
        ev_loop.matches[i].match_result=MatchDescriptorFail;
        ref2idx[ref]=i;
      }
      stat_loop_spatial += ref2idx.size();

      
      // 2nd pass: descriptor matching
      using QDescriptors = typename DescriptorType::QDescriptors;
      QDescriptors q = _floating_frame->descriptor.buildQDescriptors();


      using DescMatch = typename DescriptorMatcher::Match;
      vector<DescMatch> desc_matches;
      _descriptor_matcher->match(desc_matches, allowed_refs, q);
      if (desc_matches.empty()) {
        pushEvent(std::make_shared<EventLoopSearch>(ev_loop));
        return nullptr;
      }

      auto rotAngle = [](const auto& R) -> Scalar {
        return std::abs(GeometryTraits::angleFromRotation(R));
      };
      
      AlignerBase& aligner = _loop_aligner ? *_loop_aligner : *_local_aligner;
      Match best;

      for (auto& dm : desc_matches) {
        auto& cand       = candidates[dm.ref_match];
        auto& match_kf   = cand.moving_frame;
        auto& loop_match= ev_loop.matches[ref2idx[dm.ref_match]];
        loop_match.descriptor_distance=dm.dist_match;
        ++stat_loop_desc2floating;

        
        // convention: X_a_b = pose of b in frame a.
        // f=floating, k=current kf, m=match kf
        Match km(_keyframe, match_kf);
        const IsometryType X_k_m_graph = km.poseFromGraph();
        const IsometryType X_f_k = _pose_in_kf.inverse();

        Scalar max_graph_rot = _slam_params.loop_max_orientation_rad;
        IsometryType X_m_f_best; // this will contain the transform to feed icp
        Scalar best_rot = std::numeric_limits<Scalar>::max();
        Scalar best_trans = best_rot;

        for (int fc = 0; fc < DescriptorType::NumAxesCanonizations; ++fc) {
          const IsometryType X_m_f = cand.poseFromDescriptors(0, fc).inverse();
          const IsometryType X_m_k = X_m_f * X_f_k;               // chain via floating
          for (int kc = 0; kc < DescriptorType::NumAxesCanonizations; ++kc) {
            const IsometryType X_k_m = km.poseFromDescriptors(kc);
            
            // global check
            if (rotAngle(X_k_m.linear().transpose() * X_k_m_graph.linear()) > max_graph_rot)
              continue;

            // desc consensus check
            const IsometryType D = X_k_m * X_m_k;
            const Scalar r = rotAngle(D.linear());
            if (r < best_rot) {
              best_rot = r;
              best_trans = D.translation().norm();
              X_m_f_best = X_m_f;
            }
          }
        }
        
        loop_match.rotation_delta = best_rot;
        loop_match.translation_delta = best_trans;
        if (best_rot == std::numeric_limits<Scalar>::max()) {
          loop_match.match_result = MatchLoopGraphFail;
          continue;
        }
        if (best_rot > _slam_params.loop_consensus_max_orientation_rad
            || best_trans > _slam_params.loop_consensus_max_translation) {
          loop_match.match_result = MatchLoopConsensusFail;
          continue;
        }
        ++stat_loop_desc2kf;

        /*
        Match m_ckf_mkf_best(_keyframe, match_kf);
        IsometryType T_desc_mkf_curr_best=IsometryType::Identity();
        loop_match.match_result=MatchLoopGraphFail;
        
        Scalar best_angle = std::numeric_limits<Scalar>::max();
        Scalar best_translation= std::numeric_limits<Scalar>::max();
        for (int fc=0; fc<DescriptorType::NumAxesCanonizations; ++fc) {
          // chain match_kf->current
          IsometryType T_desc_mkf_curr=cand.poseFromDescriptors(0, fc).inverse();

          // chain match_kf ->  current_kf  via descriptors of current
          IsometryType T_desc_mkf_ckf_A = T_desc_mkf_curr * _pose_in_kf.inverse();

          // seek between canonizations the chain T_desc_mkf_ckf via descriptors of current keyframe
          // trying all canonizations
          Match m_ckf_mkf(_keyframe, match_kf);
          for (int c = 0; c < DescriptorType::NumAxesCanonizations; ++c) {
            IsometryType T_desc_ckf_mkf_cand=m_ckf_mkf.poseFromDescriptors(c);
            IsometryType  T_delta_desc=T_desc_ckf_mkf_cand*T_desc_mkf_ckf_A;
            Scalar angle_desc = std::abs(GeometryTraits::angleFromRotation(T_delta_desc.linear()));
            Scalar translation_desc=T_delta_desc.translation().norm();

            Scalar angle_graph = std::abs(GeometryTraits::angleFromRotation(T_desc_ckf_mkf_cand.linear().transpose()*m_ckf_mkf.poseFromGraph().linear()));
            if (angle_graph > _slam_params.loop_max_orientation_rad) {
              continue;
            }
          
            if (angle_desc < best_angle) {
              best_angle = angle_desc;
              best_translation=translation_desc;
              m_ckf_mkf_best.pose=T_desc_ckf_mkf_cand;
              T_desc_mkf_curr_best=T_desc_mkf_curr;
              loop_match.match_result=MatchOk;
            }

          }
      
        }
        ++stat_loop_desc2kf;
        loop_match.rotation_delta=best_angle;
        loop_match.translation_delta=best_translation;
        if (loop_match.match_result!=MatchOk)
          continue;
        
        if (best_angle > _slam_params.loop_consensus_max_orientation_rad
            || best_translation > _slam_params.loop_consensus_max_translation) {
          loop_match.match_result=MatchLoopConsensusFail;
          continue;
        }                  
        */
        
        // icp 1: floating w.r.t match_kf
        Match float_match(match_kf, _floating_frame, X_m_f_best);
        float_match.rematch(aligner, _slam_params.loop_thresholds, KDFactorType::Loop);
        // std::cerr << "FL| ";
        // float_match.print(std::cerr);
        // std::cerr << endl;
        if (float_match.result!=MatchOk) {
          loop_match.match_result=MatchLoopICPFloatFail;
          loop_match.score=float_match.score;
          loop_match.coverage=float_match.coverage;
          loop_match.inlier_ratio=float_match.inlier_ratio;
          continue;
        }
        // --- single ICP: _keyframe (moving) vs matchKF (fixed) ---
        Match kf_match(_keyframe, match_kf, _pose_in_kf*float_match.pose.inverse());
        kf_match.desc_distance = dm.dist_match;
        kf_match.rematch(aligner, _slam_params.factor_thresholds, KDFactorType::Loop);

        ++stat_loop_ICP_tries;

        loop_match.inlier_ratio=kf_match.inlier_ratio;
        loop_match.score=kf_match.score;
        loop_match.coverage=kf_match.coverage;
        loop_match.match_result=kf_match.result;
        // cerr << "loop match  " << MatchLabelResultStr[kf_match.result]
        //      << " inlier_ratio: " << kf_match.inlier_ratio
        //      << " inlier_t:    " << _slam_params.loop_thresholds.min_inlier_ratio
        //      << " score: " << kf_match.score 
        //      << " score_t: " << _slam_params.loop_thresholds.min_score << endl;
        // std::cerr << "KF| ";
        // kf_match.print(std::cerr);
        // std::cerr << endl;

        if (kf_match.result != MatchOk) { //TODO use relaxed thresholds and add a further consensus
          loop_match.match_result=MatchLoopICPKFFail;
          loop_match.score=kf_match.score;
          loop_match.coverage=kf_match.coverage;
          loop_match.inlier_ratio=kf_match.inlier_ratio;
          continue;
        }

        // third consensus: check the ICP chain
        IsometryType icp_error=_pose_in_kf*float_match.pose.inverse()*kf_match.pose.inverse();

        Scalar icp_error_rot = GeometryTraits::angleFromRotation(icp_error.linear());
        Scalar icp_error_trans = icp_error.translation().norm();
        cerr << endl << "LOOP: cons_rot = " << icp_error_rot << " const_trans = " << icp_error_trans ;

        if (icp_error_trans>_slam_params.loop_icp_consensus_max_translation
            || icp_error_rot>_slam_params.loop_icp_consensus_max_orientation_rad) {
          loop_match.match_result=MatchLoopICPConsensusFail;
          cerr << " FAIL" << endl;
          continue;
        } else {
          cerr << " ACCEPT" << endl;
        }
          
        
        ++stat_loop_ICP_OK;

        if (kf_match.score > best.score)
          best = kf_match;
      }

      pushEvent(std::make_shared<EventLoopSearch>(ev_loop));
      if (best.result != MatchOk) 
        return nullptr;
      
      
      if (_map->areConnected(_keyframe->ref(), best.moving_frame->ref())) { 
        //cerr << "illegal closure: " << _keyframe->ref() << " - " << best.moving_frame->ref() << " hops: " << best.moving_frame->hops_from_root << endl;
        return nullptr;
      }
      //cerr << "adding closure: " << _keyframe->ref() << " - " << best.moving_frame->ref() << " hops: " << best.moving_frame->hops_from_root << endl;
      auto f_ptr = makeFactor(best, Loop);

      addFactor(f_ptr);
      return f_ptr;
    }

  } // namespace slam
} // namespace kd_slam
