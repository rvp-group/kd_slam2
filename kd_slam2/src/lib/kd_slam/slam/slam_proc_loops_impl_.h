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
      

      AlignerBase& aligner = _loop_aligner ? *_loop_aligner : *_local_aligner;
      Match best;

      for (auto& dm : desc_matches) {
        auto& cand       = candidates[dm.ref_match];
        auto& match_kf   = cand.moving_frame;
        auto& loop_match= ev_loop.matches[ref2idx[dm.ref_match]];
        loop_match.descriptor_distance=dm.dist_match;
        ++stat_loop_desc2floating;

        // chain match_kf->current
        IsometryType T_desc_mkf_curr=cand.poseFromDescriptors(0, dm.q_canon); //p_desc_mkf*p_desc_curr.inverse();

        // chain match_kf ->  current_kf  via descriptors of current
        IsometryType T_desc_mkf_ckf_A = T_desc_mkf_curr * _pose_in_kf.inverse();

      // seek between canonizations the chain T_desc_mkf_ckf via descriptors of current keyframe
        // trying all canonizations
        const auto T_desc_ckf_mkf_A=T_desc_mkf_ckf_A.inverse();
        Match m_ckf_mkf(_keyframe, match_kf);
        Scalar best_angle = std::numeric_limits<Scalar>::max();
        Scalar best_translation= std::numeric_limits<Scalar>::max();
        for (int c = 0; c < DescriptorType::NumAxesCanonizations; ++c) {
          IsometryType T_desc_ckf_mkf_cand=m_ckf_mkf.poseFromDescriptors(c);
          IsometryType  T_delta_desc=T_desc_ckf_mkf_A*T_desc_ckf_mkf_cand;
          Scalar angle_desc = std::abs(GeometryTraits::angleFromRotation(T_delta_desc.linear()));
          Scalar translation_desc=T_delta_desc.translation().norm();

          
          if (angle_desc < best_angle) {
            best_angle = angle_desc;
            best_translation=translation_desc;
          }
          
        }
        loop_match.rotation_delta=best_angle;
        loop_match.translation_delta=best_translation;
        
        if (best_angle > _slam_params.loop_consensus_max_orientation_rad
            || best_translation > _slam_params.loop_consensus_max_translation) {
          continue;
        }
        loop_match.match_result=MatchLoopConsensusFail;
        ++stat_loop_desc2kf;
      
        // --- check against graph estimate ---
        Scalar angle_graph = std::abs(GeometryTraits::angleFromRotation(T_desc_mkf_ckf_A.linear().transpose()*cand.pose.linear()));
        if (angle_graph > _slam_params.loop_max_orientation_rad) {
          loop_match.match_result=MatchLoopGraphFail;
        }
        // icp 1: floating w.r.t match_kf
        Match float_match(match_kf, _floating_frame, T_desc_mkf_curr);
        float_match.rematch(aligner, _slam_params.loop_thresholds, KDFactorType::Loop);
        if (float_match.result!=MatchOk)
          continue;
        

        // --- single ICP: _keyframe (moving) vs matchKF (fixed) ---
        Match kf_match(_keyframe, match_kf, _pose_in_kf*float_match.pose.inverse());
        kf_match.desc_distance = dm.dist_match;
        kf_match.rematch(aligner, _slam_params.factor_thresholds, KDFactorType::Loop);

        ++stat_loop_ICP_tries;

        loop_match.inlier_ratio=kf_match.inlier_ratio;
        loop_match.score=kf_match.score;
        loop_match.match_result=kf_match.result;
        // cerr << "loop match  " << MatchLabelResultStr[kf_match.result]
        //      << " inlier_ratio: " << kf_match.inlier_ratio
        //      << " inlier_t:    " << _slam_params.loop_thresholds.min_inlier_ratio
        //      << " score: " << kf_match.score 
        //      << " score_t: " << _slam_params.loop_thresholds.min_score << endl;
        if (kf_match.result != MatchOk) { //TODO use relaxed thresholds and add a further consensus
          continue;
        }
        ++stat_loop_ICP_OK;

        if (kf_match.score > best.score)
          best = kf_match;
      }

      pushEvent(std::make_shared<EventLoopSearch>(ev_loop));
      if (best.result != MatchOk) 
        return nullptr;
      
      auto f_ptr = makeFactor(best, Loop);

      if (_map->areConnected(_keyframe->ref(), best.fixed_frame->ref()))
        return f_ptr;

      addFactor(f_ptr);
      return f_ptr;
    }

  } // namespace slam
} // namespace kd_slam
