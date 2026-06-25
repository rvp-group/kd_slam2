#include "eval.h"
#include "file_utils.h"

static const std::vector<float> PERCENTAGES = {0.01, 0.02, 0.03, 0.05, 0.08, 0.13, 0.21, 0.34, 0.55};

void computeATE(ResultMap& dest, const Trajectory& gt, const Trajectory& ev) {
  for (const auto& evp: ev) {
    const auto& ts=evp.first;
    const auto& ev_pose=evp.second;
    auto result=gt.getPoseByStamp(ts);
    if  (! result.first)
      continue;
    const auto& gt_pose=result.second;
    dest[ts]=poseError(gt_pose, ev_pose);
  }
}

void computeRPE(ResultMap& dest, const Trajectory& gt, const Trajectory& ev){
  // for each sample in ev
  const double length = gt.length();
  for (const auto& evp: ev) {
    const auto& ev_ts_start=evp.first;
    // seek for a match in gt
    auto result_gt=gt.getPoseByStamp(ev_ts_start);
    if  (! result_gt.first)
      continue;
    
    const auto& ev_pose_start=evp.second;
    const auto& gt_pose_start=result_gt.second;

    Isometry3f ev_pose_start_inv=ev_pose_start.inverse();
    Isometry3f gt_pose_start_inv=gt_pose_start.inverse();

    // seek for a fair pose on gt to
    // start computing the stretches along the trajectory
    double gt_distance_start=gt.ts2distance(ev_ts_start);
    if (gt_distance_start<0)
      continue;

    //cerr << "ts0: " << ev_ts_start << " d0: " << gt_distance_start;

    //cerr << "start distance: " << gt_distance_start << endl;
    int num_samples=0;
    Vector2f error=Vector2f::Zero();
    for (const auto& perc: PERCENTAGES) {

      // each stretch is length*perc
      double gt_target_distance=gt_distance_start+length*perc;

      // retrieve ts in gt around the ev point
      double gt_ts_end=gt.distance2ts(gt_target_distance);
      // out of bounds
      if (gt_ts_end<0)
        continue;

      // ignore corner cases
      gt_target_distance=gt.ts2distance(gt_ts_end);
      if (gt_target_distance<0)
        continue;

      // compute real chunk length
      double chunk_length=gt_target_distance-gt_distance_start;
      // ignore 0 stretches
      if (chunk_length<=0)
        continue;
      
      // we seek in ev for a valid start point immediately
      // before or at the same time of  gt_ts_end
      auto ev_end_it=ev.getValidSample(gt_ts_end);
      
      // ignore corner cases
      if (ev_end_it==ev.end())
        continue;

      double ev_ts_end=ev_end_it->first;
      // ok we have two nice evaluation samples in ev
      // be sure they are not the same
      if (ev_ts_start>=ev_ts_end)
        continue;

      auto ev_end_pose=ev_end_it->second;
      auto gt_end_result=gt.getPoseByStamp(ev_ts_end);
      if (! gt_end_result.first)
        continue;

      //cerr << "\t %: " << perc << " ts: " << ev_ts_end << " d: " << chunk_length << endl;

      auto gt_end_pose=gt_end_result.second;
      error += poseError(gt_pose_start_inv * gt_end_pose,
                         ev_pose_start_inv * ev_end_pose).error * (1./chunk_length);
      ++ num_samples;
    }
    if (num_samples) {
      ResultItem item {error*(1./num_samples), Eigen::Isometry3f::Identity(), Eigen::Isometry3f::Identity()};
      dest.insert(std::make_pair(ev_ts_start, item));
    }
  }
}

void dumpResults(std::ostream& os, const ResultMap& ate, const ResultMap& rpe) {
  os << std::fixed << std::setprecision(5);
  os << "#timestamp ate_r ate_t rpe_r rpe_t gt_x gt_y gt_z gt_qx gt_qy gt_qz gt_qw es_x es_y es_z es_qx es_qy es_qz es_qw" << endl;
  for (const auto& ate_it: ate) {
    double ts=ate_it.first;
    os << ts << " " << ate_it.second.error.transpose() << " ";
    const auto& rpe_it=rpe.find(ts);
    if (rpe_it==rpe.end()) {
      os << "nan nan";
    } else {
      os << rpe_it->second.error.transpose();
    }
    os << " ";
    os << ate_it.second.gt_pose << " ";
    os << ate_it.second.es_pose << " ";
    os << endl;
  }
}

Eigen::Vector2f rmse(const ResultMap& src) {
  Eigen::Vector2d sum=Eigen::Vector2d::Zero();
  for (const auto& s: src) {
    sum+= (s.second).error.cast<double>();
  }
  if (src.size())
    sum *= (1./src.size());
  sum(0)=sqrt(sum(0));
  sum(1)=sqrt(sum(1));
  return sum.cast<float>();
}

Eigen::Vector2f rpeStat(const ResultMap& src) {
  Eigen::Vector2d sum=Eigen::Vector2d::Zero();
  for (const auto& s: src) {
    sum+=s.second.error.cast<double>();
  }
  if (src.size())
    sum *= (1./src.size());
  sum(0)*=100;
  sum(1)*=100;
  return sum.cast<float>();
}

bool eval(Eigen::Vector2f& dest_ate,
          Eigen::Vector2f& dest_rpe,
          const std::string& gt_filename,
          const std::string& ev_filename,
          const std::string& output_path,
          std::ostream& log,
          bool disable_bench,
          bool interpolate_on) {

  string ev_raw_filename=rawFilename(ev_filename);
  if (ev_raw_filename.empty()) {
    log << "error, unable to extract raw filename from [" << ev_filename << "]" << endl;
    return 0;
  }
  log << "EV Raw filename  [" << ev_raw_filename << "] START" << endl;
  
  std::ifstream gt_is(gt_filename);
  log << std::fixed << std::setprecision(5);
  log << "Loading GT from file [" << gt_filename << "]" << endl;
  if(! gt_is) {
    log << "ERROR, aborting" << endl;
    return false;
  }
  Trajectory gt;
  gt.read(gt_is, log);
  gt.enable_interpolation=interpolate_on;
  log << "Read " << gt.size() << " poses" << endl;
  
  std::ifstream ev_is(ev_filename);
  log << "Loading EV from file [" << ev_filename << "]" << endl;
  if(! ev_is) {
    log << "ERROR, aborting" << endl;
    return false;
  }
  Trajectory ev;
  ev.read(ev_is, log);
  log << "Read " << ev.size() << " poses" << endl;

  log << "Computing transform " << endl;
  auto T=ev.computeAlignment(gt, log, false);
  log << T.matrix() << endl;

  log << "Applying transform to ev " << endl;
  ev.applyTransform(T);

  ResultMap ate, rpe;
  log << "Computing ATE" << endl;
  computeATE(ate, gt, ev);
  dest_ate=rmse(ate);
  log << "ATE [R, T]: " << dest_ate.transpose() 
      << " POSES: " << ate.size() << "/" << gt.size() <<endl;
  log << "Computing RPE"  << endl;
  if (! disable_bench && (ate.size()!=gt.size())) {
    log << "BENCH MODE ON and the trajectories do not match. The result will not be considered" << endl;
    log << "SKIPPING" << endl;
    return 0;
  }
  computeRPE(rpe, gt, ev);
  dest_rpe = rpeStat(rpe);
  log << "RPE [R, T]: " << dest_rpe.transpose() 
      << " POSES: " << rpe.size() << "/" << gt.size() <<endl;

  std::string traj_filename=ev_raw_filename+"_traj.txt";
  std::string err_filename=ev_raw_filename+"_err.txt";

  if (!output_path.empty()) {
    traj_filename = output_path + '/' + traj_filename;
    err_filename = output_path + '/' + err_filename;
  }

  log << "Dumping errors trajectory on file [" <<  err_filename << "]" << endl;
  ofstream os_err(err_filename);
  if (! os_err) {
    log << "ERROR, cant create an output for errors on file [" << err_filename << "]" << endl;
    return false;
  }
  dumpResults(os_err, ate, rpe);

    
  log << "Dumping aligned trajectory on file [" << traj_filename << "]" << endl;
  ofstream os_traj(traj_filename);
  if (! os_traj) {
    log << "ERROR, cant create an output for trajectory on file [" << traj_filename << "]" << endl;
    return false;
  }
  ev.write(os_traj);

  log << "geneating plots" << endl;

  std::string title=ev_raw_filename;
  std::replace(title.begin(), title.end(), '_', ' ');
  std::string ate_r_plot=output_path +"/" + ev_raw_filename+"_ate_r.pdf";
  std::string ate_t_plot=output_path +"/" + ev_raw_filename+"_ate_t.pdf"; 
  std::string rpe_r_plot=output_path +"/" + ev_raw_filename+"_rpe_r.pdf";
  std::string rpe_t_plot=output_path +"/" + ev_raw_filename+"_rpe_t.pdf";
  std::string traj_xy_plot=output_path +"/" + ev_raw_filename+"_traj_xy.pdf";
  std::string traj_yz_plot=output_path +"/" + ev_raw_filename+"_traj_yz.pdf";
  std::string traj_xz_plot=output_path +"/" + ev_raw_filename+"_traj_xz.pdf";

  ostringstream gnuplot_command;
  gnuplot_command
    << "gnuplot "
    << "-e \"filetitle='" << title << "'\" "
    << "-e \"err_filename='" << err_filename << "'\" "
    << "-e \"ate_r_plot='" << ate_r_plot << "'\" " 
    << "-e \"ate_t_plot='" << ate_t_plot << "'\" " 
    << "-e \"rpe_r_plot='" << rpe_r_plot << "'\" " 
    << "-e \"rpe_t_plot='" << rpe_t_plot << "'\" " 
    << "-e \"rpe_t_plot='" << rpe_t_plot << "'\" "
    << "-e \"gt_traj_file='" << gt_filename << "'\" "
    << "-e \"ev_traj_file='" << traj_filename << "'\" "
    << "-e \"traj_xy_plot='" << traj_xy_plot << "'\" "
    << "-e \"traj_yz_plot='" << traj_yz_plot << "'\" "
    << "-e \"traj_xz_plot='" << traj_xz_plot << "'\" "
    << " make_plots.gnuplot ";
  log << "GNUPLOT_COMMAND: " <<   gnuplot_command.str() << endl;
  int result=system(gnuplot_command.str().c_str());
  if (result!=0) {
    log << "Error in plotting" << endl;
  }
  log << "EV Raw filename  [" << ev_raw_filename << "] END" <<  endl << endl;
  return true;
}
