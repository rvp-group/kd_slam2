#include "kd_live_logger.h"
#include <sstream>
void OusterEvent::print(std::ostream& os) const {
  os<< "ou_ev";
}

KDLiveLogger::KDLiveLogger():
  last_proc(kd_slam::Init, 0, 0, 0, 0),
  last_tree(0,0,0,0)
{
  registerCallback<EvOuster>([this](std::shared_ptr<EvOuster> ev){
    last_ouster=*ev;
  });
  registerCallback<EvTree>([this](std::shared_ptr<EvTree> ev){
    last_tree=*ev;
  });
  registerCallback<EvProc>([this](std::shared_ptr<EvProc> ev){
    last_proc=*ev;
  });
}

std::string KDLiveLogger::statsJson() {
  JsonMapItem root;
  auto num=[](long long v){
    return std::make_unique<JsonSingleItem>(JsonItemBase::JsonItemType::NUMBER,
                                            std::to_string(v));
  };
  auto dnum=[](double v){
    return std::make_unique<JsonSingleItem>(JsonItemBase::JsonItemType::NUMBER,
                                            std::to_string(v));
  };
  auto stri=[](const std::string& s){
    return std::make_unique<JsonSingleItem>(JsonItemBase::JsonItemType::STRING,
                                            s);
  };
    
  auto ri = new JsonMapItem;
  (*ri)["imu_received"]=num(last_ouster.num_imus);
  (*ri)["imu_dropped"]=num(last_ouster.dropped_imus);
  (*ri)["cloud_received"]=num(last_ouster.num_clouds);
  (*ri)["cloud_dropped"]=num(last_ouster.dropped_clouds);
  (*ri)["q_filling"]=dnum(100.f*(float)last_ouster.q_size/(float)last_ouster.q_capacity);
  (*ri)["cloud_dropped"]=num(last_ouster.dropped_clouds);
  root["ingest"]=std::unique_ptr<JsonMapItem>(ri);
  ri = new JsonMapItem;
  (*ri)["t_read"]=dnum(last_tree.t_read);
  (*ri)["t_vox"]=dnum(last_tree.t_vox);
  (*ri)["t_tree"]=dnum(last_tree.t_tree);
  root["preprocess"]=std::unique_ptr<JsonMapItem>(ri);
  ri = new JsonMapItem;
  (*ri)["status"]=stri(kd_slam::KDStatusStr[last_proc.status]);
  double t_slam=last_proc.t_align+
    last_proc.t_prop+
    last_proc.t_loop+
    last_proc.t_rel+
    last_proc.t_opt;
  (*ri)["t_SLAM"]=dnum(t_slam);
  (*ri)["t_align"]=dnum(last_proc.t_prop);
  (*ri)["t_prop"]=dnum(last_proc.t_loop);
  (*ri)["t_loop"]=dnum(last_proc.t_rel);
  (*ri)["t_opt"]=dnum(last_proc.t_opt);
  root["SLAM"]=std::unique_ptr<JsonMapItem>(ri);
    
  std::ostringstream os;
  int lev=0;
  root.print(os, lev);
  return os.str();
};
