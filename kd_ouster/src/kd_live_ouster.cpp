#include "kd_live_ouster.h"
#include "ouster_lidar.h"
#include "kd_live_logger.h"

using namespace kd_slam;
using namespace kd_slam::event;

KDLiveOuster::KDLiveOuster(std::string h,
                           int p,
                           std::shared_ptr<MessageQueueBounded> m,
                           std::shared_ptr<EventQueue> e,
                           volatile bool& r):
  host(h),
  port(p),
  message_queue(m),
  ev_queue(e),
  run_flag(r)
{

}
// Old main()'s body, wrapped so it can be started/stopped as a session.
// session_run is this session's own flag, separate from the process-wide one.
void KDLiveOuster::run() {
  auto run_fn=[&](){
    run_flag=true;
    OusterLidar oust;
    std::string ouster_config;
    oust.on_config = [&](const std::string& conf) {
      ouster_config=conf;
    };
    size_t num_clouds=0;
    size_t num_imus=0;
    size_t dropped_clouds=0;
    size_t dropped_imus=0;
    int open_ok= false;
    if(port>0) {
      open_ok=oust.open(host, port);
    } else {
      open_ok=oust.open(host); // file reading
    }
    if (open_ok!=1) {
      cerr << "sensor open failed: " << open_ok << endl;
      return;
    }
    auto onCloud = [&](const OusterLidar::CloudMessageData& d) {
      oust.run=run_flag;
      auto msg=std::make_shared<Message_<OusterLidar::CloudMessageData> >(d);
      msg->topic="/ouster/points";
      msg->header.frame_id = "ouster_lidar_frame";
      if (message_queue && (port<0 || message_queue->size()<message_queue->capacity)){
        ++num_clouds;
        message_queue->push(msg);
      } else {
        ++dropped_clouds;
      }
      if (ev_queue && message_queue) {
        OusterEvent ev;
        ev.num_imus=num_imus;
        ev.num_clouds=num_clouds;
        ev.dropped_imus=dropped_imus;
        ev.dropped_clouds=dropped_clouds;
        ev.q_capacity = message_queue->capacity;
        ev.q_size = message_queue->size();
        auto event=std::make_shared<OusterEvent>(ev);
        ev_queue->pushEvent(event);
      }
    };
  
    auto onImu = [&](const OusterLidar::IMUData& d) {
      oust.run=run_flag;
      auto msg=std::make_shared<Message_<ImuData> >();
      msg->topic="/ouster/imu";
      msg->header.frame_id = "ouster_imu_frame";
      msg->orientation.setIdentity();
      msg->angular_velocity=Eigen::Vector3d(d.accel_x, d.accel_y, d.accel_z);
      msg->linear_acceleration=Eigen::Vector3d(d.gyro_x, d.gyro_y, d.gyro_z);
      if (message_queue &&
          (port<0 || message_queue->size()<message_queue->capacity)){
        ++num_imus;
        message_queue->push(msg);
      } else {
        ++dropped_imus;
      }
    };
    oust.on_cloud=onCloud;
    oust.on_imu=onImu;
    oust.run=true;
    if (port>0)
      oust.spinLive();
    else
      oust.spinLog();
    if (message_queue) {
      cerr << "terminating" << endl;
      message_queue->push(nullptr);
    }
  };
  run_th=std::thread(run_fn);
}

void KDLiveOuster::join(){
  if (run_th.joinable())
    run_th.join();
}
