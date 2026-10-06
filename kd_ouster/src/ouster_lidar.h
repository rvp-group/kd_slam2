#pragma once
#include "ouster_metadata.h"
#include "kd_slam2/kd_io/slam_messages.h"
#include "kd_slam2/kd_io/slam_message_data.h"

#include <iostream>
using namespace std;

struct OusterLidar {
  /*
  struct Point {
    double stamp;
    Eigen::Vector3f coordinates;
    uint16_t signal;
    uint16_t near_ir;
    uint8_t reflectivity;
  };

  struct CloudMessageData {
    double stamp;
    std::vector<Point> points;
    inline void reset() {
      stamp=0;
      points.clear();
    }

    void print(std::ostream& os) const;
  };
  */
  using Point=PointXYZT;;
  using CloudMessageData=PointCloudXYZTData;
  using IMUMessageData = ImuData;
  
  static void print(std::ostream& os, const CloudMessageData& cloud);
  
  struct IMUData {
    double stamp;
    double acc_stamp;
    double gyro_stamp;
    float    accel_x;
    float    accel_y;
    float    accel_z;
    float    gyro_x;
    float    gyro_y;
    float    gyro_z;
  };

  struct Profile{
    Profile(OusterLidar& oust, size_t psize):
      _ouster(oust),
      _packet_size(psize){}
    virtual ~Profile()=default;
    // returns -1 wrong size, 0 processed, 1 processed + frame/imu ready
    virtual int parsePacket(const uint8_t* p, size_t sz)=0;
    size_t packetSize() { return _packet_size; }
    OusterLidar& _ouster;
    size_t _packet_size;
  };

  struct LidarProfile: public Profile {
    LidarProfile(OusterLidar& oust, size_t psize, float res=1e-3):
      Profile (oust, psize){
      resolution=res;
      half_res=resolution*0.5;
    }
    float resolution=1e-3;
    float half_res=resolution*0.5;
    Eigen::Matrix3f r0=Eigen::Matrix3f::Identity();
    // parser's job: call this once per column, before decoding its pixels
    inline void updateColumn(uint16_t measurement_id) {
      r0=Eigen::AngleAxisf(_ouster.angle_inc*measurement_id, Eigen::Vector3f::UnitZ()).toRotationMatrix();
    }
    // shared ray-to-point mapping, common to every lidar profile
    inline Point toPoint(double stamp,
                   int cidx,
                   uint32_t range,
                   uint8_t reflectivity,
                   uint16_t signal=0,
                   uint16_t near_ir=0,
                   uint8_t  window=0,
                   uint32_t range2=0,
                   uint8_t reflectivity2=0,
                   uint16_t signal2=0){
      float r = range2 ? (range+range2)*half_res : range*resolution;
      Eigen::Vector3f ep=_ouster.beam_vectors[cidx]*(r-_ouster.b2l_n)+_ouster.b2l_t;
      ep=r0*ep;
      return Point{ep,stamp};
      //return Point {stamp,ep,signal,near_ir, reflectivity};
    }

  };

  // concrete profiles (LegacyLidarProfile, Rng19LidarProfile, ...) live in
  // their own files and are registered into the maps below by init() --
  // this class doesn't know about any specific wire format.

  // keyed by udp_profile_lidar / udp_profile_imu, selected in init()
  std::map<std::string, std::unique_ptr<Profile> > lidar_profiles;
  std::map<std::string, std::unique_ptr<Profile> > imu_profiles;
  Profile* lidar_profile=nullptr;
  Profile* imu_profile=nullptr;
  
  void init(OusterMetadata& metadata);
  virtual ~OusterLidar();
  
  CloudMessageData& readyCloud() {return clouds[1-cloud_idx];}
  IMUData& readyImu() {return imus[1-imu_idx];}
  int parseLidar(const uint8_t* packet, size_t size);  
  int parseIMU(const uint8_t* packet, size_t size);  
  int open(std::string host, uint16_t port);
  int open(std::string log);
  void _parseBuffer(uint8_t* s, uint8_t* e);
  void spinLive();
  void spinLog();
  //protected
  std::function<void(const std::string&)> on_config;
  std::function<void(const CloudMessageData&)> on_cloud;
  std::function<void(uint32_t address, uint16_t port, const uint8_t* data, size_t size)> on_packet;
  std::function<void(const IMUData&)> on_imu;
  
  float angle_inc;
  Eigen::Vector3f b2l_t=Eigen::Vector3f::Zero();
  float b2l_n=0;
  Eigen::Isometry3f imu_to_sensor=Eigen::Isometry3f::Identity();
  Eigen::Isometry3f lidar_to_sensor=Eigen::Isometry3f::Identity();
  int column_window_start;
  int column_window_end;
  int columns_per_packet;
  int columns_per_frame;
  int pixels_per_column;
  size_t lidar_packet_size;
  bool  initialized=false;
  uint previous_col=0;
  CloudMessageData clouds[2];
  IMUData   imus[2];
  int cloud_idx=0;
  int imu_idx=0;
  CloudMessageData& pendingCloud() {return clouds[cloud_idx];}
  IMUData& pendingIMU() {return imus[imu_idx];}
  uint16_t prev_frame_id=0;
  int lidar_port=-1;
  int imu_port=-1;
  size_t frames_received=0;
  size_t imus_received=0;
  size_t lidar_packets_received=0;
  size_t valid_columns=0;
  size_t dropped_columns=0;
  

  std::vector<Eigen::Vector3f>  beam_vectors;
  std::vector<int> pixel_shift;
  float lidar_origin_to_beam_origin;
  volatile bool run=false;
  
  uint32_t sensor_ip=0;
  int log_fd=-1;
};
