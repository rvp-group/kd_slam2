#include <chrono>
#include <iostream>
#include "tcp_socket.h"
#include "json_parser.h"
#include "ouster_metadata.h"
#include "tiny_http.h"
#include "udp_socket.h"
#include "ouster_lidar.h"
#include "ouster_metadata.h"
#include "legacy_profile.h"
#include "rng19_profile.h"
#include <fstream>
#include <fcntl.h>
#include <sys/stat.h>
#include <sys/mman.h>

using namespace std;

void OusterLidar::print(std::ostream& os, const CloudMessageData& cloud)  {
  for (const auto& p: cloud.points) {
    os << p.coords.transpose() << endl;
  }
}

OusterLidar::~OusterLidar(){
}

std::unique_ptr<JsonItemBase> readJson(std::string filename) {
  ifstream is(filename);
  if (!is)
    return nullptr;
  std::vector<char> rec_file;
  while (is) {
    char c;
    if(is.read(&c,1))
      rec_file.push_back(c);
  }
  std::string rec_string(rec_file.begin(), rec_file.end());
  return JsonParser::parse(rec_string);
}



int OusterLidar::open(std::string rec) {
  auto rec_node=readJson(rec);
  int ilevel=0;
  if (! rec_node)
    return -1;
  rec_node->print(cerr, ilevel);

  // read the fields from the manifest
  auto rec_map=rec_node->asMap();
  auto config_node=rec_map->find("config");
  if (config_node==rec_map->end())
    return -2;

  auto binary_filename_node=rec_map->find("binary_file");
  if (binary_filename_node==rec_map->end())
    return -3;
  std::string binary_filename=binary_filename_node->second->asString();

  auto sensor_ip_node=rec_map->find("sensor_ip");
  if (sensor_ip_node==rec_map->end())
    return -3;
  std::string sensor_ip_string=sensor_ip_node->second->asString();
  
  sockaddr_in server_addr{};
  server_addr.sin_family = AF_INET;
  server_addr.sin_port = 0;
  // Convert IP address from text to binary form
  if (inet_pton(AF_INET, sensor_ip_string.c_str(), &server_addr.sin_addr) <= 0) {
    return -4;
  }
  sensor_ip=server_addr.sin_addr.s_addr;
  
  auto config_map=config_node->second->asMap();
  OusterMetadata meta;
  fillOusterMetadata(*config_map, meta);
  init(meta);
  cerr << "initialized read" << endl;
  cerr << "sensor ip: " <<   sensor_ip_string << endl;
  cerr << "binary_file: " << binary_filename << endl;
  log_fd = ::open(binary_filename.c_str(), O_RDONLY);
  if (log_fd<0)
    return -6;
  initialized=true;
  return 1;
}

int OusterLidar::open(std::string host, uint16_t port){
  initialized=false;
  HTTPRequest req_config;
  req_config.method="GET";
  req_config.path="/api/v1/sensor/metadata";
  req_config.headers.push_back(std::make_pair("Host",host));
  req_config.headers.push_back(std::make_pair("Connection","close"));
  HTTPResponse resp_config=sendRequest(req_config, host, port);
  cerr << "got response " << resp_config.payload.size() << " chars" << endl;
  if (resp_config.status!=200) {
    cerr << "no http response" << endl;
    return -1;
  }
  // need to check the status of the response, if not ok, we exit
  
  std::string config_string((char*)resp_config.payload.data());
  auto conf=JsonParser::parse(config_string);
  if (! conf)
    return -3;
  if (on_config)
    on_config(config_string);
  int level=0;
  OusterMetadata meta;
  fillOusterMetadata(*conf->asMap(), meta);
  init(meta);

  sockaddr_in server_addr{};
  server_addr.sin_family = AF_INET;
  server_addr.sin_port = 0;
  // Convert IP address from text to binary form
  if (inet_pton(AF_INET, host.c_str(), &server_addr.sin_addr) <= 0) {
    return -4;
  }
  sensor_ip=server_addr.sin_addr.s_addr;
  return 1;
}

void OusterLidar::init(OusterMetadata& metadata) {
  // network params
  lidar_port=metadata.config_params.udp_port_lidar;
  imu_port=metadata.config_params.udp_port_imu;

  // fetch config data;
  imu_to_sensor=metadata.imu_intrinsics.imu_to_sensor_transform;
  lidar_to_sensor=metadata.lidar_intrinsics.lidar_to_sensor_transform;
  column_window_start=metadata.lidar_data_format.column_window[0];
  column_window_end=metadata.lidar_data_format.column_window[1];
  columns_per_packet=metadata.lidar_data_format.columns_per_packet;
  columns_per_frame=metadata.lidar_data_format.columns_per_frame;
  pixels_per_column=metadata.lidar_data_format.pixels_per_column;
  b2l_t=metadata.beam_intrinsics.beam_to_lidar_transform.translation()*1e-3;
  b2l_n=sqrt(b2l_t.x()*b2l_t.x()+b2l_t.z()*b2l_t.z());
      
  size_t saz=metadata.beam_intrinsics.beam_azimuth_angles.size();
  size_t sel=metadata.beam_intrinsics.beam_altitude_angles.size();
  assert(saz==sel && "mismatch in beam config");
  beam_vectors.reserve(saz);
  for (size_t i=0; i<saz; ++i) {
    float az=metadata.beam_intrinsics.beam_azimuth_angles[i]*(M_PI/180.f);
    float el=metadata.beam_intrinsics.beam_altitude_angles[i]*(M_PI/180.f);
    Eigen::Vector3f r=
      Eigen::AngleAxisf(-az, Eigen::Vector3f::UnitZ())
      *Eigen::AngleAxisf(-el, Eigen::Vector3f::UnitY())
      *Eigen::Vector3f::UnitX();
    beam_vectors.push_back(r);
  }
    
  angle_inc=-(2.f*M_PI)/columns_per_frame;

  lidar_profiles["LEGACY"]=std::make_unique<LegacyLidarProfile>(*this, columns_per_packet, pixels_per_column);
  lidar_profiles["RNG19_RFL8_SIG16_NIR16"]=std::make_unique<Rng19LidarProfile>(*this, columns_per_packet, pixels_per_column);
  imu_profiles["LEGACY"]=std::make_unique<LegacyImuProfile>(*this);

  auto lp_it=lidar_profiles.find(metadata.config_params.udp_profile_lidar);
  if (lp_it==lidar_profiles.end())
    throw std::runtime_error("unsupported udp_profile_lidar: "+metadata.config_params.udp_profile_lidar);
  lidar_profile=lp_it->second.get();
  lidar_packet_size=lidar_profile->packetSize();

  auto ip_it=imu_profiles.find(metadata.config_params.udp_profile_imu);
  if (ip_it==imu_profiles.end())
    throw std::runtime_error("unsupported udp_profile_imu: "+metadata.config_params.udp_profile_imu);
  imu_profile=ip_it->second.get();

  initialized=true;
}

int OusterLidar::parseIMU(const uint8_t* packet, size_t size) {
  if (! initialized)
    return -2;
  return imu_profile->parsePacket(packet, size);
}

int OusterLidar::parseLidar(const uint8_t* packet, size_t size) {
  if (! initialized)
    return -2;
  return lidar_profile->parsePacket(packet, size);
}

void OusterLidar::spinLive(){
  if (! initialized)
    throw std::runtime_error("! init");
  run=true;
  cloud_idx=0;
  frames_received=0;
  imus_received=0;
  lidar_packets_received=0;
  prev_frame_id=0;
  cerr << endl;
  cerr << lidar_port << endl;
  cerr << imu_port << endl;
  static constexpr int BUF_SIZE=1024*30;
  uint8_t pack[BUF_SIZE];

  int lidar_fd = socket(AF_INET, SOCK_DGRAM, 0);
  if (lidar_fd == -1) 
    throw std::runtime_error("error in opening so");
  
  struct sockaddr_in local_addr{};
  local_addr.sin_family = AF_INET;
  local_addr.sin_addr.s_addr = INADDR_ANY;
  local_addr.sin_port = htons(lidar_port);
  if (bind(lidar_fd, (sockaddr*)&local_addr, sizeof(local_addr)) < 0)
    throw std::runtime_error("error in binding lidar");

  int imu_fd = socket(AF_INET, SOCK_DGRAM, 0);
  if (imu_fd == -1) 
    throw std::runtime_error("error in opening so");

  local_addr.sin_port = htons(imu_port);
  if (bind(imu_fd, (sockaddr*)&local_addr, sizeof(local_addr)) < 0) {
    close(lidar_fd);
    throw std::runtime_error("error in binding imu");
  }
  
  struct sockaddr_in ouster_addr;
  socklen_t addr_len=sizeof(struct sockaddr_in);
  size_t l_count = 0;
  size_t l_size  = 0;
  size_t i_count = 0;
  size_t i_size  = 0;
  auto last_print = std::chrono::steady_clock::now();

  fd_set readfds;
  struct timeval timeout;
  int max_fd=std::max(lidar_fd, imu_fd);
  int nc=-1;
  try {
    while(run) {
      timeout.tv_sec = 1;
      timeout.tv_usec = 0;
      FD_ZERO(&readfds);
      FD_SET(lidar_fd, &readfds);
      FD_SET(imu_fd, &readfds);
      int retval=select(max_fd+1,&readfds, NULL, NULL, &timeout);
      if (retval==-1) {
        if (errno == EINTR) {
          continue;
        }
        throw std::runtime_error("select error");
      } else if (retval == 0){
        cerr << "UDP timeout" << endl;
        continue;
      } else {
        bool lidar_in=false;
        if (FD_ISSET(lidar_fd, &readfds) ) {
          l_size = recvfrom(lidar_fd, pack, BUF_SIZE, 0, (sockaddr*)&ouster_addr, &addr_len);
          if (l_size <= 0) continue;
          if (l_size!=lidar_packet_size) {
            throw std::runtime_error("wrong lidar packet size");
          }
          ++lidar_packets_received;
          if (on_packet) {
            on_packet(ouster_addr.sin_addr.s_addr, lidar_port, pack, l_size);
          }
          if (retval && on_cloud ) {
            int retval = parseLidar(pack, l_size);
            if (on_cloud && frames_received>=2 && retval==1)
              on_cloud(readyCloud());
          }
          ++l_count;
          auto now = std::chrono::steady_clock::now();
          if (now - last_print > std::chrono::seconds(1)) {
            lidar_in=true;
            last_print = now;
          }
        } 
        if (FD_ISSET(imu_fd, &readfds)) {
          i_size = recvfrom(imu_fd, pack, BUF_SIZE, 0, (sockaddr*)&ouster_addr, &addr_len);
          if (i_size <= 0) continue;
            ++imus_received;
          if (on_packet) {
            on_packet(ouster_addr.sin_addr.s_addr, imu_port, pack, i_size);
          }
          if (on_imu) {
            int retval = parseIMU(pack, i_size);
            if (retval>0)
              on_imu(readyImu());
          }
          ++i_count;
        }
      }
    }
  } catch (...) {
    close(lidar_fd);
    close(imu_fd);
    throw;
  }

  close(lidar_fd);
  close(imu_fd);
}


void OusterLidar::spinLog(){
  if (! initialized)
    throw std::runtime_error("! init");
  if (log_fd<=0)
    return;
  // here we mmap all the file

  // gt the size 
  struct stat sb;
  if (fstat(log_fd, &sb) == -1) {
    close(log_fd);
    log_fd=0;
    return;
  }

  // Handle empty file edge case
  if (sb.st_size == 0) {
    return;
  }

  // mmap the file into memory
  uint8_t *data_start = (uint8_t*) mmap(NULL, sb.st_size, PROT_READ, MAP_PRIVATE, log_fd, 0);
  if (data_start == MAP_FAILED) {
    perror("Error mapping file");
    close(log_fd);
    return;
  }
  close(log_fd);

  uint8_t * data_end=data_start+sb.st_size;
  _parseBuffer(data_start, data_end);
}

void OusterLidar::_parseBuffer(uint8_t* data_start, uint8_t* data_end) {
  uint8_t * f=data_start;
  size_t imu_packet_size = imu_profile->packetSize();
  
  while (f<data_end) {
    uint16_t prt=0;
    size_t pack_size=*(size_t*)f;
    uint8_t* f_end=f+pack_size;
    f+=sizeof(size_t);
    uint32_t addr=*(uint32_t*)f;
    f+=sizeof(uint32_t);
    if (addr!=sensor_ip) {
      cerr << "unknown ip[";
      uint8_t* ip=(uint8_t*)&addr;
      for (int i=0; i<4; ++i){
        cerr << (int) ip[i] <<".";
      }
      cerr <<  "]" << endl;

      goto next;
    }
    prt=*(uint16_t*)f;
    f+=sizeof(uint16_t);
    if (prt==imu_port) {
      if (on_packet) {
        on_packet(addr, imu_port, f, imu_packet_size);
      }
      if (on_imu) {
        int retval=parseIMU(f, imu_packet_size);
        if (retval>0)
          on_imu(readyImu());
      }
    } else if (prt == lidar_port) {
      if (on_packet) {
        on_packet(addr, lidar_port, f, lidar_packet_size);
      }
      if (on_cloud) {
        int retval = parseLidar(f, lidar_packet_size);
        if (retval==1)
          on_cloud(readyCloud());
      }
    } else {
      cerr << "unknown port";
    }
  next:
    f=f_end;
  }
}
