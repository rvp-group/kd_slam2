#include <Eigen/Core>
#include <sstream>
#include <list>
#include <memory>
#include <fstream>

struct RecordBase {
  double ts;
  std::string tag;
  virtual ~RecordBase(){}
};

struct RecordImu: public RecordBase {
  RecordImu(){
    tag="IMU";
  }
  Eigen::Vector3f gyro;
  Eigen::Vector3f acc;
};

struct RecordUpdate: public RecordBase {
  RecordUpdate(){
    tag ="UPDATE";
  }
  Eigen::Matrix <float, 6, 1> delta_log;
};

std::list<std::shared_ptr<RecordBase> >readLog(std::istream& is);
