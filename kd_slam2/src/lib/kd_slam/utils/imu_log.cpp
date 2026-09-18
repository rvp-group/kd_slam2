#include "imu_log.h"
#include <sstream>
#include <fstream>
#include <iomanip>

std::list<std::shared_ptr<RecordBase> >readLog(std::istream& is) {
  std::list<std::shared_ptr<RecordBase>> out;
  while (is.good()) {
    char line[10240];
    is.getline(line,10240);
    std::istringstream ls(line);
    std::string tag;
    ls >> tag;
    if (tag=="IMU"){
      auto r = std::make_shared<RecordImu>();
      ls >> r->ts
         >> r->gyro.x() >> r->gyro.y() >> r->gyro.z()
         >> r->acc.x() >> r->acc.y() >> r->acc.z();
      //cerr << "IMU ts_read: " << r->ts << " " << r->gyro.transpose() << endl;
      out.push_back(r);
    }
    if (tag=="UPDATE"){
      auto r = std::make_shared<RecordUpdate>();
      ls >> r->ts;
      //cerr << "UPD ts_read: " << r->ts << endl;
      for (int i=0; i<6; ++i)
        ls >> r->delta_log(i); 
      out.push_back(r);
    }
  }
  return out;
};
