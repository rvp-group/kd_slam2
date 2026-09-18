#include <Eigen/Core>
#include <Eigen/Dense>
#include <sstream>
#include <list>
#include <memory>
#include <iostream>
#include <fstream>
#include <iomanip>
#include "imu_log.h"
#include "imu_filter.h"
using namespace std;

int main(int argc, char**argv) {
  if (argc<2) {
    cerr<< "usage: gyro_filter <gyro_log>" << endl;
    return -1;
  }
  ifstream is(argv[1]);
  auto l=readLog(is);
  cerr << "read :" << l.size() << " records \n";
  cerr << "running filter" << endl;
  IMUFilter f;
  for (auto& r:l) {
    auto p_imu=std::dynamic_pointer_cast<RecordImu>(r);
    if (p_imu) {
      f.predict(p_imu->ts, p_imu->gyro, p_imu->acc);
    }
    auto p_update=std::dynamic_pointer_cast<RecordUpdate>(r);
    if (p_update) {
      cout << std::fixed << std::setprecision(9) << p_update->ts //1
           << " " << p_update->delta_log.transpose() << " "      //2:7
           << " " << f.predDelta().transpose();                  //8:13
      f.update(p_update->ts, p_update->delta_log);               
      cout << " " << f.updatedDelta().transpose()                //14:19
           << " " << f.sigma_x.block<3,3>(9 ,9 ).determinant()   //20
           << " " << f.sigma_x.block<3,3>(12,12).determinant()   //21
           << " " << f.X.p.transpose() << endl;                  //22:24
    }
  }
}
