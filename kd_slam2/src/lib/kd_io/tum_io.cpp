#include "tum_io.h"

#include <cmath>
#include <fstream>
#include <limits>
#include <sstream>
#include <stdexcept>

std::map<double, Eigen::Isometry3f> readTUM(const std::string& path) {
    std::ifstream file(path);
    if (!file)
        throw std::runtime_error("Cannot open poses file: " + path);

    std::map<double, Eigen::Isometry3f> result;
    std::string line;

    while (std::getline(file, line)) {
        if (line.empty() || line[0] == '#') continue;

        std::istringstream ss(line);
        double ts, tx, ty, tz, qx, qy, qz, qw;
        if (!(ss >> ts >> tx >> ty >> tz >> qx >> qy >> qz >> qw))
            throw std::runtime_error("Malformed pose line: " + line);


        
                // Eigen quaternion constructor order: (w, x, y, z)
        Eigen::Quaternionf q(static_cast<float>(qw),
                             static_cast<float>(qx),
                             static_cast<float>(qy),
                             static_cast<float>(qz));
        q.normalize();
        
        auto pose = Eigen::Isometry3f::Identity();
        pose.linear()      = q.toRotationMatrix();
        pose.translation() = Eigen::Vector3f(static_cast<float>(tx),
                                                static_cast<float>(ty),
                                                static_cast<float>(tz));
        result[ts]=pose;
    }

    return result;
}
