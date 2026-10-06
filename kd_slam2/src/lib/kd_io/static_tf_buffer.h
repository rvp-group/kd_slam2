#pragma once
#include <Eigen/Geometry>
#include <map>
#include <string>
#include <vector>

namespace kd_io {
  struct StaticTFBuffer {
  public:
    void addTransform(const std::string& parent_name,
                      const std::string& child_name,
                      const Eigen::Isometry3f& iso);

    bool getTransform(Eigen::Isometry3f& iso,
                      const std::string& from_name,
                      const std::string& to_name);
    void clear();
  protected:
    
    struct TFLink {
      std::string name;
      int parent_id=-1;
      Eigen::Isometry3f iso=Eigen::Isometry3f::Identity();
      int root=-1; // root of the subtree
      int level=-1;
    };
    inline int nameToId(const std::string& name);
    int addLink(const std::string name,
                int parent_id=-1,
                const Eigen::Isometry3f& iso=Eigen::Isometry3f::Identity());
    void fix();
    std::map<std::string, int> name_to_id;
    std::vector<TFLink> tree;
  
  };
}
