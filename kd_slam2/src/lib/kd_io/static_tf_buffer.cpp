#include "static_tf_buffer.h"
#include <iostream>

using namespace std;
namespace kd_io {

  // seeks for id, -1 if not found
  inline int StaticTFBuffer::nameToId(const std::string& name) {
    auto it=name_to_id.find(name);
    if (it==name_to_id.end())
      return -1;
    return it->second;
  }

  // adds a link (id should be not existing)
  int StaticTFBuffer::addLink(const std::string name,
                              int parent_id,
                              const Eigen::Isometry3f& iso) {
    int id=tree.size();
    tree.push_back(TFLink{name, parent_id, iso, -1, -1});
    //auto& l=tree[id];
    name_to_id[name]=id;
    //cerr << "addLink| id: " << id << " name: " << name << " parent_id: " << parent_id << endl;
    return id;
  }

  // adds a transform
  void StaticTFBuffer::addTransform(const std::string& parent_name,
                                    const std::string& child_name,
                                    const Eigen::Isometry3f& iso) {
    //cerr << "addLink " << parent_name << " " << child_name << endl;
    bool root_changed=false;
    // seek parent
    int parent_id=nameToId(parent_name);
    // is there a parent? if not we make it
    if (parent_id<0) {
      parent_id = addLink(parent_name);
      root_changed=true;
    }
    // seek child
    int child_id=nameToId(child_name);

    if (child_id<0) {
      // child not present, add
      child_id = addLink(child_name, parent_id, iso);
      root_changed=true;
    } else {
      // child present, bookkeep
      auto& c=tree[child_id];
      auto& p=tree[parent_id];
      if (c.parent_id>=0) {
        
        // parent_id was already set, we throw
        if (c.parent_id!=parent_id) 
          throw std::runtime_error("parent mess");

      } else {
        //we reparent
        if (c.root==p.root)
          throw std::runtime_error("loop");
          
        
        c.parent_id=parent_id;
        root_changed=true;
      }
      c.iso=iso;
    }
    //cerr << "addLink| " << parent_id << " " << child_id << endl;
    // if the root is changed we need to
    // recompute bookeeping
    if (root_changed)
      fix();
    
  }

  void StaticTFBuffer::fix() {
    //cerr << "fix" << endl;
    //clear bookeeping
    for (auto& n: tree){
      n.root=-1;
      n.level=-1;
    }

    // this holds the reverse path for assigning levels
    std::vector<int> path;
    path.reserve(tree.size());

    for (size_t i=0; i<tree.size(); ++i) {
      path.clear();
      // we climb to the top of he chain to find/assign the root;
      int idx=i;
      int root=-1;
      //cerr << "climb" << endl;
      while (1) {
        path.push_back(idx);
        auto& l=tree[idx];
        //cerr << "idx: "  << idx << " parent_id: " << l.parent_id << endl;
        if (l.root>=0){
          root=l.root; // root already assigned
          break;
        }
        if (l.parent_id<0){
          root=idx;
          l.root=idx;
          l.level=0;
          break;
        }
        idx=l.parent_id;
      }
      //cerr <<  "propagate " << path.size() <<  " root: " <<root << endl;
      if (root==-1)
        throw std::runtime_error("no root");
      if (! path.size())
        continue;
      for (int k=(int)path.size()-1; k>=0; --k){
        int idx=path[k];
        auto &l=tree[idx];
        //cerr << "k: " << k << " id: " << idx << " parent_id: " << l.parent_id << " root: " << l.root << endl;
        if (l.parent_id>=0) {
          l.level=tree[l.parent_id].level+1;
          l.root=root;
        }
      }
      //cerr << "done" << endl;
      // now we have a root, we can 
    }
    // for (size_t i=0; i<tree.size(); ++i) {
    //   auto& l=tree[i];
    //   cerr << "id:" << i
    //        << " name: " << l.name
    //        << " p:" << l.parent_id
    //        << " root:" << l.root
    //        << " level: " << l.level
    //        << endl;
    // }
  }

  bool StaticTFBuffer::getTransform(Eigen::Isometry3f& iso,
                                    const std::string& from_name,
                                    const std::string& to_name) {
    int from_id=nameToId(from_name);
    if (from_id<0)
      return false;

    int to_id=nameToId(to_name);
    if (to_id<0)
      return false;

    auto& from=tree[from_id];
    auto& to=tree[to_id];
    if (from.root!=to.root)
      return false;
    Eigen::Isometry3f from_iso=Eigen::Isometry3f::Identity();
    Eigen::Isometry3f to_iso=Eigen::Isometry3f::Identity();

    while (from_id!=to_id) {
      auto& f=tree[from_id];
      auto& t=tree[to_id];
      if (f.level>=t.level) {
        cerr << "from | " << f.name << endl;
        from_iso=f.iso*from_iso;
        from_id=f.parent_id;
      } else {
        cerr << "to | " << t.name << endl;
        to_iso=t.iso*to_iso;
        to_id=t.parent_id;
      }
    }
    iso = from_iso.inverse()*to_iso;
    return true;
  
  }



  void StaticTFBuffer::clear(){
    name_to_id.clear();
    tree.clear();
    
  }

}
/*
int main(){
  using namespace kd_io;
  
  Eigen::Isometry3f iso=Eigen::Isometry3f::Identity();
  StaticTFBuffer buf;
  buf.addTransform("a_root", "a_base1", iso);
  buf.addTransform("b_root", "b_base1", iso);
  buf.addTransform("world", "a_root", iso);
  buf.addTransform("a_root", "a_base2", iso);
  buf.addTransform("b_root", "b_base2", iso);
  buf.addTransform("world", "b_root", iso);

  Eigen::Isometry3f q;
  bool result= buf.getTransform(q, "b_base2","a_base1");
  cerr<< "query: " << result << endl;
}
*/
