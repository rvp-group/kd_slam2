#pragma once
#include "srrg_boss/blob.h"
#include "srrg_property/property_eigen.h"
#include "kd_slam/tree/tree_cpu_.h"
#include "slam_messages.h"

namespace kd_io {
  using namespace srrg2_core;
  using namespace kd_slam;

  template <typename ItemType_>
  struct StdVectorBlob_:
    public std::vector<ItemType_>,
    public BLOB
  {
    using ItemType = ItemType_;
    using StdVectorType=std::vector<ItemType_>;
    StdVectorBlob_() {}
    StdVectorBlob_(const std::vector<ItemType_>& src):
      StdVectorType(src){}
    bool read(std::istream& is) override {
      size_t new_size;
      is.read(reinterpret_cast<char*>(&new_size), sizeof(new_size));
      auto vec_ptr=static_cast<StdVectorType*>(this);
      vec_ptr->resize(new_size);
      size_t payload_size=sizeof(ItemType)*new_size;
      is.read(reinterpret_cast<char*>(vec_ptr->data()), payload_size);
      if (! is)
        return false;
      return true;
    }
    
    void write(std::ostream& os) const override {
      const auto vec_ptr=static_cast<const StdVectorType*>(this);
      size_t _size=vec_ptr->size();
      os.write(reinterpret_cast<const char*>(&_size),sizeof(_size));
      size_t payload_size=sizeof(ItemType)*_size;
      os.write(reinterpret_cast<const char*>(vec_ptr->data()),payload_size);
    }
    
    int serializedSize() const override {
      return sizeof(size_t) + sizeof(ItemType)*this->size();
    }

    const std::string& extension() override {
      static std::string ext("cld");
      return ext;
    }

  };

  template <typename NodeType_>
  struct TreeBlob_:
    public BLOB
  {
    using TreeBase = Tree_<NodeType_>;
    using TreeType = TreeCPU_< TreeBase >;
    TreeType tree;
    bool read(std::istream& is) override {
      tree._points_storage=nullptr; // points not owned
      tree._nodes_storage.clear();
      tree._leaves_indices_storage.clear();
      tree._points_ptr=0;
      tree._num_points=0;
      tree._num_leaves=0;
      char* begin = reinterpret_cast<char*>(&tree._points_ptr);
      char* end   = reinterpret_cast<char*>(&tree.root_mean) + sizeof(tree.root_mean);
      size_t header_size=end-begin;
      is.read(begin, header_size);
      size_t nodes_size=sizeof(NodeType_)*tree._num_nodes;
      //std::cerr << "Reading " << tree._num_nodes << " nodes" << std::endl;
      tree._nodes_storage.resize(tree._num_nodes);
      is.read(reinterpret_cast<char*>(tree._nodes_storage.data()), nodes_size);
      tree._nodes_ptr=tree._nodes_storage.data();
      tree._points_ptr=0;
      tree._num_points=0;
      tree._num_leaves=0;
      tree._leaves_indices_ptr=0;
      tree._leaves_indices_storage.clear();
      if (! is)
        return false;
      return true;
    }
    
    void write(std::ostream& os) const override {
      const char* begin = reinterpret_cast<const char*>(&tree._points_ptr);           // first data member, after the vptr
      const char* end   = reinterpret_cast<const char*>(&tree.root_mean) + sizeof(tree.root_mean);
      size_t header_size=end-begin;
      os.write(begin, header_size);
      size_t nodes_size=sizeof(NodeType_)*tree._nodes_storage.size();
      //std::cerr << "writing " << tree._nodes_storage.size() << " nodes " << std::endl;
      os.write(reinterpret_cast<const char*>(tree._nodes_storage.data()), nodes_size);
      
    }
    
    int serializedSize() const override {
      return sizeof(TreeBase) + sizeof(NodeType_)*tree._nodes_storage.size();
    }
    const std::string& extension() override {
      static std::string ext("tree");
      return ext;
    }
  };


  template <typename NodeType_>
  using TreeDataBlobReference_ =  BLOBReference<TreeBlob_<NodeType_>>;
  


  template <typename NodeType_>
  using TreeDataBlobReference_ =  BLOBReference<TreeBlob_<NodeType_>>;

  template <typename ItemType_>
  using StdVectorBlobReference_ =  BLOBReference<StdVectorBlob_<ItemType_>>;

}

