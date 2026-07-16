#pragma once
#include "tree_.h"
#include "tree_predicates_.h"

namespace kd_slam {

  /**
     kd tree cpu class.
     In this implementation the tree can be buil from points only in CPU and then copied on GPU.
     The CPU tree has a storage for the nodes as std::vector, and stores a reference to the points
  */

  template <typename KDTreeBase_>
  struct TreeCUDA_;

  template <typename KDTreeBase_>
  struct TreeCPU_: public KDTreeBase_{

    using KDTreeBase = KDTreeBase_;
    using Traits     = typename KDTreeBase::Traits;
    using NodeType     = typename KDTreeBase::NodeType;
    using PointType  = typename KDTreeBase::PointType;
    using VectorType = typename KDTreeBase::VectorType;
    using Scalar     = typename KDTreeBase::Scalar;
    static constexpr int Dim = KDTreeBase::Dim;
    using MatrixType  =Eigen::Matrix<Scalar, Dim, Dim>;
    using IsometryType = typename KDTreeBase::IsometryType;
    using PointsVectorType=std::vector<PointType>;
    using NodesVectorType=std::vector<NodeType>;
    PointsVectorType* _points_storage = nullptr;
    NodesVectorType _nodes_storage;
    std::vector<size_t> _leaves_indices_storage;
    static constexpr bool IsGPU=false;
    bool isGPU() const override {return IsGPU;}

    using VelocityVectorType = typename KDTreeBase::VelocityVectorType;
    using KDTreeBase::_points_ptr;
    using KDTreeBase::_num_points;
    using KDTreeBase::_nodes_ptr;
    using KDTreeBase::_num_nodes;
    using KDTreeBase::_ts_start;
    using KDTreeBase::_leaves_indices_ptr;
    using KDTreeBase::_num_leaves;
    using KDTreeBase:: numNodes;
    using KDTreeBase::root_mean;
    using KDTreeBase::root_eigenvectors;
  
    TreeCPU_() : _points_storage(nullptr) {}

    TreeCPU_(const TreeCPU_<KDTreeBase>& other);

    TreeCPU_(const KDTreeBase& tree);

    void copyNodes(Tree_<NodeType>& other) const override;

    std::vector<VectorType> extractCompressedLeaves() const override;
    std::vector<VectorType> extractCompressedLeaves(const VelocityVectorType& v) const override;

  
    /**constructs a tree from an array of points based on the policy*/
    template <typename PolicyType_>
    TreeCPU_(PointsVectorType& points,
             const PolicyType_& policy):
      _points_storage(&points){
      using namespace std;
      _points_ptr=&(*_points_storage)[0];
      _num_points=_points_storage->size();
      _nodes_storage.resize(2*_points_storage->size()); //HACK
      _nodes_ptr=&_nodes_storage[0];

      std::atomic<int> idx=0;
      _nodes_storage[0]=NodeType(_points_ptr, _nodes_ptr, idx, 0, _num_points, policy);

      KDTreeBase::root_eigenvectors.setZero();
      KDTreeBase::root_eigenvalues.setZero();
      KDTreeBase::root_mean.setZero();
      _num_nodes=idx+1;
      if (_num_nodes) {
        typename PolicyType_::LabelPolicyMetadata metadata;
        policy.fillMetadata(metadata, _nodes_storage[0], _points_ptr, 0);
        KDTreeBase::root_eigenvectors=metadata.eigenvectors;
        // ensure the eigenvector matrix has positive determinant;
        if (KDTreeBase::root_eigenvectors.determinant()<0) {
          KDTreeBase::root_eigenvectors.col(0)=-KDTreeBase::root_eigenvectors.col(0);
        }
        KDTreeBase::root_eigenvalues=metadata.eigenvalues;
        KDTreeBase::root_mean=metadata.mean;
        KDTreeBase::_ts_start=metadata.ts_start;
      }
    }
  
    // returns the index of the point in points_ptr that is closest to
    // thr query. Implemented only in cpu
    size_t searchFast(const VectorType& q) const;

    // returns all the points lying in a ball of radius r
    // exact search
    void searchFull(std::vector<size_t>& result, const VectorType& q, Scalar r, int node_idx=0) const;

    // visits a tree from the root (node 0) and returns
    // a permutation of the nodes that would arrange them
    // in a bf order
    std::vector<int> breadthFirstPermutation() const;

    // counts the nodes for which a predicate is true
    template <typename PredicateType_>
    size_t countNodes(const PredicateType_& predicate) const {
      size_t count=0;
      for (size_t i=0; i<_num_nodes; ++i)
        count+=predicate(_nodes_ptr[i]);
      return count;
    }

    // prints on the stream the nodes for which the predicate is true
    // before printing the node is transformed by iso
    template <typename PredicateType_>
    std::ostream& dumpSelectedNodes(std::ostream& os,
                                    const PredicateType_& predicate,
                                    const IsometryType& iso=IsometryType::Identity()) const {
      for (size_t i=0; i<_num_nodes; ++i) {
        const auto& n=_nodes_ptr[i];
        if (predicate(n))
          os << applyIsometry_(iso,n) << std::endl;
      }
      return os;
    }

    // writes in dest the indices of the nodes for which the predicate is true
    template <typename PredicateType_>
    void selectNodeIndices(std::vector<size_t>& dest, const PredicateType_& predicate) const {
      dest.resize(numNodes());
      int count=0;
      size_t* dest_ptr=&dest[0];
      for (size_t i=0; i<_num_nodes; ++i) {
        if (predicate(_nodes_ptr[i])) {
          dest_ptr[count]=i;
          ++count;
        }
      }
      dest.resize(count);
    }

    // writes in dest a copy of the nodes for which the predicate is true
    template <typename PredicateType_>
    void selectNodes(std::vector<NodeType>& dest, const PredicateType_& predicate) const {
      dest.resize(numNodes());
      NodeType* dest_start=&dest[0];
      NodeType* dest_ptr=dest_start;
      for (size_t i=0; i<_num_nodes; ++i) {
        if (predicate(_nodes_ptr[i])) {
          *dest_ptr=_nodes_ptr[i];
          ++dest_ptr;
        }
      }
      size_t count= dest_ptr-dest_start;
      dest.resize(count);
    }

    /**applies a permutation to the nodes
       perm[i]=j means that after applying the permutation
       the node at pos[i] will be the one that before was at position j
    */
    void applyPermutation(const std::vector<int>& permutation);



    /**
       applies a pruning predicate that asserts only for the leaves to be retained
       and suppresses all nodes that lead to a bad subtree.
       The resulting tree is smaller or equal to the original one
    */
    template <typename PredicateType_>
    int prune(const PredicateType_& predicate) {
      if (! numNodes())
        return 0;
      int new_root=_prune(0, predicate);
      if (new_root<0) {
        _num_nodes=0;
        _nodes_storage.clear();
        _nodes_ptr=0;
      }
    
      auto perm=breadthFirstPermutation();
      applyPermutation(perm);
      recomputeLeaves();
      return perm.size();
    }

  public:
    void applyVelocitiesInPlace(const VelocityVectorType& v, bool skip_leaves = false) override;

    void applyIsometryInPlace(const IsometryType& iso) override;

    std::shared_ptr<KDTreeBase_> clone() const override;

    // recomputes the leaves
    inline void recomputeLeaves() {
      _leaves_indices_storage.clear();
      _leaves_indices_storage.reserve(_num_nodes);
      for(size_t i=0; i<_num_nodes; ++i) {
        const auto& n=_nodes_ptr[i];
        if (n.isLeaf())
          _leaves_indices_storage.push_back(i);
      }
      _num_leaves=_leaves_indices_storage.size();
      _leaves_indices_ptr=_num_leaves ? &_leaves_indices_storage[0] : nullptr;
    }

    template <typename PredicateType_>
    int _prune(int idx, const PredicateType_& predicate) {
      using namespace std;
      if (! _num_nodes)
        return false;
      NodeType& n=_nodes_ptr[idx];
      if (n.isLeaf()) {
        if (predicate(n))
          return idx;
        else
          return -1;
      }
      if (n._idx_left>=0) {
        n._idx_left=_prune(n._idx_left, predicate);
      }
      if (n._idx_right>=0) {
        n._idx_right=_prune(n._idx_right, predicate);
      }
      if (n.isLeaf()){
        n._direction.setZero();
        return -1;
      } 
      return idx;
    }

  };


} // namespace kd_slam
