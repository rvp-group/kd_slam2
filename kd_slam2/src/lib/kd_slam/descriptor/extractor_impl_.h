#include "extractor_.h"
#include "descriptor_impl_.h"


namespace kd_slam { namespace descriptor {

template <typename Tree_, int Level_>
typename Extractor_<Tree_, Level_>::DescriptorType
Extractor_<Tree_, Level_>::extract(const Tree_& tree, int canonization) {
  using namespace std;
  DescriptorType desc;
  using VectorType = typename Tree_::VectorType;
  desc.axes_canonization=-1;
  if (canonization>=NumAxesCanonizations)
    return desc;
  MatrixType R=DescriptorType::applyCanonization(tree.root_eigenvectors, canonization);
  MatrixType Rt=R.transpose();
        
  //cerr << "R " << endl << R << endl;
  VectorType t=tree.root_mean;
  //cerr << "t " << endl << t.transpose() << endl;
  for (size_t i=0; i<Size; ++i) {
    if (i>=tree.numNodes()) {
      return desc;
    }
    const auto& n=tree._nodes_ptr[i+1];
    if (n.isLeaf()) {
      //cerr << "n.isLeaf() : " << n.isLeaf() << endl;
      desc.axes_canonization=-1;
      cerr << "num_nodes: " << tree.numNodes() << endl;
      //throw std::runtime_error("tree too small");
    }
    //cerr << "n: " << n._mean.transpose() << " " << n._direction.transpose() <<endl ;
    desc.values[i].m=Rt*(n._mean-t);
    desc.values[i].d=Rt*n._direction;
  }
  desc.root_transform.linear()=R;
  desc.root_transform.translation()=t;
  desc.axes_canonization=canonization;
  return desc;
 
}

} } // namespace kd_slam::descriptor
