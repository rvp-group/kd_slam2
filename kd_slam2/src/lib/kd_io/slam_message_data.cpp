#include "slam_message_data.h"
#include "tree_blob_.h"

using namespace srrg2_core;
void RosHeader::serialize(ObjectData& odata, IdContext& context){
  odata.setUnsignedInt("ts", stamp_ns);
  odata.setString("frame_id", frame_id);
}

void RosHeader::deserialize(ObjectData& odata, IdContext& context){
  stamp_ns=odata.getUnsignedInt("ts");
  frame_id=odata.getString("frame_id");
}


void ImuData::serialize(ObjectData& odata, IdContext& context) {
  auto hdr=new ObjectData;
  header.serialize(*hdr, context);
  odata.setField("header", hdr);
  odata.setEigen("orientation",orientation);
  odata.setEigen("ang_v",angular_velocity);
  odata.setEigen("lin_a",linear_acceleration);
}

void ImuData::deserialize(ObjectData& odata, IdContext& context){
  auto& hdr=odata.getField("header")->getObject();
  header.deserialize(hdr, context);
  orientation = odata.getEigen<Eigen::Quaterniond>("orientation");
  angular_velocity = odata.getEigen<Eigen::Vector3d>("ang_v");
  linear_acceleration = odata.getEigen<Eigen::Vector3d>("lin_a");
}


void TransformData::serialize(ObjectData& odata, IdContext& context) {
  auto hdr=new ObjectData;
  header.serialize(*hdr,context);
  odata.setField("header", hdr);
  odata.setString("child_frame_id", child_frame_id);
  odata.setEigen("translation",translation);
  odata.setEigen("rotation",rotation);
}

void TransformData::deserialize(ObjectData& odata, IdContext& context){
  auto& hdr=odata.getField("header")->getObject();
  header.deserialize(hdr, context);
  child_frame_id=odata.getString("child_frame_id");
  translation = odata.getEigen<Eigen::Vector3d>("translation");
  rotation = odata.getEigen<Eigen::Quaterniond>("rotation");
}

void OdometryData::serialize(ObjectData& odata, IdContext& context) {
  auto hdr=new ObjectData;
  header.serialize(*hdr, context);
  odata.setField("header", hdr);
  odata.setString("child_frame_id", child_frame_id);
  odata.setEigen("position",position);
  odata.setEigen("orientation",orientation);
}

void OdometryData::deserialize(ObjectData& odata, IdContext& context){
  auto hdr=odata.getField("header")->getObject();
  header.deserialize(hdr, context);
  child_frame_id=odata.getString("child_frame_id");
  position = odata.getEigen<Eigen::Vector3d>("position");
  orientation = odata.getEigen<Eigen::Quaterniond>("orientation");
  pose_covariance.setZero();
  linear_velocity.setZero();
  angular_velocity.setZero();
  twist_covariance.setZero();
}

void TFMessageData::serialize(ObjectData& odata, IdContext& context) {
  if (transforms.size())
    header=transforms[0].header;
  auto tfs=new ArrayData;
  for(size_t i=0; i<transforms.size(); ++i) {
    auto ot=new ObjectData;
    transforms[i].serialize(*ot, context);
    tfs->add(ot);
  }
  odata.setField("transforms", tfs);
}

void TFMessageData::deserialize(ObjectData& odata, IdContext& context){
  auto tfs=odata.getField("transforms");
  if (! tfs)
    return;
  auto& adata=tfs->getArray();
  transforms.resize(adata.size());
  for (size_t i=0; i<adata.size(); ++i)
    transforms[i].deserialize(adata[i].getObject(), context);
  if (transforms.size())
    header=transforms[0].header;
}

void LaserScanData::serialize(ObjectData& odata, IdContext& context){
  auto hdr=new ObjectData;
  header.serialize(*hdr, context);
}

void LaserScanData::deserialize(ObjectData& odata, IdContext& context){
  auto hdr=odata.getField("header")->getObject();
  header.deserialize(hdr, context);
}

/*PointXYT*/
namespace kd_io {
  template struct StdVectorBlob_<PointXYT>;
}

using PointXYTVectorBlob=kd_io::StdVectorBlob_<PointXYT>;

namespace srrg2_core{
  template struct BLOBReference<kd_io::StdVectorBlob_<PointXYT>>;
}

using PointXYTVectorBlobReference=kd_io::StdVectorBlobReference_<PointXYT>;


void PointCloudXYTData::serialize(ObjectData& odata, IdContext& context) {
  auto hdr=new ObjectData;
  header.serialize(*hdr, context);
  odata.setField("header", hdr);
  PointXYTVectorBlobReference blob_ref;
  auto blob=new PointXYTVectorBlob(points);
  blob_ref.set(blob);
  auto blob_data=new ObjectData;
  blob_ref.serialize(*blob_data, context);
  odata.setField("points", blob_data);
}

void PointCloudXYTData::deserialize(srrg2_core::ObjectData& odata, IdContext& context) {
  auto& hdr=odata.getField("header")->getObject();
  header.deserialize(hdr, context);
  ObjectData& blob_data=odata.getField("points")->getObject();
  PointXYTVectorBlobReference blob_ref;
  blob_ref.deserialize(blob_data, context);
  PointXYTVectorBlob* points_blob=blob_ref.get();
  if (points_blob)
    points=*points_blob;
}

/*PointXYZT*/

namespace kd_io {
  template struct StdVectorBlob_<PointXYZT>;
}

using PointXYZTVectorBlob=kd_io::StdVectorBlob_<PointXYZT>;

namespace srrg2_core{
  template struct BLOBReference<kd_io::StdVectorBlob_<PointXYZT>>;
}

using PointXYZTVectorBlobReference=kd_io::StdVectorBlobReference_<PointXYZT>;


void PointCloudXYZTData::serialize(ObjectData& odata, IdContext& context) {
  auto hdr=new ObjectData;
  header.serialize(*hdr, context);
  odata.setField("header", hdr);
  odata.setBool("per_pt_ts",has_per_point_timestamps);
  PointXYZTVectorBlobReference blob_ref;
  auto blob=new PointXYZTVectorBlob(points);
  blob_ref.set(blob);
  auto blob_data=new ObjectData;
  blob_ref.serialize(*blob_data, context);
  odata.setField("points", blob_data);
}

void PointCloudXYZTData::deserialize(srrg2_core::ObjectData& odata, IdContext& context) {
  auto& hdr=odata.getField("header")->getObject();
  header.deserialize(hdr, context);
  has_per_point_timestamps = odata.getBool("per_pt_ts");
  ObjectData& blob_data=odata.getField("points")->getObject();
  PointXYZTVectorBlobReference blob_ref;
  blob_ref.deserialize(blob_data, context);
  PointXYZTVectorBlob* points_blob=blob_ref.get();
  if (points_blob)
    points=*points_blob;
}

/*descriptor*/

/*
  RosHeader   header;
  std::string src_topic;
  int32_t     dim               = 0;
  int32_t     level             = 0;
  int32_t     axes_canonization = -1;
  std::vector<float> root_eigenvectors;  // dim*dim, col-major
  std::vector<float> entries;            // ((1<<level)-2) * 2 * dim
  void serialize(srrg2_core::ObjectData&, srrg2_core::IdContext& );
  void deserialize(srrg2_core::ObjectData&, srrg2_core::IdContext&);

 */
void KDDescriptorData::serialize(ObjectData& odata, IdContext& context) {
  auto hdr=new ObjectData;
  header.serialize(*hdr, context);
  odata.setField("header", hdr);
  odata.setInt("dim", dim);
  odata.setInt("level", level);
  odata.setInt("axes_canonization", axes_canonization);
  ArrayData* adata=new ArrayData;
  for (size_t i=0; i<root_eigenvectors.size(); ++i)
    adata->add(root_eigenvectors[i]);
  odata.setField("root_eigenvectors", adata);
  adata=new ArrayData;
  for (size_t i=0; i<entries.size(); ++i)
    adata->add(entries[i]);
  odata.setField("entries", adata);
}

void KDDescriptorData::deserialize(srrg2_core::ObjectData& odata, IdContext& context) {
  auto& hdr=odata.getField("header")->getObject();;
  header.deserialize(hdr, context);
  dim=odata.getInt("dim");
  level=odata.getInt("level");
  axes_canonization = odata.getInt("axes_canonization");

  root_eigenvectors.clear();
  ArrayData& adata=odata.getField("root_eigenvectors")->getArray();
  for (size_t i=0; i<adata.size(); ++i)
    root_eigenvectors.push_back(adata[i].getFloat());

  entries.clear();
  ArrayData& edata=odata.getField("entries")->getArray();
  for (size_t i=0; i<edata.size(); ++i)
    entries.push_back(edata[i].getFloat());
}

/* Tree2f*/

namespace kd_io {
  template struct TreeBlob_<kd_slam::TreeCPUPoint2f::NodeType>;
}

using TreeBlob2f=kd_io::TreeBlob_<kd_slam::TreeCPUPoint2f::NodeType>;

namespace srrg2_core{
  template struct BLOBReference<kd_io::TreeBlob_<kd_slam::TreeCPUPoint2f::NodeType>>;
}

using TreeBlob2fReference=BLOBReference<kd_io::TreeBlob_<kd_slam::TreeCPUPoint2f::NodeType>>;


void KDTreeData2f::serialize(ObjectData& odata, IdContext& context) {
  auto hdr=new ObjectData;
  header.serialize(*hdr, context);
  odata.setField("header", hdr);
  odata.setString("src_topic", src_topic);
  auto blob=new TreeBlob2f;
  kd_slam::Tree_CPUfromCPU_(blob->tree,tree);
  TreeBlob2fReference blob_ref;
  blob_ref.set(blob);
  auto blob_data=new ObjectData;
  blob_ref.serialize(*blob_data, context);
  odata.setField("tree", blob_data);
}

void KDTreeData2f::deserialize(srrg2_core::ObjectData& odata, IdContext& context) {
  auto& hdr=odata.getField("header")->getObject();;
  header.deserialize(hdr, context);
  src_topic=odata.getString("src_topic");
  ObjectData& blob_data=odata.getField("tree")->getObject();
  TreeBlob2fReference blob_ref;
  blob_ref.deserialize(blob_data, context);
  TreeBlob2f* blob=blob_ref.get();
  if (blob) {
    kd_slam::Tree_CPUfromCPU_(tree,blob->tree);
 }
}

/*Tree3f*/

namespace kd_io {
  template struct TreeBlob_<kd_slam::TreeCPUPoint3f::NodeType>;
}

using TreeBlob3f=kd_io::TreeBlob_<kd_slam::TreeCPUPoint3f::NodeType>;

namespace srrg2_core{
  template struct BLOBReference<kd_io::TreeBlob_<kd_slam::TreeCPUPoint3f::NodeType>>;
}

using TreeBlob3fReference=BLOBReference<kd_io::TreeBlob_<kd_slam::TreeCPUPoint3f::NodeType>>;


void KDTreeData3f::serialize(ObjectData& odata, IdContext& context) {
  auto hdr=new ObjectData;
  header.serialize(*hdr, context);
  odata.setField("header", hdr);
  odata.setString("src_topic", src_topic);
  auto blob=new TreeBlob3f;
  kd_slam::Tree_CPUfromCPU_(blob->tree,tree);
  TreeBlob3fReference blob_ref;
  blob_ref.set(blob);
  auto blob_data=new ObjectData;
  blob_ref.serialize(*blob_data, context);
  odata.setField("tree", blob_data);
}

void KDTreeData3f::deserialize(srrg2_core::ObjectData& odata, IdContext& context) {
  auto& hdr=odata.getField("header")->getObject();;
  header.deserialize(hdr, context);
  src_topic=odata.getString("src_topic");
  ObjectData& blob_data=odata.getField("tree")->getObject();
  TreeBlob3fReference blob_ref;
  blob_ref.deserialize(blob_data, context);
  TreeBlob3f* blob=blob_ref.get();
  if (blob) {
    kd_slam::Tree_CPUfromCPU_(tree,blob->tree);
 }
}

void __attribute__((constructor)) registerBLOBS() {
    BOSS_REGISTER_BLOB(TreeBlob3f);
    BOSS_REGISTER_BLOB(TreeBlob2f);
    BOSS_REGISTER_BLOB(PointXYZTVectorBlob);
    BOSS_REGISTER_BLOB(PointXYTVectorBlob);
}

