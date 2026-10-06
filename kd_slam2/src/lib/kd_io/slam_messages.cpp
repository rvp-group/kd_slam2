#include "slam_messages.h"
#include "slam_message_data.h"
#include "boss_message_reader.h"
#include "boss_message_writer.h"
#include "queue_message_reader.h"
using namespace srrg2_core;

void MessageBase::serialize(ObjectData& odata, IdContext& context) {
  odata.setUnsignedInt("file_offset",file_offset);
  odata.setUnsignedInt("log_stamp_ns",log_stamp_ns);
  odata.setString("topic", topic);
}

void MessageBase::deserialize(ObjectData& odata, IdContext& context) {
  file_offset=odata.getUnsignedInt("file_offset");
  log_stamp_ns=odata.getUnsignedInt("log_stamp_ns");
  topic=odata.getString("topic");
}


namespace kd_io {
  void __attribute__((constructor)) registerMessages() {
    using namespace srrg2_core;
    using namespace kd_slam;
    using MessageImu=Message_<ImuData>;
    using MessageOdometry=Message_<OdometryData>;
    using MessageTF=Message_<TransformData>;
    using MessageTFArray=Message_<TFMessageData>;
    using MessageLaserScan=Message_<LaserScanData>;
    using MessagePointCloudXYZT=Message_<PointCloudXYZTData>;
    using MessagePointCloudXYT=Message_<PointCloudXYZTData>;
    using MessageKDDescriptor=Message_<KDDescriptorData>;
    using MessageKDTree2f=Message_<KDTreeData2f>;
    using MessageKDTree3f=Message_<KDTreeData3f>;
    BOSS_REGISTER_CLASS(MessageImu);
    BOSS_REGISTER_CLASS(MessageOdometry);
    BOSS_REGISTER_CLASS(MessageTF);
    BOSS_REGISTER_CLASS(MessageTFArray);
    BOSS_REGISTER_CLASS(MessageLaserScan);
    BOSS_REGISTER_CLASS(MessagePointCloudXYZT);
    BOSS_REGISTER_CLASS(MessagePointCloudXYT);
    BOSS_REGISTER_CLASS(MessageKDDescriptor);
    BOSS_REGISTER_CLASS(MessageKDTree2f);
    BOSS_REGISTER_CLASS(MessageKDTree3f);
    BOSS_REGISTER_CLASS(BossMessageReader);
    BOSS_REGISTER_CLASS(BossMessageWriter);
    BOSS_REGISTER_CLASS(QueueMessageReader);

  }
}
