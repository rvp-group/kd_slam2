#include <iostream>
#include <string>
#include "kd_app_common_.h"
#include "srrg_system_utils/parse_command_line.h"
#include "srrg_system_utils/system_utils.h"
#include "srrg_property/property_container_manager.h"
#include "kd_io/reader_factory.h"

using namespace srrg2_core;
using namespace kd_io;

const char* banner[] = {
  "kd_translator: converts messages between ros and boss",
  "usage: kd_translator [options]",
  nullptr
};

int main(int argc, char** argv) {
  srrgInit(argc, argv);
  using namespace std;
  using namespace kd_slam::cuda;
  srrg2_core::ParseCommandLine cmd(argv, banner);
  srrg2_core::ArgumentString a_config(&cmd, "c",  "config",       "pipeline config file",                                "config.json");
  srrg2_core::ArgumentString a_input (&cmd, "i",  "input",        "name of input bag or mcap file",                      "");
  srrg2_core::ArgumentString a_output (&cmd, "o",  "output",        "name of output mcap",                      "");
  srrg2_core::ArgumentString a_reader (&cmd, "r",  "reader",        "name of message reader",                  "reader");
  srrg2_core::ArgumentString a_writer (&cmd, "w",  "writer",        "name of message writer",                  "writer");
  cmd.parse();
  PropertyContainerManager manager;
  cmd.summary();
  manager.read(a_config.value());
  auto _reader = manager.getByName(a_reader.value());
  if (!_reader) {
    cerr << "cannot find a reader named [" << a_reader.value() << "] in config" << endl;
    return -1;
  }
  auto reader=std::dynamic_pointer_cast<MessageReader>(_reader);
  if (! reader) {
    cerr << "unable of upcast object [" << a_reader.value() << "] of type" << _reader->className() << " to MessageReader" << endl;
    return -1;
  }
  auto _writer = manager.getByName(a_writer.value());
  if (!_writer) {
    cerr << "cannot find a writer named [" << a_writer.value() << "] in config" << endl;
    return -1;
  }
  auto writer=std::dynamic_pointer_cast<MessageWriter>(_writer);
  if (! writer) {
    cerr << "unable of upcast object [" << a_writer.value() << "] of type" << _writer->className() << " to MessageWriter" << endl;
    return -1;
  }
  reader->param_bag_path.setValue(a_input.value());
  writer->param_out_path.setValue(a_output.value());
  reader->open();
  writer->open();
  while (reader->isGood()){
    auto m=reader->readOne();
    if (! m)
      break;
    writer->write(m->topic, m->log_stamp_ns, *m);
  }
}
