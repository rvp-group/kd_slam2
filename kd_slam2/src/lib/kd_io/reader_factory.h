#pragma once
#include "message_reader.h"
#include "message_writer.h"
#include <memory>
#include <string>
#include <vector>

std::shared_ptr<MessageReader> makeMessageReader(const std::string& type="Rosbag2MessageReader",
                                                 const std::string& bag_path = "",
                                                 const std::vector<std::string>& topics = {});

std::shared_ptr<MessageWriter> makeMessageWriter(const std::string& type="Rosbag2MessageWriter",
                                                 const std::string& bag_path = "");
