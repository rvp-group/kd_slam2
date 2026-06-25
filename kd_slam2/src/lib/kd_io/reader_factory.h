#pragma once
#include "message_reader.h"
#include "message_writer.h"
#include <memory>
#include <string>
#include <vector>

std::shared_ptr<MessageReader> makeRosbag2Reader(
    const std::string& bag_path = "",
    const std::vector<std::string>& topics = {});

std::shared_ptr<MessageWriter> makeRosbag2Writer(
    const std::string& out_path = "");
