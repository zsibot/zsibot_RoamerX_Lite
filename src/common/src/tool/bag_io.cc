/**
 * @file bag_io.cc
 * @author zhouchao (zhouchao@jushenzhiren.com)
 * @brief 
 * @version 0.1
 * @date 2024-11-13
 * 
 * @copyright Copyright (c) 2024
 * 
 */
#include "tool/bag_io.hpp"

namespace zsibot::common {

void RosbagIO::Go(int sleep_usec) {
rosbag2_cpp::Reader reader(
    std::make_unique<rosbag2_cpp::readers::SequentialReader>());
rosbag2_cpp::ConverterOptions cv_options{"cdr", "cdr"};

reader.open({bag_file_, "sqlite3"}, cv_options);
while (reader.has_next()) {
    if (exit_.load())
    break;

    auto msg = reader.read_next();
    auto iter = process_func_.find(msg->topic_name);
    if (iter != process_func_.end()) {
    iter->second(msg);
    }

    if (sleep_usec > 0) {
    std::this_thread::sleep_for(std::chrono::microseconds(sleep_usec));
    }

    if (flg_next_.load()) {
    char get_enter;
    std::cin.get(get_enter);
    }

    if (play_speed_ < 9.999) {
    float speep_time = -0.0009 * play_speed_ + 0.01;
    std::this_thread::sleep_for(
        std::chrono::microseconds(static_cast<int>(speep_time * 1e6)));
    }
}
}
}