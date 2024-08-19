#pragma once

#include <Eigen/Dense>
#include <filesystem>

#include "helpers.hpp"

namespace fs = std::filesystem;

inline std::string makeFixedLength(const int i, const int length) {
  std::ostringstream ostr;
  if (i < 0)
    ostr << '-';
  ostr << std::setfill('0') << std::setw(length) << (i < 0 ? -i : i);
  return ostr.str();
}

inline std::tuple<std::string, std::string> create_folder_for_new_episode(
    const std::string& data_folder) {
  std::cout << "[create_folder_for_new_episode] Creating folder for new episode"
            << std::endl;
  int timestamp = std::chrono::seconds(std::time(NULL)).count();
  const auto timestamp_string = std::to_string(timestamp);
  std::string episode_folder = data_folder + "/episode_" + timestamp_string;
  std::string rgb_folder = episode_folder + "/rgb";

  if (!fs::exists(episode_folder)) {
    fs::create_directory(episode_folder);
    fs::create_directory(rgb_folder);
  } else {
    std::cerr << "Episode folder " << episode_folder
              << " already exists. Exiting." << std::endl;
    exit(1);
  }
  std::string data_filename = episode_folder + "/low_dim_data.json";
  return {rgb_folder, data_filename};
}

inline bool save_low_dim_data_json(std::ostream& os, int seq_id,
                                   double timestamp_ms,
                                   const RUT::Vector7d& pose,
                                   const RUT::Vector6d& wrench, bool mask) {
  Eigen::IOFormat good_looking_fmt(Eigen::StreamPrecision, Eigen::DontAlignCols,
                                   ", ", ", ", "", "", "", "");
  os << "\t{\n";
  os << "\t\t\"seq_id\": " << seq_id << ",\n";
  os << "\t\t\"mask\": " << mask << ",\n";
  os << "\t\t\"low_dim_time_stamps\": " << std::fixed << std::setprecision(2)
     << timestamp_ms << ",\n";
  os << std::fixed << std::setprecision(4);
  os << "\t\t\"ts_pose_fb\": [" << pose.format(good_looking_fmt) << "],\n";
  os << "\t\t\"wrench\": [" << wrench.format(good_looking_fmt) << "],\n";
  return true;
}

inline bool save_rgb_data(const std::string& foldername, int seq_id,
                          double time_ms, const cv::Mat& color_mat) {
  std::string file_name = foldername + "/img_" + makeFixedLength(seq_id, 6) +
                          "_" + std::to_string(time_ms) + "_ms.jpg";
  cv::imwrite(file_name, color_mat);
  return true;
}

inline void json_file_start(std::ostream& os) {
  os << "[\n";
}

inline void json_frame_ending(std::ostream& os) {
  os << "\t},\n";
}

inline void json_file_ending(std::ostream& os) {
  os << "\t}\n";
  os << "]\n";
}
