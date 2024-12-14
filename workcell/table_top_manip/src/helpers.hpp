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

inline void create_folder_for_new_episode(
    const std::string& data_folder, std::vector<int> id_list,
    std::vector<std::string>& rgb_folders,
    std::vector<std::string>& robot_json_files,
    std::vector<std::string>& wrench_json_files) {
  std::cout << "[create_folder_for_new_episode] Creating folder for new episode"
            << std::endl;
  int timestamp = std::chrono::seconds(std::time(NULL)).count();
  const auto timestamp_string = std::to_string(timestamp);
  std::string episode_folder = data_folder + "/episode_" + timestamp_string;

  if (!fs::exists(episode_folder)) {
    fs::create_directory(episode_folder);
  } else {
    std::cerr << "Episode folder " << episode_folder
              << " already exists. Exiting." << std::endl;
    exit(1);
  }
  rgb_folders.clear();
  robot_json_files.clear();
  wrench_json_files.clear();
  for (int id : id_list) {
    std::string rgb_folder = episode_folder + "/rgb_" + std::to_string(id);
    fs::create_directory(rgb_folder);
    std::string robot_json_file =
        episode_folder + "/robot_data_" + std::to_string(id) + ".json";
    std::string wrench_json_file =
        episode_folder + "/wrench_data_" + std::to_string(id) + ".json";
    rgb_folders.push_back(rgb_folder);
    robot_json_files.push_back(robot_json_file);
    wrench_json_files.push_back(wrench_json_file);
    std::cout << "[create_folder_for_new_episode] Created rgb folder: "
              << rgb_folder << std::endl;
    std::cout << "[create_folder_for_new_episode] generated robot file: "
              << robot_json_file << std::endl;
    std::cout << "[create_folder_for_new_episode] generated wrench file: "
              << wrench_json_file << std::endl;
  }
}

inline bool save_robot_data_json(std::ostream& os, int seq_id,
                                 double timestamp_ms, const RUT::Vector7d& pose,
                                 bool mask) {
  Eigen::IOFormat good_looking_fmt(Eigen::StreamPrecision, Eigen::DontAlignCols,
                                   ", ", ", ", "", "", "", "");
  os << "\t{\n";
  os << "\t\t\"seq_id\": " << seq_id << ",\n";
  os << "\t\t\"mask\": " << mask << ",\n";
  os << "\t\t\"robot_time_stamps\": " << std::fixed << std::setprecision(2)
     << timestamp_ms << ",\n";
  os << std::fixed << std::setprecision(7);
  os << "\t\t\"ts_pose_fb\": [" << pose.format(good_looking_fmt) << "],\n";
  return true;
}

inline bool save_wrench_data_json(std::ostream& os, int seq_id,
                                  double timestamp_ms,
                                  const RUT::VectorXd& wrench) {
  Eigen::IOFormat good_looking_fmt(Eigen::StreamPrecision, Eigen::DontAlignCols,
                                   ", ", ", ", "", "", "", "");
  os << "\t{\n";
  os << "\t\t\"seq_id\": " << seq_id << ",\n";
  os << "\t\t\"wrench_time_stamps\": " << std::fixed << std::setprecision(2)
     << timestamp_ms << ",\n";
  os << std::fixed << std::setprecision(4);
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
