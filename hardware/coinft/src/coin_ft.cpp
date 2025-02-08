/***************************************************************************************************
 * File:        CoinFT.cpp
 * Author:      Hojung Choi
 * Email:       hjchoi92@stanford.edu
 * Institution: Stanford University
 * Date:        2024-11-19
 * Description: Implementation of the CoinFT class.
 *
 * Usage:       The variable 'latest_data' is where the latest wrench is stored.
 *              ['Fx', 'Fy', 'Fz', 'Mx', 'My', 'Mz']
 *              Force in [N]. Moment in [Nm].
 ***************************************************************************************************/

#include "coinft/coin_ft.h"
#include <fstream>
#include <sstream>

CoinFT::CoinFT() {
  _force = RUT::Vector3d::Zero();
  _force_old = RUT::Vector3d::Zero();
  _torque = RUT::Vector3d::Zero();
  _torque_old = RUT::Vector3d::Zero();
  _wrench_tool_temp = RUT::VectorXd::Zero(12);
  _wrench_sensor_temp = RUT::VectorXd::Zero(12);
  _stall_counts = 0;
}

CoinFT::~CoinFT() {}

bool CoinFT::init(RUT::TimePoint time0, const CoinFTConfig& config) {
  std::cout << "[CoinFT] initializing connection to " << config.port
            << std::endl;
  _time0 = time0;
  _config = config;
  _adj_sensor_tool_left =
      RUT::SE32Adj(RUT::pose2SE3(config.PoseSensorToolLeft));
  _adj_sensor_tool_right =
      RUT::SE32Adj(RUT::pose2SE3(config.PoseSensorToolRight));

  std::vector<std::pair<int, std::string>> sensorConfigs = {
      {CoinFTBus::LEFT, config.left_calibration_file},
      {CoinFTBus::RIGHT, config.right_calibration_file}};

  _coinft_bus_ptr =
      std::make_shared<CoinFTBus>(config.port, config.baud_rate, sensorConfigs);
  std::cout << "[CoinFT] initialized and streaming." << std::endl;
  _flag_started = true;
  return true;
}

int CoinFT::getWrenchSensor(RUT::VectorXd& wrench, int num_of_sensors) {
  if (num_of_sensors != 2) {
    std::cerr << "[CoinFT] Number of sensors must be 2. Currently is "
              << num_of_sensors << std::endl;
  }
  assert(num_of_sensors == 2);
  assert(wrench.size() == 6 * num_of_sensors);

  _latest_data_left = _coinft_bus_ptr->getLatestData(CoinFTBus::LEFT);
  _latest_data_right = _coinft_bus_ptr->getLatestData(CoinFTBus::RIGHT);

  for (int i = 0; i < 6; ++i) {
    wrench[i] = _latest_data_left[i];
    wrench[i + 6] = _latest_data_right[i];
  }
  // TODO: check stall

  // safety
  for (int i = 0; i < 6; ++i) {
    if ((abs(wrench[i]) > _config.WrenchSafety[i]) ||
        (abs(wrench[i + 6]) > _config.WrenchSafety[i])) {
      std::cout << "\033[1;31m[CoinFT] Force magnitude is above the safety "
                   "threshold:\033[0m\n";
      std::cout << "  feedback:" << wrench.transpose() << std::endl;
      std::cout << "  safety limit: " << _config.WrenchSafety.transpose()
                << std::endl;
      return -1;
    }
  }

  return 0;
}

int CoinFT::getWrenchTool(RUT::VectorXd& wrench_T, int num_of_sensors) {
  int flag = this->getWrenchSensor(_wrench_sensor_temp, num_of_sensors);
  wrench_T.head(6) =
      _adj_sensor_tool_left.transpose() * _wrench_sensor_temp.head(6);
  wrench_T.tail(6) =
      _adj_sensor_tool_right.transpose() * _wrench_sensor_temp.tail(6);
  return flag;
}

int CoinFT::getWrenchNetTool(const RUT::Vector7d& pose,
                             RUT::VectorXd& wrench_net_T, int num_of_sensors) {
  // CoinFT does not use calibration here
  return this->getWrenchTool(wrench_net_T, num_of_sensors);
}
