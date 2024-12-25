#include <iostream>
#include <mutex>

#include <RobotUtilities/spatial_utilities.h>
#include <RobotUtilities/timer_linux.h>
// #include <arx_can/arx_can.h>
#include <ati_netft/ati_netft.h>
#include <realsense/realsense.h>
#include <robotiq_ft_modbus/robotiq_ft_modbus.h>

int main() {
  ATINetft ati;
  Realsense realsense;
  RobotiqFTModbus robotiq;

  RUT::Timer timer;
  RUT::TimePoint time0 = timer.tic();

  ATINetft::ATINetftConfig ati_config;
  ati_config.ip_address = "192.168.1.101";
  ati_config.sensor_name = "wrist_ft_sensor";
  ati_config.fullpath = "";
  ati_config.print_flag = false;
  ati_config.publish_rate = 1000.0;
  ati_config.Foffset = {0.0, 0.0, 0.0};
  ati_config.Toffset = {0.0, 0.0, 0.0};
  ati_config.Gravity = {0.0, 0.0, 0.0};
  ati_config.Pcom = {0.0, 0.0, 0.0};
  ati_config.WrenchSafety = {50.0, 50.0, 70.0, 0.5, 0.5, 0.5};
  ati_config.PoseSensorTool = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};

  RobotiqFTModbus::RobotiqFTModbusConfig robotiq_config;
  robotiq_config.sensor_name = "RobotiqFTModbus";
  robotiq_config.fullpath = "";
  robotiq_config.print_flag = false;
  robotiq_config.publish_rate = 100.0;
  robotiq_config.noise_level = 0.0;
  robotiq_config.stall_threshold = 50;
  robotiq_config.Foffset = {0.0, 0.0, 0.0};
  robotiq_config.Toffset = {0.0, 0.0, 0.0};
  robotiq_config.Gravity = {0.0, 0.0, 0.0};
  robotiq_config.Pcom = {0.0, 0.0, 0.0};
  robotiq_config.WrenchSafety = {50.0, 50.0, 70.0, 0.5, 0.5, 0.5};
  robotiq_config.PoseSensorTool = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};

  Realsense::RealsenseConfig realsense_config;

  // ati.init(time0, ati_config);
  // realsense.init(time0, realsense_config);
  // robot.init(time0, arx_config);
  robotiq.init(time0, robotiq_config);

  while (true) {
    if (!robotiq.is_data_ready()) {
      sleep(0.5);
      continue;
    }
    // get data from sensors
    RUT::Vector6d wrench;
    robotiq.getWrenchSensor(wrench);
    std::cout << "t = " << timer.toc_ms() << ", wrench: " << wrench.transpose()
              << std::endl;
    usleep(10000);
  }
  return 0;
}
