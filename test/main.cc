#include <iostream>
// #include <memory>
#include <mutex>

#include <RobotUtilities/TimerLinux.h>
#include <RobotUtilities/utilities.h>
#include <arx_can/arx_can.h>
#include <ati_netft/ati_netft.h>
#include <realsense/realsense.h>

int main() {
  ATINetft ati;
  ARXCAN robot;
  Realsense realsense;

  RUT::Timer timer;
  RUT::TimePoint time0 = timer.tic();

  ARXCAN::ARXCANConfig arx_config;
  arx_config.can_interface = "can0";
  arx_config.urdf_path = "";
  arx_config.send_receive_in_background = false;
  arx_config.enable_gravity_compensation = false;
  arx_config.reset_to_home_upon_start = true;

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

  Realsense::RealsenseConfig realsense_config;

  ati.init(time0, ati_config);
  realsense.init(time0, realsense_config);
  robot.init(time0, arx_config);

  while (true) {
    // get data from sensors
    RUT::Vector6d wrench;
    ati.getWrenchSensor(wrench);
    std::cout << "t = " << timer.toc_ms() << ", wrench: " << wrench.transpose()
              << std::endl;
  }
  return 0;
}
