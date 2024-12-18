#include <time.h>
#include <exception>
#include <iostream>

#include "wsg_gripper/WSG50Controller.h"

// #include <boost/thread/thread.hpp>

#ifndef PI
#define PI 3.1415926535897932384626433832795
#endif

#ifndef RAD
#define RAD(A) ((A) * PI / 180.0)
#endif

#ifndef DEG
#define DEG(A) ((A) * 180.0 / PI)
#endif

int main(int argc, char* argv[]) {
  // create controller
  //
  WSG50Controller wsgController("192.168.1.101", "1000");

  while (!wsgController.ready()) {
    std::this_thread::sleep_for(std::chrono::milliseconds(200));
  }

  // homing
  wsgController.homing(80.0);

  while (!wsgController.ready()) {
    std::this_thread::sleep_for(std::chrono::milliseconds(200));
  }

  // // move to position
  std::cout << "Setting force limit." << std::endl;
  wsgController.setForceLimit(10.0);  // set the grasping-force-limit
  while (!wsgController.ready()) {
    std::this_thread::sleep_for(std::chrono::milliseconds(1000));
  }
  // std::cout << "Grasping." << std::endl;
  // wsgController.grasp(0.0, 400.0);
  // while (!wsgController.ready()) {
  //   std::this_thread::sleep_for(std::chrono::milliseconds(200));
  // }

  // std::cout << "Press Enter to continue." << std::endl;
  // getchar();
  // return 0;
  // std::cout << "Done." << std::endl;

  // test PD loop
  double stiffness = 1;  // N/mm
  double kd = 1;
  double pos_target = 0.0;
  double speedlimit = 400.0;
  double forcelimit = 50;

  auto time0 = std::chrono::duration_cast<std::chrono::milliseconds>(
                   std::chrono::system_clock::now().time_since_epoch())
                   .count();

  wsgController.grasp(pos_target, speedlimit);
  std::this_thread::sleep_for(std::chrono::milliseconds(2000));

  while (true) {
    // double pos = wsgController.getWidth();
    // double speed = wsgController.getSpeed();
    // double force = wsgController.getForce();

    // double pos_cmd = pos_target + force / stiffness - speed * kd;

    // wsgController.prePositionFingers(false, pos_cmd, speedlimit);

    // std::cout << "speed: " << speed << ", force: " << force
    //           << ", pos_cmd: " << pos_cmd << ", time: "
    //           << std::chrono::duration_cast<std::chrono::milliseconds>(
    //                  std::chrono::system_clock::now().time_since_epoch())
    //                      .count() -
    //                  time0
    //           << std::endl;
    auto time_now = std::chrono::duration_cast<std::chrono::milliseconds>(
                        std::chrono::system_clock::now().time_since_epoch())
                        .count() -
                    time0;
    int time_ms = time_now % 2000;
    if (time_ms > 1000) {
      forcelimit = 80;
    } else {
      forcelimit = 5;
    }
    wsgController.setForceLimit(forcelimit);
    while (!wsgController.ready()) {
      std::this_thread::sleep_for(std::chrono::milliseconds(2));
    }
    wsgController.grasp(pos_target, speedlimit);
    std::cout << "time_ms: " << time_ms << ", forcelimit: " << forcelimit
              << std::endl;

    while (!wsgController.ready()) {
      std::this_thread::sleep_for(std::chrono::milliseconds(2));
    }
    // std::this_thread::sleep_for(std::chrono::milliseconds(30));

    if (time_now > 10000) {
      break;
    }
  }

  return 0;
}