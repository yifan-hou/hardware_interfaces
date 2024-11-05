#pragma once

#include <unistd.h>
#include <csignal>
#include <deque>
#include <iostream>
#include <memory>
#include <mutex>
#include <thread>

#include <RobotUtilities/timer_linux.h>

class TestServer {
 public:
  TestServer();
  ~TestServer();

  const double get_test(double seconds);

 private:
  // timing
  RUT::Timer _timer;
};