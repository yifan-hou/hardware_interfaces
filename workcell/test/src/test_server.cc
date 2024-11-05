#include "test/test_server.h"

TestServer::TestServer() {
  _timer.tic();
}
TestServer::~TestServer() {}

const double TestServer::get_test(double seconds) {
  // sleep for 1 second
  usleep(seconds * 1000000);
  return _timer.toc_ms();
}
