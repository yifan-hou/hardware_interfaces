/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2008, Willow Garage, Inc.
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of the Willow Garage nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************/

#ifndef NETFT_RDT_DRIVER
#define NETFT_RDT_DRIVER

#include <RobotUtilities/timer_linux.h>

#include <boost/asio.hpp>
#include <boost/system/error_code.hpp>
#include <boost/thread/condition.hpp>
#include <boost/thread/mutex.hpp>
#include <boost/thread/thread.hpp>
#include <string>

namespace netft_rdt_driver {

struct RDTRecord {
  uint32_t rdt_sequence_;
  uint32_t ft_sequence_;
  uint32_t status_;
  int32_t fx_;
  int32_t fy_;
  int32_t fz_;
  int32_t tx_;
  int32_t ty_;
  int32_t tz_;

  // constexpr std::size_t RDT_RECORD_SIZE = 36;
#define RDT_RECORD_SIZE 36
  // :-( but I don't want to force C++11...

  void unpack(const uint8_t* buffer);
  static uint32_t unpack32(const uint8_t* buffer);
};

struct WrenchData {
  double fx;
  double fy;
  double fz;
  double tx;
  double ty;
  double tz;
  uint seq;
  RUT::TimePoint stamp;
};

class Writer {
 public:
  Writer(boost::asio::ip::udp::socket& socket_);

  template <class Buffer>
  boost::system::error_code write(Buffer const& buffer) {
    this->socket_.async_send(
        buffer, boost::bind(&Writer::write_handler, this,
                            boost::asio::placeholders::error,
                            boost::asio::placeholders::bytes_transferred));
    boost::unique_lock<boost::mutex> lock(this->mutex);
    this->cond.wait(lock);

    return this->ec;
  }

 private:
  void write_handler(boost::system::error_code const& ec, std::size_t);

  boost::asio::ip::udp::socket& socket_;
  boost::system::error_code ec;

  boost::condition_variable cond;
  boost::mutex mutex;
};

class NetFTRDTDriver {
 public:
  // Start receiving data from NetFT device
  NetFTRDTDriver(const std::string& address, double counts_per_force,
                 double counts_per_torque);

  ~NetFTRDTDriver();

  //! Get newest RDT data from netFT device
  void getData(WrenchData& data);

  //! Wait for new NetFT data to arrive.
  // Returns true if new data has arrived, false it function times out
  bool waitForNewData(void);

  boost::system::error_code resetThresholdLatch();

  boost::system::error_code setSoftwareBias();

 protected:
  // void recvThreadFunc(void);

  void recvData(boost::system::error_code const& ec,
                std::size_t bytes_transferred);

  uint8_t buffer[RDT_RECORD_SIZE + 1];

  //! Asks NetFT to start streaming data.
  void startStreaming(void);

  void run();

  enum { RDT_PORT = 49152 };
  std::string address_;

  boost::asio::io_service io_service_;
  boost::asio::ip::udp::socket socket_;
  boost::mutex mutex_;
  boost::thread recv_thread_;
  boost::condition condition_;
  volatile bool stop_recv_thread_;
  //! True if recv loop is still running
  bool recv_thread_running_;
  //! Set if recv thread exited because of error
  std::string recv_thread_error_msg_;

  //! Newest data received from netft device
  WrenchData new_data_;
  //! Count number of received <good> packets
  unsigned packet_count_;
  //! Count of lost RDT packets using RDT sequence number
  unsigned lost_packets_;
  //! Counts number of out-of-order (or duplicate) received packets
  unsigned out_of_order_count_;
  //! Incremental counter for wrench header
  unsigned seq_counter_;

  //! Scaling factor for converting raw force values from device into Newtons
  double force_scale_;
  //! Scaling factor for converting raw torque values into Newton*meters
  double torque_scale_;

  //! to keep track of out-of-order or duplicate packet
  uint32_t last_rdt_sequence_;
  //! to keep track of any error codes reported by netft
  uint32_t system_status_;
};

}  // end namespace netft_rdt_driver

#endif  // NETFT_RDT_DRIVER
