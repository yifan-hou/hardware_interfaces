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

#include "netft_rdt_driver/netft_rdt_driver.h"

#include <stdint.h>

#include <exception>
#include <iostream>

using boost::asio::ip::udp;

namespace netft_rdt_driver {

uint32_t RDTRecord::unpack32(const uint8_t *buffer) {
  return (uint32_t(buffer[0]) << 24) | (uint32_t(buffer[1]) << 16) |
         (uint32_t(buffer[2]) << 8) | (uint32_t(buffer[3]) << 0);
}

void RDTRecord::unpack(const uint8_t *buffer) {
  rdt_sequence_ = unpack32(buffer + 0);
  ft_sequence_ = unpack32(buffer + 4);
  status_ = unpack32(buffer + 8);
  fx_ = unpack32(buffer + 12);
  fy_ = unpack32(buffer + 16);
  fz_ = unpack32(buffer + 20);
  tx_ = unpack32(buffer + 24);
  ty_ = unpack32(buffer + 28);
  tz_ = unpack32(buffer + 32);
}

struct RDTCommand {
  uint16_t command_header_;
  uint16_t command_;
  uint32_t sample_count_;

  RDTCommand() : command_header_(HEADER) {
    // empty
  }

  enum { HEADER = 0x1234 };

  // Possible values for command_
  enum {
    CMD_STOP_STREAMING = 0,
    CMD_START_HIGH_SPEED_STREAMING = 2,
    CMD_RESET_THRESHOLD_LATCH = 0x0041,
    CMD_SET_SOFTWARE_BIAS = 0x0042
    // More command values are available but are not used by this driver
  };

  // Special values for sample count
  enum { INFINITE_SAMPLES = 0 };

  enum { RDT_COMMAND_SIZE = 8 };

  //! Packet structure into buffer for network transport
  //  Buffer should be RDT_COMMAND_SIZE
  void pack(uint8_t *buffer) const;
};

void RDTCommand::pack(uint8_t *buffer) const {
  // Data is big-endian
  buffer[0] = (command_header_ >> 8) & 0xFF;
  buffer[1] = (command_header_ >> 0) & 0xFF;
  buffer[2] = (command_ >> 8) & 0xFF;
  buffer[3] = (command_ >> 0) & 0xFF;
  buffer[4] = (sample_count_ >> 8) & 0xFF;
  buffer[5] = (sample_count_ >> 0) & 0xFF;
  buffer[6] = (sample_count_ >> 8) & 0xFF;
  buffer[7] = (sample_count_ >> 0) & 0xFF;
}

NetFTRDTDriver::NetFTRDTDriver(const std::string &address)
    : address_(address),
      socket_(io_service_),
      stop_recv_thread_(false),
      recv_thread_running_(false),
      packet_count_(0),
      lost_packets_(0),
      out_of_order_count_(0),
      seq_counter_(0),
      last_rdt_sequence_(0),
      system_status_(0) {
  // Construct UDP socket
  udp::endpoint netft_endpoint(
      boost::asio::ip::address_v4::from_string(address), RDT_PORT);
  socket_.open(udp::v4());
  socket_.connect(netft_endpoint);

  // TODO : Get/Set Force/Torque scale for device
  // Force/Sclae is based on counts per force/torque value from device
  // these value are manually read from device webserver, but in future they
  // may be collected using http get requests
  static const double counts_per_force = 1000000;
  static const double counts_per_torque = 1000000;
  force_scale_ = 1.0 / counts_per_force;
  torque_scale_ = 1.0 / counts_per_torque;

  // Start receive thread
  socket_.async_receive(
      boost::asio::buffer(buffer, RDT_RECORD_SIZE + 1),
      boost::bind(&NetFTRDTDriver::recvData, this,
                  boost::asio::placeholders::error,
                  boost::asio::placeholders::bytes_transferred));
  recv_thread_ = boost::thread(&NetFTRDTDriver::run, this);

  // Since start steaming command is sent with UDP packet,
  // the packet could be lost, retry startup 10 times before giving up
  for (int i = 0; i < 10; ++i) {
    startStreaming();
    if (waitForNewData()) break;
  }
  {
    boost::unique_lock<boost::mutex> lock(mutex_);
    if (packet_count_ == 0) {
      throw std::runtime_error("No data received from NetFT device");
    }
  }
}

NetFTRDTDriver::~NetFTRDTDriver() {
  // TODO stop transmission,
  // stop thread
  stop_recv_thread_ = true;
  if (!recv_thread_.timed_join(boost::posix_time::time_duration(0, 0, 1, 0))) {
    std::cerr << "Interrupting recv thread" << std::endl;
    recv_thread_.interrupt();
    if (!recv_thread_.timed_join(
            boost::posix_time::time_duration(0, 0, 1, 0))) {
      std::cerr << "Failed second join to recv thread" << std::endl;
    }
  }
  socket_.close();
}

bool NetFTRDTDriver::waitForNewData() {
  // Wait upto 100ms for new data
  bool got_new_data = false;
  {
    boost::mutex::scoped_lock lock(mutex_);
    unsigned current_packet_count = packet_count_;
    condition_.timed_wait(lock, boost::posix_time::milliseconds(100));
    got_new_data = packet_count_ != current_packet_count;
  }

  return got_new_data;
}

void NetFTRDTDriver::startStreaming(void) {
  // Command NetFT to start data transmission
  RDTCommand start_transmission;
  start_transmission.command_ = RDTCommand::CMD_START_HIGH_SPEED_STREAMING;
  start_transmission.sample_count_ = RDTCommand::INFINITE_SAMPLES;
  // TODO change buffer into boost::array
  uint8_t buffer[RDTCommand::RDT_COMMAND_SIZE];
  start_transmission.pack(buffer);
  socket_.send(boost::asio::buffer(buffer, RDTCommand::RDT_COMMAND_SIZE));
}

boost::system::error_code NetFTRDTDriver::resetThresholdLatch() {
  RDTCommand reset_latch;
  reset_latch.command_ = RDTCommand::CMD_RESET_THRESHOLD_LATCH;
  reset_latch.sample_count_ = 0;
  // TODO change buffer into boost::array
  uint8_t buffer[RDTCommand::RDT_COMMAND_SIZE];
  reset_latch.pack(buffer);
  Writer writer(this->socket_);
  return writer.write(
      boost::asio::buffer(buffer, RDTCommand::RDT_COMMAND_SIZE));
}

boost::system::error_code NetFTRDTDriver::setSoftwareBias() {
  RDTCommand set_bias;
  set_bias.command_ = RDTCommand::CMD_SET_SOFTWARE_BIAS;
  set_bias.sample_count_ = 0;
  // TODO change buffer into boost::array
  uint8_t buffer[RDTCommand::RDT_COMMAND_SIZE];
  set_bias.pack(buffer);
  Writer writer(this->socket_);
  return writer.write(
      boost::asio::buffer(buffer, RDTCommand::RDT_COMMAND_SIZE));
}

void NetFTRDTDriver::recvData(boost::system::error_code const &ec,
                              std::size_t bytes_transferred) {
  if (ec) {
    recv_thread_running_ = false;
    {
      boost::unique_lock<boost::mutex> lock(mutex_);
      recv_thread_error_msg_ = ec.message();
    }
  } else {
    RDTRecord rdt_record;
    WrenchData tmp_data;
    if (bytes_transferred != RDT_RECORD_SIZE) {
      std::cerr << "Receive size of " << int(bytes_transferred)
                << " bytes does not match expected size of "
                << int(RDT_RECORD_SIZE) << std::endl;
    } else {
      rdt_record.unpack(buffer);
      if (rdt_record.status_ != system_status_) {
        // Latch any system status error code
        boost::unique_lock<boost::mutex> lock(mutex_);
        system_status_ = rdt_record.status_;
      }
      int32_t seqdiff = int32_t(rdt_record.rdt_sequence_ - last_rdt_sequence_);
      last_rdt_sequence_ = rdt_record.rdt_sequence_;
      if (seqdiff < 1) {
        boost::unique_lock<boost::mutex> lock(mutex_);
        // Don't use data that is old
        ++out_of_order_count_;
      } else {
        tmp_data.seq = seq_counter_++;
        tmp_data.stamp = RUT::Clock::now();
        tmp_data.fx = double(rdt_record.fx_) * force_scale_;
        tmp_data.fy = double(rdt_record.fy_) * force_scale_;
        tmp_data.fz = double(rdt_record.fz_) * force_scale_;
        tmp_data.tx = double(rdt_record.tx_) * torque_scale_;
        tmp_data.ty = double(rdt_record.ty_) * torque_scale_;
        tmp_data.tz = double(rdt_record.tz_) * torque_scale_;
        {
          boost::unique_lock<boost::mutex> lock(mutex_);
          new_data_ = tmp_data;
          lost_packets_ += (seqdiff - 1);
          ++packet_count_;
          condition_.notify_all();
        }
      }
    }
    if (!stop_recv_thread_) {
      socket_.async_receive(
          boost::asio::buffer(buffer, RDT_RECORD_SIZE + 1),
          boost::bind(&NetFTRDTDriver::recvData, this,
                      boost::asio::placeholders::error,
                      boost::asio::placeholders::bytes_transferred));
    }
  }
}

void NetFTRDTDriver::run() { io_service_.run(); }

Writer::Writer(boost::asio::ip::udp::socket &socket_) : socket_(socket_) {
  // empty
}

void Writer::write_handler(boost::system::error_code const &ec, std::size_t) {
  this->ec = ec;
  this->cond.notify_all();
}

void NetFTRDTDriver::getData(WrenchData &data) {
  {
    boost::unique_lock<boost::mutex> lock(mutex_);
    data = new_data_;
  }
}

}  // end namespace netft_rdt_driver
