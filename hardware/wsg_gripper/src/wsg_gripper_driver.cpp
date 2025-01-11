#include "wsg_gripper/wsg_gripper_driver.h"
#include <stdio.h>
#include <stdlib.h>
#include <boost/asio.hpp>
#include <chrono>
#include <cstdint>
#include <cstring>  // For memcpy
#include <iostream>
#include <mutex>
#include <stdexcept>
#include <thread>
#include <vector>

/* ###########################################
 * ###   Define Default Values   #############
 * ###########################################
 */
#define DEBUG false

#ifndef SER_MSG_NUM_HEADER_BYTES
#define SER_MSG_NUM_HEADER_BYTES 3  //!< number of header bytes
#endif

#ifndef SER_MSG_HEADER_BYTE
#define SER_MSG_HEADER_BYTE 0xAA  //!< header byte value
#endif

/*!
 * Commands
 */
static short _HOMING = 0x20;      // homing movement
static short _DISCONNECT = 0x07;  // announce Disconnect -> stops every movement
static short _ACKFASTSTOP = 0x24;  // Acknowledgement FastStop

static short _EMPTY = 0xBA;   // Customized: do nothing, just trigger a feedback
static short _PDCTRL = 0xBB;  // Customized: PD control with force limit
static short _VELRESCTRL =
    0xBC;  // Customized: Velocity-resolved impedance control

std::array<char, 512> read_buffer;

//! Status codes
typedef enum {
  E_SUCCESS = 0,                  //!< No error
  E_NOT_AVAILABLE = 1,            //!< Device, service or data is not avialable
  E_NO_SENSOR = 2,                //!< No sensor connected
  E_NOT_INITIALIZED = 3,          //!< The device is not initialized
  E_ALREADY_RUNNING = 4,          //!< Service is already running
  E_FEATURE_NOT_SUPPORTED = 5,    //!< The asked feature is not supported
  E_INCONSISTENT_DATA = 6,        //!< One or more dependent parameters mismatch
  E_TIMEOUT = 7,                  //!< Timeout error
  E_READ_ERROR = 8,               //!< Error while reading from a device
  E_WRITE_ERROR = 9,              //!< Error while writing to a device
  E_INSUFFICIENT_RESOURCES = 10,  //!< No memory available
  E_CHECKSUM_ERROR = 11,          //!< Checksum error
  E_NO_PARAM_EXPECTED = 12,       //!< No parameters expected
  E_NOT_ENOUGH_PARAMS = 13,       //!< Not enough parameters
  E_CMD_UNKNOWN = 14,             //!< Unknown command
  E_CMD_FORMAT_ERROR = 15,        //!< Comand format error
  E_ACCESS_DENIED = 16,           //!< Access denied
  E_ALREADY_OPEN = 17,            //!< The interface is already open
  E_CMD_FAILED = 18,              //!< Command failed
  E_CMD_ABORTED = 19,             //!< Command aborted
  E_INVALID_HANDLE = 20,          //!< Invalid handle
  E_NOT_FOUND = 21,               //!< Device not found
  E_NOT_OPEN = 22,                //!< Device not open
  E_IO_ERROR = 23,                //!< I/O error
  E_INVALID_PARAMETER = 24,       //!< Invalid parameter
  E_INDEX_OUT_OF_BOUNDS = 25,     //!< Index out of bounds
  E_CMD_PENDING = 26,             //!< Command execution needs more time
  E_OVERRUN = 27,                 //!< Data overrun
  E_RANGE_ERROR = 28,             //!< Range error
  E_AXIS_BLOCKED = 29,            //!< Axis is blocked
  E_FILE_EXISTS = 30              //!< File already exists
} TStat;

void printErrorCode(TStat ec) {
  switch (ec) {
    case E_NOT_AVAILABLE:
      std::cout << "Received command error code: E_NOT_AVAILABLE" << std::endl;
      break;
    case E_NO_SENSOR:
      std::cout << "Received command error code: E_NO_SENSOR" << std::endl;
      break;
    case E_NOT_INITIALIZED:
      std::cout << "Received command error code: E_NOT_INITIALIZED"
                << std::endl;
      break;
    case E_ALREADY_RUNNING:
      std::cout << "Received command error code: E_ALREADY_RUNNING"
                << std::endl;
      break;
    case E_FEATURE_NOT_SUPPORTED:
      std::cout << "Received command error code: E_FEATURE_NOT_SUPPORTED"
                << std::endl;
      break;
    case E_INCONSISTENT_DATA:
      std::cout << "Received command error code: E_INCONSISTENT_DATA"
                << std::endl;
      break;
    case E_TIMEOUT:
      std::cout << "Received command error code: E_TIMEOUT" << std::endl;
      break;
    case E_READ_ERROR:
      std::cout << "Received command error code: E_READ_ERROR" << std::endl;
      break;
    case E_WRITE_ERROR:
      std::cout << "Received command error code: E_WRITE_ERROR" << std::endl;
      break;
    case E_INSUFFICIENT_RESOURCES:
      std::cout << "Received command error code: E_INSUFFICIENT_RESOURCES"
                << std::endl;
      break;
    case E_CHECKSUM_ERROR:
      std::cout << "Received command error code: E_CHECKSUM_ERROR" << std::endl;
      break;
    case E_NO_PARAM_EXPECTED:
      std::cout << "Received command error code: E_NO_PARAM_EXPECTED"
                << std::endl;
      break;
    case E_NOT_ENOUGH_PARAMS:
      std::cout << "Received command error code: E_NOT_ENOUGH_PARAMS"
                << std::endl;
      break;
    case E_CMD_UNKNOWN:
      std::cout << "Received command error code: E_CMD_UNKNOWN" << std::endl;
      break;
    case E_CMD_FORMAT_ERROR:
      std::cout << "Received command error code: E_CMD_FORMAT_ERROR"
                << std::endl;
      break;
    case E_ACCESS_DENIED:
      std::cout << "Received command error code: E_ACCESS_DENIED" << std::endl;
      break;
    case E_ALREADY_OPEN:
      std::cout << "Received command error code: E_ALREADY_OPEN" << std::endl;
      break;
    case E_CMD_FAILED:
      std::cout << "Received command error code: E_CMD_FAILED" << std::endl;
      break;
    case E_CMD_ABORTED:
      std::cout << "Received command error code: E_CMD_ABORTED" << std::endl;
      break;
    case E_INVALID_HANDLE:
      std::cout << "Received command error code: E_INVALID_HANDLE" << std::endl;
      break;
    case E_NOT_FOUND:
      std::cout << "Received command error code: E_NOT_FOUND" << std::endl;
      break;
    case E_NOT_OPEN:
      std::cout << "Received command error code: E_NOT_OPEN" << std::endl;
      break;
    case E_IO_ERROR:
      std::cout << "Received command error code: E_IO_ERROR" << std::endl;
      break;
    case E_INVALID_PARAMETER:
      std::cout << "Received command error code: E_INVALID_PARAMETER"
                << std::endl;
      break;
    case E_INDEX_OUT_OF_BOUNDS:
      std::cout << "Received command error code: E_INDEX_OUT_OF_BOUNDS"
                << std::endl;
      break;
    case E_CMD_PENDING:
      std::cout << "Received command error code: E_CMD_PENDING" << std::endl;
      break;
    case E_OVERRUN:
      std::cout << "Received command error code: E_OVERRUN" << std::endl;
      break;
    case E_RANGE_ERROR:
      std::cout << "Received command error code: E_RANGE_ERROR" << std::endl;
      break;
    case E_AXIS_BLOCKED:
      std::cout << "Received command error code: E_AXIS_BLOCKED" << std::endl;
      break;
    case E_FILE_EXISTS:
      std::cout << "Received command error code: E_FILE_EXISTS" << std::endl;
      break;
    default:
      break;
  }
}

static const unsigned short CRC_TABLE[256] = {
    0x0000, 0x1021, 0x2042, 0x3063, 0x4084, 0x50a5, 0x60c6, 0x70e7, 0x8108,
    0x9129, 0xa14a, 0xb16b, 0xc18c, 0xd1ad, 0xe1ce, 0xf1ef, 0x1231, 0x0210,
    0x3273, 0x2252, 0x52b5, 0x4294, 0x72f7, 0x62d6, 0x9339, 0x8318, 0xb37b,
    0xa35a, 0xd3bd, 0xc39c, 0xf3ff, 0xe3de, 0x2462, 0x3443, 0x0420, 0x1401,
    0x64e6, 0x74c7, 0x44a4, 0x5485, 0xa56a, 0xb54b, 0x8528, 0x9509, 0xe5ee,
    0xf5cf, 0xc5ac, 0xd58d, 0x3653, 0x2672, 0x1611, 0x0630, 0x76d7, 0x66f6,
    0x5695, 0x46b4, 0xb75b, 0xa77a, 0x9719, 0x8738, 0xf7df, 0xe7fe, 0xd79d,
    0xc7bc, 0x48c4, 0x58e5, 0x6886, 0x78a7, 0x0840, 0x1861, 0x2802, 0x3823,
    0xc9cc, 0xd9ed, 0xe98e, 0xf9af, 0x8948, 0x9969, 0xa90a, 0xb92b, 0x5af5,
    0x4ad4, 0x7ab7, 0x6a96, 0x1a71, 0x0a50, 0x3a33, 0x2a12, 0xdbfd, 0xcbdc,
    0xfbbf, 0xeb9e, 0x9b79, 0x8b58, 0xbb3b, 0xab1a, 0x6ca6, 0x7c87, 0x4ce4,
    0x5cc5, 0x2c22, 0x3c03, 0x0c60, 0x1c41, 0xedae, 0xfd8f, 0xcdec, 0xddcd,
    0xad2a, 0xbd0b, 0x8d68, 0x9d49, 0x7e97, 0x6eb6, 0x5ed5, 0x4ef4, 0x3e13,
    0x2e32, 0x1e51, 0x0e70, 0xff9f, 0xefbe, 0xdfdd, 0xcffc, 0xbf1b, 0xaf3a,
    0x9f59, 0x8f78, 0x9188, 0x81a9, 0xb1ca, 0xa1eb, 0xd10c, 0xc12d, 0xf14e,
    0xe16f, 0x1080, 0x00a1, 0x30c2, 0x20e3, 0x5004, 0x4025, 0x7046, 0x6067,
    0x83b9, 0x9398, 0xa3fb, 0xb3da, 0xc33d, 0xd31c, 0xe37f, 0xf35e, 0x02b1,
    0x1290, 0x22f3, 0x32d2, 0x4235, 0x5214, 0x6277, 0x7256, 0xb5ea, 0xa5cb,
    0x95a8, 0x8589, 0xf56e, 0xe54f, 0xd52c, 0xc50d, 0x34e2, 0x24c3, 0x14a0,
    0x0481, 0x7466, 0x6447, 0x5424, 0x4405, 0xa7db, 0xb7fa, 0x8799, 0x97b8,
    0xe75f, 0xf77e, 0xc71d, 0xd73c, 0x26d3, 0x36f2, 0x0691, 0x16b0, 0x6657,
    0x7676, 0x4615, 0x5634, 0xd94c, 0xc96d, 0xf90e, 0xe92f, 0x99c8, 0x89e9,
    0xb98a, 0xa9ab, 0x5844, 0x4865, 0x7806, 0x6827, 0x18c0, 0x08e1, 0x3882,
    0x28a3, 0xcb7d, 0xdb5c, 0xeb3f, 0xfb1e, 0x8bf9, 0x9bd8, 0xabbb, 0xbb9a,
    0x4a75, 0x5a54, 0x6a37, 0x7a16, 0x0af1, 0x1ad0, 0x2ab3, 0x3a92, 0xfd2e,
    0xed0f, 0xdd6c, 0xcd4d, 0xbdaa, 0xad8b, 0x9de8, 0x8dc9, 0x7c26, 0x6c07,
    0x5c64, 0x4c45, 0x3ca2, 0x2c83, 0x1ce0, 0x0cc1, 0xef1f, 0xff3e, 0xcf5d,
    0xdf7c, 0xaf9b, 0xbfba, 0x8fd9, 0x9ff8, 0x6e17, 0x7e36, 0x4e55, 0x5e74,
    0x2e93, 0x3eb2, 0x0ed1, 0x1ef0};

/*********************************************************************/
/*!
Calculates the CRC checksum of an array by using a table.
The start value for calculating the CRC should be set to 0xFFFF.
@param *data points to the byte array from which checksum should
be calculated
@param size size of the byte array
@param crc
value calculated over another array and start value
of the crc16 calculation
@return CRC16 checksum
*/
/*********************************************************************/
unsigned short checksum_update_crc16(unsigned char* data, unsigned int size,
                                     unsigned short crc) {
  unsigned long c;
  /* process each byte prior to checksum field */
  for (c = 0; c < size; c++) {
    crc = CRC_TABLE[(crc ^ *(data++)) & 0x00FF] ^ (crc >> 8);
  }
  return crc;
}

unsigned short checksum_update_crc16(const std::vector<unsigned char>& data,
                                     unsigned short crc) {
  /* process each byte prior to checksum field */
  for (unsigned long c = 0; c < data.size(); c++) {
    crc = CRC_TABLE[(crc ^ data[c]) & 0x00FF] ^ (crc >> 8);
  }
  return crc;
}

/*********************************************************************/
/*!
Builds a data packet from the given message.
You have to free the returned buffer, if you do not use it anymore.
@param *msg Pointer to the source message
@param *size
Returns the size of the created buffer
@return buffer containing the bytewise packet data or NULL in case
of an error.
*/
/*********************************************************************/
unsigned char* msg_build(TMESSAGE* msg, unsigned int* size) {
  unsigned char* buf;
  unsigned short chksum;
  unsigned int c, len;

  len = SER_MSG_NUM_HEADER_BYTES + 3 + 2 + msg->length;

  // allocate buffer (remember to delete buffer later!)
  //
  buf = new unsigned char[len];
  if (!buf) {
    *size = 0;
    return nullptr;
  }

  // Assemble the message header:
  for (c = 0; c < SER_MSG_NUM_HEADER_BYTES; c++)
    buf[c] = SER_MSG_HEADER_BYTE;
  buf[SER_MSG_NUM_HEADER_BYTES] = msg->id;  // Message ID
  buf[SER_MSG_NUM_HEADER_BYTES + 1] =
      msg->length & 0xFF;  // Msg. length low byte
  buf[SER_MSG_NUM_HEADER_BYTES + 2] =
      msg->length >> 8;  // Msg. length high byte

  // Copy payload to buffer:
  if (msg->length)
    memcpy(&buf[SER_MSG_NUM_HEADER_BYTES + 3], msg->data, msg->length);

  // Calculate the checksum over the header, include the preamble:
  chksum = checksum_update_crc16(
      buf, SER_MSG_NUM_HEADER_BYTES + 3 + msg->length, 0xFFFF);

  // Add checksum to message:
  buf[SER_MSG_NUM_HEADER_BYTES + 3 + msg->length] = chksum & 0xFF;
  buf[SER_MSG_NUM_HEADER_BYTES + 4 + msg->length] = chksum >> 8;
  *size = len;
  return (buf);
}

FBMessage WSGGripperDriver::msg_receive() {
  unsigned char sync = 0;

  // Syncing
  while (sync != 3) {
    unsigned char byte;
    {
      std::lock_guard<std::mutex> lock(_socket_mutex);
      _socket_ptr->recv_k_bytes(&byte, 1);
    }
    if (byte == 0xAA) {
      sync++;
    }
  }

  // Read command ID
  unsigned char command_id;
  {
    std::lock_guard<std::mutex> lock(_socket_mutex);
    _socket_ptr->recv_k_bytes(&command_id, 1);
  }

  // Read size
  unsigned short size;
  {
    std::lock_guard<std::mutex> lock(_socket_mutex);
    _socket_ptr->recv_k_bytes(reinterpret_cast<unsigned char*>(&size), 2);
  }

  // Read payload
  std::vector<unsigned char> payload_bytes(size);
  {
    std::lock_guard<std::mutex> lock(_socket_mutex);
    _socket_ptr->recv_k_bytes(payload_bytes.data(), size);
  }
  unsigned short status_code = payload_bytes[0] | (payload_bytes[1] << 8);
  // std::cout << "Received status code: " << status_code
  //           << ", payload_bytes[0]: " << int(payload_bytes[0])
  //           << ", payload_bytes[1]: " << int(payload_bytes[1]) << std::endl;

  // Read checksum
  unsigned short checksum;
  {
    std::lock_guard<std::mutex> lock(_socket_mutex);
    _socket_ptr->recv_k_bytes(reinterpret_cast<unsigned char*>(&checksum), 2);
  }

  // Validate checksum
  std::vector<unsigned char> all_bytes;

  // Add the command ID (1 byte)
  all_bytes.push_back(command_id);

  // Add the size (2 bytes, little-endian)
  all_bytes.push_back(static_cast<unsigned char>(size & 0xFF));  // Lower byte
  all_bytes.push_back(
      static_cast<unsigned char>((size >> 8) & 0xFF));  // Upper byte

  // Add the payload (as-is, already bytes)
  all_bytes.insert(all_bytes.end(), payload_bytes.begin(), payload_bytes.end());

  // Add the checksum (2 bytes, little-endian)
  all_bytes.push_back(
      static_cast<unsigned char>(checksum & 0xFF));  // Lower byte
  all_bytes.push_back(
      static_cast<unsigned char>((checksum >> 8) & 0xFF));  // Upper byte

  unsigned short header_checksum = 0x50F5;
  unsigned short msg_checksum =
      checksum_update_crc16(all_bytes, header_checksum);
  if (msg_checksum != 0) {
    throw std::runtime_error("Corrupted packet received from WSG");
  }

  // Construct and return result
  FBMessage result = {command_id, status_code,
                      std::vector<unsigned char>(payload_bytes.begin() + 2,
                                                 payload_bytes.end())};
  return result;
}

/*
 *  Constructor
 *  @param
 *      string ip = IP of the gripper
 *      string port = Port to connect with the gripper
 */
WSGGripperDriver::WSGGripperDriver(std::string ip, std::string port) {
  this->_IP = ip;
  this->_PORT = port;

  std::cout << "Starting connection" << std::endl;
  try {
    // create a new socket
    _socket_ptr = std::make_shared<Socket>(this->_IP, std::stoi(this->_PORT));
  } catch (const std::exception& e) {
    std::cerr << "[WSGCommunicator] Failed to create socket: " << e.what()
              << std::endl;
    throw;
  }
  std::this_thread::sleep_for(std::chrono::milliseconds(10));
}

WSGGripperDriver::~WSGGripperDriver() {}

unsigned char WSGGripperDriver::disconnect() {
  // create Message
  //
  _msg.id = _DISCONNECT;
  _msg.length = 0;
  _msg.data = nullptr;

  return sendMsg(&_msg);
}

unsigned char WSGGripperDriver::homing() {
#define DATA_SIZE_HOMING 1
  int i;
  unsigned char data[DATA_SIZE_HOMING];

  // data[0] = 0x00;  // homing in the default direction
  data[0] = 0x01;  // homing in the positive direction
  data[0] = 0x02;  // homing in the negative direction

  // create Message
  //
  _msg.id = _HOMING;
  _msg.length = DATA_SIZE_HOMING;
  _msg.data = data;

  return sendMsg(&_msg);
}

unsigned char WSGGripperDriver::ackFastStop() {
#define DATA_SIZE_ACK 3
  int i;
  unsigned char data[DATA_SIZE_ACK];

  data[0] = 0x61;  // 'a'
  data[1] = 0x63;  // 'c'
  data[2] = 0x6B;  // 'k'

  // create Message
  //
  _msg.id = _ACKFASTSTOP;
  _msg.length = DATA_SIZE_ACK;
  _msg.data = data;

  return sendMsg(&_msg);
}

unsigned char WSGGripperDriver::setPDControl(float pos, float kp, float kd,
                                             float force_limit) {
#define DATA_SIZE_PDC 20
  int i;
  unsigned char data[DATA_SIZE_PDC];
  unsigned char tmpFloat[4];

  // pos
  memcpy(tmpFloat, &pos, sizeof(float));
  for (i = 0; i < 4; i++) {
    data[i] = (unsigned char)tmpFloat[i];
  }

  // kp
  memcpy(tmpFloat, &kp, sizeof(float));
  for (i = 0; i < 4; i++) {
    data[(i + 4)] = tmpFloat[i];
  }

  // kd
  memcpy(tmpFloat, &kd, sizeof(float));
  for (i = 0; i < 4; i++) {
    data[(i + 8)] = tmpFloat[i];
  }

  // travel_force_limit
  memcpy(tmpFloat, &force_limit, sizeof(float));
  for (i = 0; i < 4; i++) {
    data[(i + 12)] = tmpFloat[i];
  }

  // blocking_force_limit
  memcpy(tmpFloat, &force_limit, sizeof(float));
  for (i = 0; i < 4; i++) {
    data[(i + 16)] = tmpFloat[i];
  }

  // create Message
  //
  _msg.id = _PDCTRL;
  _msg.length = DATA_SIZE_PDC;
  _msg.data = data;

  return sendMsg(&_msg);
}

unsigned char WSGGripperDriver::setVelResolvedControl(
    float pos_target, float force_target_feedback, float kp, float kf) {
#define DATA_SIZE_VELRES 16
  int i;
  unsigned char data[DATA_SIZE_VELRES];
  unsigned char tmpFloat[4];

  // pos_target
  memcpy(tmpFloat, &pos_target, sizeof(float));
  for (i = 0; i < 4; i++) {
    data[i] = (unsigned char)tmpFloat[i];
  }

  // force_target_feedback
  memcpy(tmpFloat, &force_target_feedback, sizeof(float));
  for (i = 0; i < 4; i++) {
    data[(i + 4)] = tmpFloat[i];
  }

  // kp
  memcpy(tmpFloat, &kp, sizeof(float));
  for (i = 0; i < 4; i++) {
    data[(i + 8)] = tmpFloat[i];
  }

  // kf
  memcpy(tmpFloat, &kf, sizeof(float));
  for (i = 0; i < 4; i++) {
    data[(i + 12)] = tmpFloat[i];
  }

  // create Message
  _msg.id = _VELRESCTRL;
  _msg.length = DATA_SIZE_VELRES;
  _msg.data = data;

  return sendMsg(&_msg);
}

unsigned char WSGGripperDriver::askForState() {
#define DATA_SIZE_EMPTY 4
  int i;
  unsigned char data[DATA_SIZE_EMPTY];
  unsigned char tmpFloat[4];

  float dummpy = 0.0;
  memcpy(tmpFloat, &dummpy, sizeof(float));
  for (i = 0; i < 4; i++) {
    data[i] = (unsigned char)tmpFloat[i];
  }

  // create Message
  //
  _msg.id = _EMPTY;
  _msg.length = DATA_SIZE_EMPTY;
  _msg.data = data;

  return sendMsg(&_msg);
}

void WSGGripperDriver::getAck(unsigned char expected_id) {
  _fbmsg = msg_receive();
  if (_fbmsg.command_id != expected_id) {
    throw std::runtime_error("Received unexpected command ID: " +
                             std::to_string(_fbmsg.command_id));
  }
}

WSGState WSGGripperDriver::getState(unsigned char expected_id) {
  _fbmsg = msg_receive();
  if (_fbmsg.command_id != expected_id) {
    throw std::runtime_error("Received unexpected command ID: " +
                             std::to_string(_fbmsg.command_id));
  }
  TStat status = (TStat)_fbmsg.status_code;
  std::vector<unsigned char> response_payload = _fbmsg.payload_bytes;

  if (status == E_CMD_UNKNOWN) {
    throw std::runtime_error(
        "Command unknown - make sure script (cmd_measure.lua) is running");
  } else if (status != E_SUCCESS) {
    throw std::runtime_error("Command failed");
  }
  if (response_payload.size() != 17) {
    throw std::runtime_error(
        "Response payload incorrect. Expected 17 bytes, "
        "received " +
        std::to_string(response_payload.size()));
  }

  // parse payload
  WSGState info;
  unsigned char state = response_payload[0];
  info.state = state;
  info.position = *reinterpret_cast<float*>(&response_payload[1]);
  info.velocity = *reinterpret_cast<float*>(&response_payload[5]);
  info.force_motor = *reinterpret_cast<float*>(&response_payload[9]);
  info.measure_timestamp =
      *reinterpret_cast<unsigned long*>(&response_payload[13]);
  info.is_moving = (state & 0x02) != 0;
  return info;
}

void WSGGripperDriver::printState(const WSGState& state) {
  std::cout << "State: " << std::endl;
  std::cout << "  state: " << state.state << std::endl;
  std::cout << "  position: " << state.position << std::endl;
  std::cout << "  velocity: " << state.velocity << std::endl;
  std::cout << "  force_motor: " << state.force_motor << std::endl;
  std::cout << "  measure_timestamp: " << state.measure_timestamp << std::endl;
  std::cout << "  is_moving: " << state.is_moving << std::endl;
}

unsigned char WSGGripperDriver::sendMsg(TMESSAGE* msg) {
  // send message
  // Convert message into byte sequence:
  unsigned int msg_size = 0;
  unsigned char* write_buffer;
  write_buffer = msg_build(msg, &msg_size);
  if (!write_buffer) {
    printErrorCode(E_INSUFFICIENT_RESOURCES);
    throw std::runtime_error("Failed to build message");
  }

  //    if(DEBUG) {
  //        std::cout << "Push TCP message: ";
  //        this->printHexArray(write_buffer, msg_size);
  //    }

  {
    std::lock_guard<std::mutex> lock(_socket_mutex);
    _socket_ptr->send(
        std::string(reinterpret_cast<char*>(write_buffer), msg_size));
  }
  delete write_buffer;

  return msg->id;
}