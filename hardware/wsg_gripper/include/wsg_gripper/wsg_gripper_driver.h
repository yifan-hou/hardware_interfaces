#pragma once

#include "hardware_interfaces/socket.hpp"

#include <memory>
#include <mutex>
#include <queue>
#include <string>

/*!
 * Manufacturer's default network config
 */
#define DEFAULTIP "192.168.1.20"
#define DEFAULTPORT "1000"

typedef struct {
  unsigned short length;  //!< Length of message's payload in bytes
                          //!  (0, if the message has no payload)
  unsigned char id;       //!< ID of the message
  unsigned char* data;    //!< Pointer to the message's payload
} TMESSAGE;               //!< command message format

struct FBMessage {
  unsigned char command_id;    // 1 byte
  unsigned short status_code;  // 2 bytes
  std::vector<unsigned char> payload_bytes;
};

struct WSGState {
  unsigned char state;
  float position;
  float velocity;
  float force_motor;
  unsigned long measure_timestamp;
  bool is_moving;
};

class WSGGripperDriver {

 public:
  // Connection Manager
  //
  WSGGripperDriver(void);
  WSGGripperDriver(std::string ip, std::string port);
  virtual ~WSGGripperDriver(void);

  unsigned char disconnect();
  unsigned char homing();
  unsigned char ackFastStop();

  unsigned char setPDControl(float pos, float kp, float kd, float force_limit);

  /// @brief One update of the velocity-resolved impedance control
  /// @param pos_target target position.
  /// @param force_target_feedback target force minus force feedback.
  /// @param kp
  /// @param kf
  /// @return command id.
  unsigned char setVelResolvedControl(float pos_target,
                                      float force_target_feedback, float kp,
                                      float kf);
  unsigned char askForState();  // just to trigger a feedback package

  /// @brief Read a package with a specific id. Must be called after homing(), ackFastStop().
  /// @param expected_id expected command id
  void getAck(unsigned char expected_id);
  /// @brief Read states from a package with a specific id. Must be called after setPDControl(), setVelResolvedControl(), askForState().
  /// @param expected_id
  /// @return
  WSGState getState(unsigned char expected_id);

  static void printState(const WSGState& state);

 private:
  FBMessage msg_receive();
  unsigned char sendMsg(TMESSAGE* msg);

  std::shared_ptr<Socket> _socket_ptr;
  std::mutex _socket_mutex;

  std::string _IP;
  std::string _PORT;

  TMESSAGE _msg;
  FBMessage _fbmsg;
};