/**
 * URSocket: interface for socket communication with UR e robot. Realtime socket
 * format is according to e5.4. Maintains a 500Hz communication with the robot.
 *
 * Author:
 *      Yifan Hou <yifanh@cmu.edu>
 */

#ifndef _UR_SOCKET_HEADER_
#define _UR_SOCKET_HEADER_

#include <thread>
#include <mutex>
#include <sys/socket.h>
#include <arpa/inet.h>

#include <ros/ros.h>

#include "hardware_interfaces/robot_interfaces.h"



typedef std::chrono::high_resolution_clock Clock;


class URSocket : public RobotInterfaces {
public:
    /// for singleton implementation
    static URSocket* Instance();

    /**
     * Initialize socket communication. Create a thread to run the 500Hz
     * communication with URe. The function will read parameters from ROS
     * parameter server.
     *
     * @param      root_nh  ROS node handle. Used to access parameters.
     * @param[in]  time0    Start time. Time will count from this number.
     *
     * @return     True if success.
     */
    int init(ros::NodeHandle& root_nh, Clock::time_point time0);

    bool getCartesian(double *pose) override;
    bool setCartesian(const double *pose) override;

    /**
     * Not implemented yet
     */
    bool getJoints(double *joints) override;
    bool setJoints(const double *joints) override;


private:
    /**
     * For singleton implementation
     */
    static URSocket* pinstance;
    URSocket();
    URSocket(const URSocket&){}
    URSocket& operator= (const URSocket&){}
    ~URSocket();

    // callbacks
    void UR_STATE_MONITOR();

    // motion parameters
    float _move_para_t;
    float _move_para_lookahead;
    float _move_para_gain;

    char *_send_buffer;

    double *_pose;
    double *_joints;
    std::mutex _mtx_pose;
    std::mutex _mtx_joint;

    int _sock;
    std::mutex _mtx_sock;
    std::thread _thread;
    Clock::time_point _time0;

    bool _stop_monitoring;
};

#endif