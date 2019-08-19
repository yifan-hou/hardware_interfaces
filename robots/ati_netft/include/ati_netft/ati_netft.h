/*
    roscontrol hardware_interface wrapper for ati Netft RDT interface.

    Three methods to use this interface:
    1. use it as a hardware_interface. It provides force_torque_sensor_interface.
    2. read the public member _force and _torque, or call getWrench()
    3. use it as the netft_rdt_node. Messages and services are established.
*/
#pragma once
#include <hardware_interface/robot_hw.h>
#include <hardware_interface/force_torque_sensor_interface.h>


#include <ros/ros.h>

#include <iostream>
#include <fstream>
#include <string>
#include <chrono>
#include <cmath>
#include <unistd.h>
#include <pthread.h>
#include <memory>
#include <mutex>
#include <geometry_msgs/WrenchStamped.h>
#include <diagnostic_msgs/DiagnosticArray.h>
#include <diagnostic_updater/DiagnosticStatusWrapper.h>

// ATI Netft Specific headers
#include "netft_rdt_driver/netft_rdt_driver.h"

#include "hardware_interfaces/ft_interfaces.h"

using namespace std;

typedef std::chrono::high_resolution_clock Clock;

class ATINetft : public FTInterfaces {
  public:
    ATINetft();
    ~ATINetft();
    bool init(ros::NodeHandle& root_nh, Clock::time_point time0);
    /**
     * Get the sensor reading.
     *
     * @param  wrench  The wrench
     *
     * @return  0: no error.
     *   1: still waiting for new data. 2: dead stream.
     *   3: force is too big.
     */
    int getWrenchSensor(double *wrench) override;
    /**
     * Get the wrench in tool frame.
     *
     * @param      wrench_T  The wrench in tool frame
     *
     * @return     0: no error. 1: still waiting for new data. 2: dead stream.
     *             3: force is too big.
     */
    int getWrenchTool(double *wrench_T) override;
    /**
     * Get the tool wrench after tool weight compensation.
     *
     * @param[in]  pose          The Cartesian pose of the robot tool
     * @param      wrench_net_T  The net wrench in tool frame
     *
     * @return     0: no error. 1: still waiting for new data. 2: dead stream.
     *             3: force is too big.
     */
    int getWrenchNetTool(const double *pose, double *wrench_net_T) override;


    double *_force;
    double *_torque;
    double _publish_rate;

    Clock::time_point _time0; ///< high resolution timer.
    ofstream _file;
    bool _print_flag;

    // netft
    ros::Publisher _pub;
    ros::Publisher _diag_pub;
    std::shared_ptr<netft_rdt_driver::NetFTRDTDriver> _netft;
    string _frame_id;

    // monitor pausing of the data stream.
    // if the data is the same in 50 frames, the stream is considered dead.
    int _stall_counts;
    mutex _stall_counts_mtx;
    // set to true when a timeout happens. Set to false when a good data comes.
    bool _time_out_flag;
    mutex _time_out_flag_mtx;

  private:

    // thread
    pthread_t _thread;
};
