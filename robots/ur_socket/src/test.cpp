#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <assert.h>
#include <string>

#include <ur_socket/ur_socket.h>
#include <RobotUtilities/utilities.h>

#define PI 3.1416
using namespace std;
using namespace RUT;

int main(int argc, char* argv[])
{
    ROS_INFO_STREAM("UR socket test node starting");
    ros::init(argc, argv, "ur_socket_node");
    ros::NodeHandle hd;

    Clock::time_point time0 = std::chrono::high_resolution_clock::now();

    URSocket *ur = URSocket::Instance();
    if (!ur->init(hd, time0)) {
        cout << "Unable to initialize ur socket." << endl;
        return -1;
    }

    /**
     * Read from robot
     */
    printf("Reading Pose:\n");
    double pose[7];
    ur->getCartesian(pose);
    printf("Pose:\t%.3f\t%.3f\t%.3f\t%.3f\t%.3f\t%.3f\t%.3f\n", pose[0],
            pose[1], pose[2], pose[3], pose[4], pose[5], pose[6]);

    /**
     * Send pose
     */
    Quaterniond q(pose[3], pose[4], pose[5], pose[6]);
    q = q*Eigen::AngleAxisd(20.0*PI/180.0, Eigen::Vector3d::UnitY());
    pose[3] = q.w();
    pose[4] = q.x();
    pose[5] = q.y();
    pose[6] = q.z();
    printf("Setting pose:\n");
    ur->setCartesian(pose);
    printf("done. Press Enter to continue.\n");
    getchar();
    q = q*Eigen::AngleAxisd(-20.0*PI/180.0, Eigen::Vector3d::UnitY());
    pose[3] = q.w();
    pose[4] = q.x();
    pose[5] = q.y();
    pose[6] = q.z();
    printf("Setting pose:\n");
    ur->setCartesian(pose);
    printf("Done.");

    return 0;
}