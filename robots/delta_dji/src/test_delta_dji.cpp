#include "delta_dji/delta_dji.h"
#include <chrono>

// #include <RobotUtilities/utilities.h>
// using namespace RUT;

#define PI 3.1416
using namespace std;

int main(int argc, char* argv[])
{
    ROS_INFO_STREAM("DeltaDJI test node starting");
    ros::init(argc, argv, "delta_dji_node");
    ros::NodeHandle hd;

    cout << "[test] Initializing DeltaDJI robot:" << endl;
    DeltaDJI robot(hd);

    std::this_thread::sleep_for(std::chrono::milliseconds(200));
    cout << "[test] DeltaDJI initialized." << endl;

    /**
     * Get
     */
    int code;
    double pos[3], joints[3];

    cout << "[test] Getting Joints:" << endl;
    if(!robot.getJoints(joints)) {
        cout << "[test] ERROR: failed to getJoints." << endl;
    } else {
        cout << "[test] getJoints: " << joints[0] << ", "
                << joints[1] << ", " << joints[2] << endl;
    }

    cout << "[test] Getting pos:" << endl;
    code = robot.getPos(pos);
    if(code == 0) {
        cout << "[test] getPos:" << pos[0] << ", "
                << pos[1] << ", " << pos[2] << endl;
    } else if(code == -1) {
        cout << "[test] ERROR: pos is not available." << endl;
    } else if(code == -2) {
        cout << "[test] ERROR: FK has no solution!" << endl;
    }

    cout << "[test] Next: set joints." << endl;
    cout << "[test] Press Enter to continue" << endl;
    getchar();
    /**
     * Set
     */
    cout << "[test] setting joints: " << endl;
    for (int i = 0; i < 3; ++i) {
        joints[i] = 35.0*PI/180.0;
    }

    if(!robot.setJoints(joints)) {
        cout << "[test] ERROR: failed to setJoints." << endl;
    } else {
        cout << "[test] setJoints is successful." << endl;
    }

    /**
     * Cartesian Motion
     */
    cout << "[test] Next: Cartesian motion." << endl;
    cout << "[test] Press Enter to continue" << endl;
    getchar();

    cout << "[test] setting pos: " << endl;
    code = robot.getPos(pos);
    if(code == 0) {
        cout << "[test] current pos:" << pos[0] << ", "
                << pos[1] << ", " << pos[2] << endl;
    } else if(code == -1) {
        cout << "[test] ERROR: pos is not available." << endl;
    } else if(code == -2) {
        cout << "[test] ERROR: FK has no solution!" << endl;
    }
    pos[2] -= 0.02;
    pos[1] += 0.02;
    cout << "[test] goal pos:" << pos[0] << ", " << pos[1] << ", " << pos[2] << endl;
    code = robot.setPos(pos);
    if(code == 0) {
        cout << "[test] setPos is successful." << endl;
        std::this_thread::sleep_for(std::chrono::milliseconds(200));
    } else if(code == -1){
        cout << "[test] ERROR: setPos failed." << endl;
    } else {
        cout << "[test] ERROR: setPos failed: no solution for IK" << endl;
    }

    code = robot.getPos(pos);
    if(code == 0) {
        cout << "[test] current pos:" << pos[0] << ", "
                << pos[1] << ", " << pos[2] << endl;
    } else if(code == -1) {
        cout << "[test] ERROR: pos is not available." << endl;
    } else if(code == -2) {
        cout << "[test] ERROR: FK has no solution!" << endl;
    }


    return 0;
}

