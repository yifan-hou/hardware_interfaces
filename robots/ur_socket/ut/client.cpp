// Client side C/C++ program to demonstrate Socket programming
#include <stdio.h>
#include <stdlib.h>
#include <sys/socket.h>
#include <arpa/inet.h>
#include <unistd.h>
#include <string.h>
#include <assert.h>
#include <string>

#include <RobotUtilities/utilities.h>

#define PIf 3.1416
#define PORT 30003

using namespace std;
using namespace RUT;

typedef union {
  double d;
  unsigned char bytes[sizeof(double)];
} DOUBLE_UNION;


void ReadXBytes(int socket, unsigned int x, void* buffer)
{
    int bytesRead = 0;
    int result;
    while (bytesRead < x)
    {
        result = read(socket, buffer + bytesRead, x - bytesRead);
        if (result < 1 )
        {
            // Throw your error.
        }

        bytesRead += result;
    }
}

int buffToInteger(unsigned char * buffer)
{
    int a = int((unsigned char)(buffer[0]) << 24 |
                (unsigned char)(buffer[1]) << 16 |
                (unsigned char)(buffer[2]) << 8 |
                (unsigned char)(buffer[3]));
    return a;
}

double reverseDouble(const char *data){
    double result;
    char *dest = (char *)&result;
    for(int i=0; i<sizeof(double); i++)
        dest[i] = data[sizeof(double)-i-1];
    return result;
}

double buffToDouble(uint8_t * buff){
    double value;
    memcpy(&value,buff,sizeof(double));
    // use this to convert big endian (Network) to little endian (host)
    // You can use the following code to check whether your system is little endian or not:
    //      unsigned int i = 1;
    //      char *c = (char*)&i;
    //      if (*c)
    //          printf("Little endian");
    //      else
    //          printf("Big endian");
    // For big endian system, there is no need to call reverseDouble.
    return reverseDouble((char *)&value);
}

int main(int argc, char const *argv[])
{
    int sock = 0;
    struct sockaddr_in serv_addr;
    if ((sock = socket(AF_INET, SOCK_STREAM, 0)) < 0)
    {
        printf("\n Socket creation error \n");
        return -1;
    }

    serv_addr.sin_family = AF_INET;
    serv_addr.sin_port = htons(PORT);

    // Convert IPv4 and IPv6 addresses from text to binary form
    if(inet_pton(AF_INET, "192.168.1.98", &serv_addr.sin_addr)<=0)
    {
        printf("\nInvalid address/ Address not supported \n");
        return -1;
    }

    if (connect(sock, (struct sockaddr *)&serv_addr, sizeof(serv_addr)) < 0)
    {
        printf("\nConnection Failed \n");
        return -1;
    }

    /**
     * Send pose
     */
    // float pose[6] = {-806.8, -234.17, -335.42, 1.563, 0.001, 0.001};
    // char buffer1 [100];
    // sprintf (buffer1, "movej(p[ %f, %f, %f, %f, %f, %f], a = %f, v = %f)\n",
    //     pose[0]/1000.0, pose[1]/1000.0, pose[2]/1000.0,
    //     pose[3], pose[4], pose[5], 0.4, 0.6);

    // send(sock , buffer1 , strlen(buffer1) , 0 );
    // printf("Pose sent\n");


    /**
     * Send joint
     */
    // float joint_command_deg[6] = {94.28, -81.68, 119.96, -129.11, 275.54, -0.0};
    // float joint_command_rad[6];
    // for (int i = 0; i < 6; ++i) joint_command_rad[i] = joint_command_deg[i]*PIf/180.0f;

    // char buffer2 [100];
    // sprintf (buffer2, "movej([ %f, %f, %f, %f, %f, %f], a = %f, v = %f)\n",
    //     joint_command_rad[0], joint_command_rad[1], joint_command_rad[2],
    //     joint_command_rad[3], joint_command_rad[4], joint_command_rad[5],
    //     0.4, 0.6);

    // send(sock , buffer2 , strlen(buffer2) , 0 );
    // printf("Pose sent\n");

    /**
     * Read from robot
     */
    unsigned int length = 0;
    assert(sizeof(length) == 4);
    // we assume that sizeof(length) will return 4 here.
    for (int i = 0; i < 10; ++i) {
        unsigned char buffer[1116];
        ReadXBytes(sock, 1116, (void*)(buffer));

        // decode the message
        unsigned char *pointer = buffer;
        int length = buffToInteger(pointer);
        pointer += 12;
        double j1 = buffToDouble(pointer);
        pointer += 8;
        double j2 = buffToDouble(pointer);
        pointer += 8;
        double j3 = buffToDouble(pointer);
        pointer += 8;
        double j4 = buffToDouble(pointer);
        pointer += 8;
        double j5 = buffToDouble(pointer);
        pointer += 8;
        double j6 = buffToDouble(pointer);
        pointer += 8;
        pointer += 384;
        double x = buffToDouble(pointer);
        pointer += 8;
        double y = buffToDouble(pointer);
        pointer += 8;
        double z = buffToDouble(pointer);
        pointer += 8;
        double Rx = buffToDouble(pointer);
        pointer += 8;
        double Ry = buffToDouble(pointer);
        pointer += 8;
        double Rz = buffToDouble(pointer);

        // convert to pose
        Vector3d ax;
        ax << Rx, Ry, Rz;
        double angle = ax.norm();
        Quaterniond q(Eigen::AngleAxisd(angle, ax.normalized()));

        // printf("x: %f, y: %f, z: %f, rx: %f, ry: %f, rz: %f\n",
        //         x, y, z, Rx, Ry, Rz);
        printf("p: %f, %f, %f, q: %f, %f, %f, %f\n", x*1000.0, y*1000.0, z*1000.0, q.w(), q.x(), q.y(), q.z());

    }

    return 0;
}