#ifndef GENERICDEVICESPH_H
#define GENERICDEVICESPH_H

#pragma comment(lib, "ws2_32.lib")
#define _USE_MATH_DEFINES

#ifdef _WIN32
#include <windows.h>
#endif
#include "genericdevice.h"

class DeviceSPh: virtual public GenericDevice
{
private:
    SOCKET server_socket;
    SOCKET client_socket;
    struct sockaddr_in clientAddr;
    struct sockaddr_in serverAddr;
    char buffer[128];
    char *pch;
    char sensorTypeAsCString[2];
    int clientLen, ret;

    SOCKET mAccept (SOCKET s, struct sockaddr *addr, int *addrlen, int timeout);

public:
    virtual Eigen::Matrix3d& readIMU ();

    DeviceSPh ();
    DeviceSPh ( const int &sphoneID );
    virtual ~DeviceSPh () = 0;

    static void init ();
    static void closeAll ();
};

#endif // GENERICDEVICESPH_H