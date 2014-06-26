#ifndef GENERICDEVICESPH_H
#define GENERICDEVICESPH_H

#define _USE_MATH_DEFINES

#include <arpa/inet.h>  /* for sockaddr_in and inet_ntoa() */
#include "genericdevice.h"
#include <cstdio> /* for sprintf */

class DeviceSPh: virtual public GenericDevice
{
private:
    int server_socket;
    int client_socket;
    struct sockaddr_in clientAddr;
    struct sockaddr_in serverAddr;
    char buffer[128];
    char *pch;
    char sensorTypeAsCString[2];
    unsigned int clientLen; 
		int ret;

    int mAccept (int s, struct sockaddr *addr, unsigned int *addrlen, int timeout);

public:
    virtual Eigen::Matrix3d& readIMU ();

    DeviceSPh ();
    DeviceSPh ( const int &sphoneID );
    virtual ~DeviceSPh () = 0;

    static void init ();
    static void closeAll ();
};

#endif // GENERICDEVICESPH_H
