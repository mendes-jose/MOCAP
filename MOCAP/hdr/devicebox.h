#ifndef GENERICDEVICEBOX_H
#define GENERICDEVICEBOX_H

#ifdef _WIN32
#include <windows.h>
#endif
#include <iostream>
#include <process.h>
#include "srcTrCAN_IO_DLL.h"
#include "genericdevice.h"

using namespace Eigen;
using namespace std;

class DeviceBox: virtual public GenericDevice
{
private:
    unsigned char iAdapter; /**< TODO */
    tTrCAN_CanData dataPack;

public:
    DeviceBox ();
    DeviceBox ( const unsigned char &iAdapter, const int &boxID );
    virtual ~DeviceBox () = 0;
    
    virtual Matrix3d& readIMU ();
    Matrix3d& readIMU ( const unsigned char &sensorType );

    static void init ();
    static void closeAll ();
};

#endif // GENERICDEVICEBOX_H