#include "devicesph.h"

const unsigned ALL_THREE = 3;

DeviceSPh::DeviceSPh (): GenericDevice() {}

DeviceSPh::DeviceSPh ( const int &sphoneID ):
    GenericDevice()
{
    this->deviceID = sphoneID;
    // Create a TCP socket
    server_socket = socket (AF_INET, SOCK_STREAM, IPPROTO_TCP); // Create socket

    if (server_socket == 0)
    {
        std::cerr << "WARNING: fail to create a TCP socket for device " << int(sphoneID) << std::endl;
        status = false;
        return;
    }

    serverAddr.sin_family = AF_INET;
    serverAddr.sin_addr.s_addr = htonl(INADDR_ANY);
    serverAddr.sin_port = htons(Common::TCPSERVERBASEPORT+sphoneID);

    if ( ::bind(server_socket, (struct sockaddr *)&serverAddr, sizeof(serverAddr)) == 0 )
    {
        std::cerr << "WARNING: fail to bind to the device "<< int(sphoneID) << " socket" << std::endl;
        status = false;
        return;
    }
            
    if ( listen(server_socket, 14) == 0 )
    {
        std::cerr << "WARNING: fail to start listen to device " << int(sphoneID) << " connection request" << std::endl;
        status = false;
        return;
    }
    
    std::clog <<"Waiting for connection request from device " << int(this->deviceID) << std::endl;

    clientLen = sizeof(clientAddr);
    if ( (client_socket = mAccept(server_socket, (struct sockaddr *)&clientAddr, &clientLen, Common::TCPSERVERLISTENINGTIMEOUT/1000)) == 0 )
    {
        std::cerr << "WARNING: fail to accept connection from device " << int(sphoneID) << " or listening timeout" << std::endl;
        status = false;
        return;
    }
    
    std::clog <<"Connection with device " << int(this->deviceID) << " accepted" << std::endl;
    close(server_socket);

    status = true;
}

Eigen::Matrix3d& DeviceSPh::readIMU ()
{
    sprintf(sensorTypeAsCString, "%d", ALL_THREE);
//    sensorTypeAsCString[1] = '\n';

    // SENDING DATA REQUEST
    if ( sendto ( client_socket, sensorTypeAsCString, sizeof(sensorTypeAsCString), 0, (struct sockaddr *)&clientAddr, sizeof(clientLen) ) == 0 )
    {
        std::cerr << "WARNING: fail to send data request to device " << deviceID << std::endl;
        status = false;
        return imuData;
    }

    // RECIEVING DATA
		// wait a while before trying to get the requested data
    nanosleep ( (struct timespec[]){{0, Common::RECVSMARTPHONEDATAWAITINGTIME*1000000}}, NULL);
    ret = recvfrom ( client_socket, buffer, sizeof(buffer), 0, (struct sockaddr *)&clientAddr, &clientLen );
    for ( int counter=0; ret < 1 && counter*Common::RECVSMARTPHONEDATAWAITINGTIME < Common::RECVSMARTPHONEDATATIMEOUT; counter++ )
    {
        nanosleep ( (struct timespec[]){{0, Common::RECVSMARTPHONEDATAWAITINGTIME*1000000}}, NULL);
        ret = recvfrom ( client_socket, buffer, sizeof(buffer), 0, (struct sockaddr *)&clientAddr, &clientLen );
    }

    if ( ret < 1 )
    {
        std::cerr << "WARNING: fail to receive IMU data from device " << deviceID << std::endl;
        status = false;
        return imuData;
    }

    // PARSING DATA
    pch = strtok ( buffer, "," );
    for ( int i=ACCELERATION; i <= MAGNETIC_FIELD; i++ )
    {
        for (int j=0; j < 3; j++)
        {
            if ( pch != NULL )
                imuData(j,i) = atof(pch);
            else
                imuData(j,i) = std::numeric_limits<double>::infinity();
            pch = strtok ( NULL, "," );
        }
    }

    // Adjusting to the box standards
    imuData.col(ACCELERATION) *= 100.0; // mg
    imuData.col(ANGULAR_SPEED) *= 180.0/M_PI; // degrees/s
    imuData.col(MAGNETIC_FIELD) *= 10.0; // mg

    return imuData;
}

int DeviceSPh::mAccept (int s, struct sockaddr *addr, unsigned int *addrlen, int timeout)
{
    int iResult;
    struct timeval tv;
    fd_set rfds;
    FD_ZERO(&rfds);
    FD_SET(s, &rfds);

    tv.tv_sec = (long)timeout;
    tv.tv_usec = 0;

    iResult = select(s, &rfds, (fd_set *) 0, (fd_set *) 0, &tv);
    if(iResult > 0)
    {
        return accept(s, addr, addrlen);
    }
    return 0;
}


DeviceSPh::~DeviceSPh () {}

void DeviceSPh::init ()
{
    // Init Winsock.
//    WSADATA wsaData;
//    int iResult = WSAStartup(MAKEWORD(2,2), &wsaData);

//    if (iResult != NO_ERROR)
		if (0)
    {
        std::cerr << "ERROR: DeviceSPh::init() failure" << std::endl;
//        system ( "pause" );
        exit ( EXIT_FAILURE );
    }
    else
        std::clog << "DeviceSPh::init() OK" << std::endl;
}

void DeviceSPh::closeAll ()
{
//    WSACleanup ();
}
