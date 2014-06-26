#include "devicebox.h"

const unsigned short SEND_PORT = 2000;
const unsigned short LISTEN_PORT = 2000;
const unsigned char IP_ADDR_BASE[4] = {192, 168, 2, 128};

DeviceBox::DeviceBox (): GenericDevice() {}

DeviceBox::DeviceBox ( const unsigned char &iAdapter, const int &boxID ):
    GenericDevice(), iAdapter(iAdapter)
{
    this->deviceID = boxID;
    
    if (this->deviceID > -1)
    {

        const unsigned IPAddr = (unsigned) ( (IP_ADDR_BASE[0] << 24) | (IP_ADDR_BASE[1] << 16) | (IP_ADDR_BASE[2] << 8) | IP_ADDR_BASE[3]+boxID );

        if ( (ret = TrCAN_Dev_OpenTCP(iAdapter, IPAddr, SEND_PORT, LISTEN_PORT)) == 1 )
            clog << "TrCAN Dev OpenTCP (GenericDevice "<< boxID << "): "<< ret << endl;
        else
        {
            std::cerr << "WARNING: could not open TCP/IP link with device " << boxID << std::endl;
            status = false;
            return;
        }
    
        if ( (ret = TrCAN_Dev_CommInit(iAdapter)) == 1 )
            clog << "TrCAN Dev CommInit (GenericDevice "<< boxID << "): " << ret << std::endl;
        else
        {
            std::cerr << "WARNING: fail to start communication with device " << boxID << std::endl;
            status = false;
            return;
        }

        status = true; // Communication well succeed
    }
    else status = false;
}

Matrix3d& DeviceBox::readIMU ( const unsigned char &sensorType )
{
    // SETTING DATAPACK
    this->dataPack.iRetVal = 0; 
    this->dataPack.uiId = 0x200;
    this->dataPack.chLength = 1;
    this->dataPack.chData[0] = sensorType; dataPack.chData[1] = 1; dataPack.chData[2] = 2; dataPack.chData[3] = 3;
    this->dataPack.chData[4] = 4; dataPack.chData[5] = 5; dataPack.chData[6] = 6; dataPack.chData[7] = 7;

    // SENDING DATA REQUEST
    if ( (ret = TrCAN_Dev_SendData(this->iAdapter, &this->dataPack)) < 1 ) // quit? resend? do nothing? what to do?
    {
        std::cerr << "WARNING: fail to send data to device" << deviceID << std::endl;
        status = false;
        return this->imuData;
    }

    // RECIEVING DATA
    System::Threading::Thread::Sleep ( Common::RECVBOXDATAWAITINGTIME ); // wait a while before trying to get the requested data
    ret = TrCAN_Dev_RecvData ( this->iAdapter, &this->dataPack );
    for ( int counter=0; ret < 1 && counter*Common::RECVBOXDATAWAITINGTIME < Common::RECVBOXDATATIMEOUT; counter++ )
    {
        System::Threading::Thread::Sleep ( Common::RECVBOXDATAWAITINGTIME ); // wait a while before trying to get the requested data
        ret = TrCAN_Dev_RecvData ( this->iAdapter, &this->dataPack );
    }

    if ( this->dataPack.chData[0] == 255 )
    {
        std::cerr << "WARNING: fail to receive IMU data from device " << deviceID << std::endl;
        status = false;
        return this->imuData;
    }

    // PARSING DATA
    this->imuData.col(sensorType)<<
        double(short( (this->dataPack.chData[1] << 8) | this->dataPack.chData[2] )),
        double(short( (this->dataPack.chData[3] << 8) | this->dataPack.chData[4] )),
        double(short( (this->dataPack.chData[5] << 8) | this->dataPack.chData[6] ));

    return this->imuData;
}

Matrix3d& DeviceBox::readIMU()
{
    this->readIMU(ACCELERATION);
    this->readIMU(MAGNETIC_FIELD);
    this->readIMU(ANGULAR_SPEED); imuData(1,ANGULAR_SPEED) *= -1; // correcting the inversed gyro y-axis
    return this->imuData;
}

DeviceBox::~DeviceBox () {}

void DeviceBox::init ()
{
    int ret;
    //int vMajor,
    //    vMinor;

    if ( (ret = TrCAN_Init()) == 1 )
        std::clog << "DeviceBox::init() OK" << std::endl;
    else
    {
        std::cerr << "ERROR: DeviceBox::init() failure" << std::endl;
        system ( "pause" );
        exit ( EXIT_FAILURE );
    }
}

void DeviceBox::closeAll ()
{
    TrCAN_Dev_CloseAll ();
}