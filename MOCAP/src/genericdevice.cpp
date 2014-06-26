#include "genericdevice.h"

const unsigned char ACCELERATION = 0;
const unsigned char ANGULAR_SPEED = 1;
const unsigned char MAGNETIC_FIELD = 2;

GenericDevice::GenericDevice () {}

void GenericDevice::writeCalib ( const Eigen::Matrix<double,36,1> &calib )
{
    this->acc_bias = calib.block<3,1>(0,0);
    this->_aK = (Eigen::Matrix<double,3,3>() << calib(3,0), calib(6,0), calib(7,0),
        calib(6,0), calib(4,0), calib(8,0),
        calib(7,0), calib(8,0), calib(5,0)).finished();
    this->acc_var = calib.block<3,1>(9,0);

    this->gyro_bias = calib.block<3,1>(12,0);
    this->_gK = (Eigen::Matrix<double,3,3>() << calib(15,0), calib(18,0), calib(19,0),
        calib(18,0), calib(16,0), calib(20,0),
        calib(19,0), calib(20,0), calib(17,0)).finished();
    this->gyro_var = calib.block<3,1>(21,0);

    this->mag_bias = calib.block<3,1>(24,0);
    this->_mK = (Eigen::Matrix<double,3,3>() << calib(27,0), calib(30,0), calib(31,0),
        calib(30,0), calib(28,0), calib(32,0),
        calib(31,0), calib(32,0), calib(29,0)).finished();
    this->mag_var = calib.block<3,1>(33,0);

    this->_inv_gK = this->_gK.inverse();
    this->_inv_aK = this->_aK.inverse();
    this->_inv_mK = this->_mK.inverse();
}

GenericDevice::~GenericDevice () {}