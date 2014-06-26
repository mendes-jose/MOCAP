/*!
 * Author:  José Magno MENDES FILHO
 * Site:    http://www.ensta-paristech.fr/~mendesfilho/public/
 * License: Creative Commons Attribution 3.0 Unported License
 *          http://creativecommons.org/licenses/by/3.0/
 */
#ifndef DEVICE_H
#define DEVICE_H

//using namespace System; // it gives a symbol conflict, so better not use since it is used just for the Thread::Sleep(...) function
//using namespace System::Threading;

#include "common.h"

/*
 * Constants for IMU data type
 */
extern const unsigned char ACCELERATION; /**< Index for the acceleration IMU type of data */
extern const unsigned char ANGULAR_SPEED; /**< Index for the angular speed IMU type of data */
extern const unsigned char MAGNETIC_FIELD; /**< Index for the magnetic field IMU type of data */

/*!
 * \brief
 * Specifies a generic IMU sensoring device having a 3-axis accelerometer, 3-axis gyroscope and 3-axis magnetometer.
 * 
 * This abstract class implementing basic properties and functionalities of a IMU sensoring device as described before.
 * It has some data fields for sensors calibration values, sensors variance values, environment constants as well as some 
 * setters and getters for this fields. Besides that it defines also a pure virtual method for reading IMU data.
 * 
 */
class GenericDevice
{

protected:
    bool status; /**< TODO */
    int deviceID; /**< TODO */
    int ret; /**< TODO */

    Eigen::Vector3d gyro_bias; /**< TODO */
    Eigen::Vector3d acc_bias; /**< TODO */
    Eigen::Vector3d mag_bias; /**< TODO */

    Eigen::Matrix3d _gK; /**< TODO */
    Eigen::Matrix3d _aK; /**< TODO */
    Eigen::Matrix3d _mK; /**< TODO */
    Eigen::Matrix3d _inv_gK; /**< TODO */
    Eigen::Matrix3d _inv_aK; /**< TODO */
    Eigen::Matrix3d _inv_mK; /**< TODO */

    Eigen::Vector3d gyro_var; /**< 3-axis gyroscope variance */
    Eigen::Vector3d acc_var; /**< 3-axis accelerometer variance */
    Eigen::Vector3d mag_var; /**< 3-axis magnetometer variance */

    Eigen::Vector3d grav;  /**< G force */
    Eigen::Vector3d gmf; /**< Geomagnetic field */

    Eigen::Matrix3d imuData; /**< Stores IMU data */

public:
    /* 
     * Macro for overload the operator new for fixed-size eigen structures avoiding the misaligning asserting problem
     * see: http://eigen.tuxfamily.org/dox/TopicStructHavingEigenMembers.html
     */
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    GenericDevice ();
    virtual ~GenericDevice () = 0; // pure virtual destructor => abstract class
    virtual Eigen::Matrix3d& readIMU () = 0;
    
    void writeCalib ( const Eigen::Matrix<double,36,1>& calib );

    inline bool is_online () const { return this->status; }
    inline unsigned char devID () const { return this->deviceID; }

    inline const Eigen::Vector3d& gb () const { return gyro_bias; }
    inline const Eigen::Vector3d& ab () const { return acc_bias; }
    inline const Eigen::Vector3d& mb () const { return mag_bias; }
    inline const double& gb (int i) const { return gyro_bias(i); }
    inline const double& ab (int i) const { return acc_bias(i); }
    inline const double& mb (int i) const { return mag_bias(i); }

    inline const Eigen::Matrix3d& gK () const { return _gK; }
    inline const Eigen::Matrix3d& aK () const { return _aK; }
    inline const Eigen::Matrix3d& mK () const { return _mK; }
    inline const double& gK ( int i, int j ) const { return _gK(i,j); }
    inline const double& aK ( int i, int j ) const { return _aK(i,j); }
    inline const double& mK ( int i, int j ) const { return _mK(i,j); }
    inline const Eigen::Matrix3d& inv_gK () const { return _inv_gK; }
    inline const Eigen::Matrix3d& inv_aK () const { return _inv_aK; }
    inline const Eigen::Matrix3d& inv_mK () const { return _inv_mK; }
    inline const double& inv_gK ( int i, int j ) const { return _inv_gK(i,j); }
    inline const double& inv_aK ( int i, int j ) const { return _inv_aK(i,j); }
    inline const double& inv_mK ( int i, int j ) const { return _inv_mK(i,j); }

    inline Eigen::Vector3d& g () { return grav; }
    inline Eigen::Vector3d& h () { return gmf; }
    inline const double& g (int i) const { return grav(i); }
    inline const double& h (int i) const { return gmf(i); }

    inline const Eigen::Vector3d& vg () const { return gyro_var; }
    inline const Eigen::Vector3d& va () const { return acc_var; }
    inline const Eigen::Vector3d& vm () const { return mag_var; }
};

#endif // DEVICE_H
