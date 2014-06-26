/*!
 * Author:  José Magno MENDES FILHO
 * Site:    http://www.ensta-paristech.fr/~mendesfilho/public/
 * License: Creative Commons Attribution 3.0 Unported License
 *          http://creativecommons.org/licenses/by/3.0/
 */
#ifndef BODYPART_H
#define BODYPART_H

#include <fstream>
#include <string>
#include <iostream>
#include <gl.h>
#include <glu.h>
#include <glut.h>
#include <math.h>
#include <omp.h>
#include "devicesph.h"

#define SDUKF 14
#define ZD 6
#define SDEKF 4
#define PTIMEINIT .01

using namespace Eigen;

/*!
 * \brief
 * Specifies a base class for the representation of any body part composing a human body model.
 * 
 * \see
 * Head | Thoraxabdo | Upperarm | Lowerarm | Hand | Upperleg | Lowerleg | Foot
 */
class BodyPart: public DeviceBox, public DeviceSPh
{
private:
    /*!
     * \brief
     * Updates the matrix \f$A\f$ which is the quaternion multiplication matrix based on
     * the angular speed and the elapsed time between two IMU readings.
     * 
     * \param dt
     * Elapsed time between two IMU readings.
     * 
     * \param w
     * Angular Velocity of the IMU Device on its on "ABC" referential.
     * 
     * \param A
     * Quaternion multiplication matrix representing the rotation undergone by the IMU 
     * device with respect to its own previous coordinate system "ABC"
     *
     * \remarks
     * The matrix \f$A\f$ is calculated as showed below:
     * \f[
     *    A\left(T,\ \mathbf{\omega}\right)=\exp\left(\frac{1}{2}\Psi\left(\mathbf{\Omega}\right)T\right)
     * \f]
     * where \f$T\f$ is the elapsed time, \f$\Omega\f$ represents the quaternion composed by
     * the angular velocity vector:
     * \f[
     *        \Omega=\left[\begin{array}{c}
     *            0\\
     *            \omega_{a}\\
     *            \omega_{b}\\
     *            \omega_{c}
     *            \end{array}\right]
     * \f]
     * and the \f$\Psi\f$ function is as defined below:
     * \f[
     *        \Psi(\mathbf{q})=[\mathbf{q},\ \Xi(\mathbf{q})]
     * \f]
     * \f[
     *        \Xi(\mathbf{q})=\begin{bmatrix}-\vec{\mathbf{e}}^{T}\\
     *                     q_{1}\mathbf{I}+[\vec{\mathbf{e}}]\times
     *                    \end{bmatrix}
     * \f]
     * \f[
     *        [\vec{\mathbf{e}}]\times=\begin{bmatrix}0 & -q_{4} & q_{3}\\
     *            q_{4} & 0 & -q_{2}\\
     *            -q_{3} & q_{2} & 0
     *            \end{bmatrix}
     * \f]
     *
     * \see
     * To understand better about this Quaternion orientation and its relation with angular speed read
     * http://www.ensta-paristech.fr/~mendesfilho/public/MOCAP/Report.pdf on the
     * "Determination of the Body Segment Orientation" section.
     */
    inline static void priorEstimateMatrix ( const double &dt, Vector3d w, Matrix<double,4,4> &A );
    
    /*!
     * \brief
     * Calculates the time depending part of the Process Noise Covariance Matrix according
     * to the Unscented Kalman Filtering model
     * 
     * \param dt
     * Elapsed time between two IMU readings.
     * 
     * \param d
     * Quaternion orientation of the device.
     *
     * \param Q
     * Process Noise Covariance Matrix
     * 
     * \remarks
     * The UKF Process Noise Covariance Matrix \f$Q\f$ is defined as following:
     * \f[
     *        Q=\left[\begin{array}{cccc}
     *        (T/2)^{2}\Xi(\mathbf{d})\Sigma_{g}\Xi(\mathbf{d})^{T} &  & 0 & 0\\
     *        0 & (T/2)^{2}\Xi(\mathbf{d})\Sigma_{g}\Xi(\mathbf{d})^{T} &  & 0\\
     *        &  & \Sigma_{a}\\
     *        0 &  & 0 & \Sigma_{m}
     *        \end{array}\right]
     * \f]
     * where \f$\Xi\f$ is defined above, for the BodyPart::priorEstimateMatrix() and
     * \f$\Sigma_{g}\f$, \f$\Sigma_{a}\f$ and \f$\Sigma_{m}\f$ are the covariance matrices
     * of the gyroscope, accelerometer and magnetometer noises, respectively.
     * Only the \f$8\times 8\f$ top left block matrix is calculated on this function since
     * it's the only time depending part.
     *
     * \see
     * To understand better about this UKF Process Noise Covariance Matrix read
     * http://www.ensta-paristech.fr/~mendesfilho/public/MOCAP/Report.pdf on the
     * "Determination of the Body Segment Orientation" section.
     */
    inline void processNoiseCovarianceMatrixUKF ( const double &dt, const Vector4d &d, Matrix<double,SDUKF,SDUKF> &Q );

    /*!
     * \brief
     * Calculates the Process Noise Covariance Matrix according to the Extended Kalman Filtering model
     * 
     * \param dt
     * Elapsed time between two IMU readings.
     * 
     * \param d
     * Quaternion orientation of the device (for this EKF model it is the same as the 
     * EKF state vector).
     * 
     * \returns
     * Returns the EKF Process Noise Covariance Matrix \f$Q\f$ defined as following:
     * \f[
     *        Q=(T/2)^{2}\Xi(\mathbf{d})\Sigma_{g}\Xi(\mathbf{d})^{T}
     * \f]
     * where \f$\Xi\f$ is defined above, for the BodyPart::priorEstimateMatrix() and \f$\Sigma_{g}\f$
     * is the covariance matrix of the gyroscope noise.
     *
     * \see
     * To understand better about this EKF Process Noise Covariance Matrix read
     * http://www.ensta-paristech.fr/~mendesfilho/public/MOCAP/Report.pdf on the
     * "Determination of the Body Segment Orientation" section.
     */
    inline Matrix<double,SDEKF,SDEKF> processNoiseCovarianceMatrixEKF ( const double &dt, const Vector4d &d );

    /*!
     * \brief
     * Calculates the measurement estimative \f$\hat{z}\f$ base on the current IMU device orientation,
     * calibration values, local gravity and local geomagnetic field.
     * 
     * \param x
     * Quaternion orientation of the IMU device.
     * 
     * \returns
     * Returns the estimated measurement vector \f$\hat{z}\f$.
     * 
     * \remarks
     * The actual measurement vector \f$z\f$ is just the output from the accelerometer
     * stacked over the the one from the magnetometer to form a vector on \f$\mathbb{R}^{6}\f$ space.
     * 
     * \see
     * To understand how the measurement estimation is done see
     * http://www.ensta-paristech.fr/~mendesfilho/public/MOCAP/Report.pdf on the
     * "Determination of the Body Segment Orientation" section.
     */
    inline Matrix<double,ZD,1> measurementFunction ( const Matrix<double,4,1> &x );

    /*!
     * \brief
     * Evaluates the Jacobian of the measurement function with respect to the EKF state vector
     * on the current EKF state vector.
     * 
     * \param x
     * Quaternion orientation of the device (for this EKF model it is the same as the 
     * EKF state vector).
     * 
     * \returns
     * Returns the evaluated Jacobian matrix \f$C\f$.
     * 
     * \see
     * To understand the measurement function and its Jacobian matrix computation read
     * http://www.ensta-paristech.fr/~mendesfilho/public/MOCAP/Report.pdf on the
     * "Determination of the Body Segment Orientation" section.
     */
    inline Matrix<double,ZD,SDEKF> jacobian_h (const Vector4d &x);
    

    /*!
     * \brief
     * Provides a pure virtual method for the implementation of body parts position
     * update.
     * 
     * Normally, if a body part (a) has a dependence on another body part (b)
     * this will be where to calculate the position changes on (a) caused by
     * position and orientation changes of (b).
     * 
     * \remarks
     * This function should be called as frequent as possible in order to have
     * good and smooth movements on the openGL visualization. That's why this
     * function wasn't called inside BodyPart::updateEKF() nor inside BodyPart::updateUKF().
     * This two functions stay in a loop which can take 20ms or so to do one interaction.
     * Therefore BodyPart::postionUpdate() overrides where called inside their
     * respectives BodyPart::drawMySelf() overrides.
     * 
     * \see
     * Foot::postionUpdate() | Hand::positionUpdate() | Head::positionUpdate() |
     * Lowerarm::positionUpdate() | Lowerleg::positionUpdate() | Thoraxabdo::positionUpdate() |
     * Upperarm::positionUpdate() | Upperleg::positionUpdate()
     */
    inline virtual void positionUpdate () = 0;

    const bool KFT; /**< Kalman Filter type (UKF or EKF) */
    const float W0; /**< UKF Sigma points weighting */
    const int LPFSIZE; /**< Low Pass Filter constant */

    Matrix3d ABCxyz; /**< Initial position of a device referential  ABC relative to the XYZ fixed referential */
    Matrix3d LTOxyz; /**< Initial position of a body part referential LTO relative to the XYZ fixed referential */

protected:
    Vector3d position; /**< 3D vector representing the body part position with respect to the XYZ coordinate system*/
    Vector4f color; /**< RGBA color of the body part */
    const Vector3d dim; /**< The dimensions of this body part represented by three values */

public:
    /*!
     * \brief
     * Specialized constructor which attempts to associate this body part representation class with a Box type sensoring device.
     * 
     * \param iAdapter
     * Specifies the Adapter number for the connection with a Box type sensoring device.
     *
     * \param DeviceID
     * Specifies the Device identification number to be associated to this body part.
     *
     * \param KFType
     * Specifies which variant of the Kalman Filter will be used, UKF or EKF.
     * 
     * \param KFW
     * Specifies a UKF weight value used for the sigma point calculation
     * 
     * \param LPF
     * Low Pass Filter gain.
     * 
     * \param position
     * Specifies this body part position on the \f$\mathbb{R}^{3}\f$ space.
     *
     * \param Aori
     * Specifies the initial "A" unitary vector from the "ABC" device referential with respect to the "XYZ" fixed referential.
     * 
     * \param Cori
     * Specifies the initial "C" unitary vector from the "ABC" device referential with respect to the "XYZ" fixed referential.
     * 
     * \param Lori
     * Specifies the initial "L" unitary vector from the "LTO" body part referential with respect to the "XYZ" fixed referential.
     * 
     * \param Oori
     * Specifies the initial "O" unitary vector from the "LTO" body part referential with respect to the "XYZ" fixed referential.
     * 
     * \param color
     * Specifies the body part color (used only if its device is online).
     * 
     * \param dim
     * Specifies the body part dimensions.
     * 
     */
    BodyPart ( const unsigned char &iAdapter, const int &DeviceID, const bool KFType,
        const float &KFW, const int &LPF, const Vector3d &position, const Vector3d &Aori,
        const Vector3d &Cori, const Vector3d &Lori, const Vector3d &Oori,
        const Vector4f &color, const Vector3d dim  );

    /*!
     * \brief
     * Specialized constructor which attempts to associate this body part representation class with a Android based sensoring device.
     *
     * \param DeviceID
     * Specifies the Device identification number to be associated to this body part.
     *
     * \param KFType
     * Specifies which variant of the Kalman Filter will be used, UKF or EKF.
     * 
     * \param KFW
     * Specifies a UKF weight value used for the sigma point calculation
     * 
     * \param LPF
     * Low Pass Filter gain.
     * 
     * \param position
     * Specifies this body part position on the \f$\mathbb{R}^{3}\f$ space.
     *
     * \param Aori
     * Specifies the initial "A" unitary vector from the "ABC" device referential with respect to the "XYZ" fixed referential.
     * 
     * \param Cori
     * Specifies the initial "C" unitary vector from the "ABC" device referential with respect to the "XYZ" fixed referential.
     * 
     * \param Lori
     * Specifies the initial "L" unitary vector from the "LTO" body part referential with respect to the "XYZ" fixed referential.
     * 
     * \param Oori
     * Specifies the initial "O" unitary vector from the "LTO" body part referential with respect to the "XYZ" fixed referential.
     * 
     * \param color
     * Specifies the body part color (used only if its device is online).
     * 
     * \param dim
     * Specifies the body part dimensions.
     * 
     */
    BodyPart ( const int &DeviceID, const bool KFType, const float &KFW, const int &LPF,
        const Vector3d &position, const Vector3d &Aori, const Vector3d &Cori,
        const Vector3d &Lori, const Vector3d &Oori, const Vector4f &color,
        const Vector3d dim  );
    /*!
     * \brief
     * Pure virtual destructor in order to make this class abstract.
     */
    virtual ~BodyPart () = 0;
    
    /*!
     * \brief
     * Getter for the body part position.
     */
    inline const Vector3d& pos () const { return this->position; }
    
    /*!
     * \brief
     * Setter/getter for the initial position of the body part referential LTO relative to the XYZ fixed referential
     */
    inline Vector3d L () const { return this->LTOxyz.col(0); }
    inline Vector3d T () const { return this->LTOxyz.col(1); }
    inline Vector3d O () const { return this->LTOxyz.col(2); }
    inline Vector4f &RGBAcolor () { return this->color; }
    inline bool KFType () const { return this->KFT; }
    virtual inline const double getDim ( const int &i ) const {return this->dim(i);}
    inline const Matrix3d& ABC2XYZ () const { return ABCxyz; }

    static void updateUKF ( void *param );
    static void updateEKF ( void *param );

    Matrix3d& readIMU ();
    
    virtual void drawMySelf(const double &lightX, const double &lightY, const double &lightZ) = 0;
    
};

#endif // BODYPART_H
