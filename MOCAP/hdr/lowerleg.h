/*!
 * Author:  José Magno MENDES FILHO
 * Site:    http://www.ensta-paristech.fr/~mendesfilho/public/
 * License: Creative Commons Attribution 3.0 Unported License
 *          http://creativecommons.org/licenses/by/3.0/
 */
#ifndef LOWERLEG_H
#define LOWERLEG_H

#ifdef _WIN32
#include <windows.h>
#endif

#include "bodypart.h"

/*!
 * \brief
 * Specifies the representation of a lower leg of the virtual human body model.
 * 
 * \remarks
 * This class is used to represent both left and right lower legs.
 * 
 * \see
 * BodyPart.
 */
class Lowerleg: public BodyPart
{
private:
    Vector3d light; /**< Specifies the light direction */
    Vector3d ecc; /**< Specifies the three dimension eccentricity of this body part */
    BodyPart *uleg; /**< Specifies a pointer to the body part dependence of this body part */

    /*!
     * \brief
     * See BodyPart::positionUpdate().
     */
    inline virtual void positionUpdate () { this->position = this->uleg->pos()-abs(this->uleg->getDim(2))*this->uleg->O(); }

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
     * \param uleg
     * Pointer to the body part dependence of this body part.
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
     * \param rSize
     * Specifies the multiplying factors for this body part dimensions with respect to the parameter "Height". 
     * The absolute value of the dimension vector will be equal to rSize*Height.
     *
     * \param HHeight
     * Specifies the virtual human body height.
     * 
     * \param ecc
     * Specifies the eccentricity of the body part in each direction L, T, O.
     */
    Lowerleg ( const unsigned char &iAdapter, const int &DeviceID,
        const bool KFType, const float &KFW,
        const int &LPF,
        BodyPart *uleg,
        const Vector3d &Aori, const Vector3d &Cori, const Vector3d &Lori, const Vector3d &Oori,
        const Vector4f &color,
        const Vector3d &rSize, const float &HHeight, const Vector3d &ecc);

    /*!
     * \brief
     * Specialized constructor which attempts to associate this body part representation class with a Box type sensoring device.
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
     * \param uleg
     * Pointer to the body part dependence of this body part.
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
     * \param rSize
     * Specifies the multiplying factors for this body part dimensions with respect to the parameter "Height". 
     * The absolute value of the dimension vector will be equal to rSize*Height.
     *
     * \param HHeight
     * Specifies the virtual human body height.
     * 
     * \param ecc
     * Specifies the eccentricity of the body part in each direction L, T, O.
     */
    Lowerleg ( const int &DeviceID,
        const bool KFType, const float &KFW,
        const int &LPF,
        BodyPart *uleg,
        const Vector3d &Aori, const Vector3d &Cori, const Vector3d &Lori, const Vector3d &Oori,
        const Vector4f &color,
        const Vector3d &rSize, const float &HHeight, const Vector3d &ecc);

    /*!
     * \brief
     * See BodyPart::drawMySelf().
     */
    virtual void drawMySelf(const double &lightX, const double &lightY, const double &lightZ);
    
};

#endif // LOWERLEG_H