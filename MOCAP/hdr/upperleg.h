/*!
 * Author:  José Magno MENDES FILHO
 * Site:    http://www.ensta-paristech.fr/~mendesfilho/public/
 * License: Creative Commons Attribution 3.0 Unported License
 *          http://creativecommons.org/licenses/by/3.0/
 */
#ifndef UPPERLEG_H
#define UPPERLEG_H


#ifdef _WIN32
#include <windows.h>
#endif

#include "bodypart.h"

/*!
 * \brief
 * Specifies the representation of an upper leg of the virtual human body model.
 * 
 * \remarks
 * This class is used to represent both left and right upper legs.
 * 
 * \see
 * BodyPart.
 */
class Upperleg: public BodyPart
{
private:
    Vector3d light; /**< Specifies the light direction */
    Vector3d ecc; /**< Specifies the three dimension eccentricity of this body part */
    BodyPart *thorax; /**< Specifies a pointer to the body part dependence of this body part */
    char side; /**< Specifies on which side of the body part dependence this body part is located */

    /*!
     * \brief
     * See BodyPart::positionUpdate().
     */
    inline virtual void positionUpdate () { this->position = this->thorax->pos() + this->side*PELVISF*this->thorax->getDim(2)*this->thorax->T(); }

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
     * \param thorax
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
     * 
     * \param side
     * Specifies on which side of the body part dependence this body part will be linked.
     * 
     */
    Upperleg ( const unsigned char &iAdapter, const int &DeviceID,
        const bool KFType, const float &KFW,
        const int &LPF,
        BodyPart *thorax,
        const Vector3d &Aori, const Vector3d &Cori, const Vector3d &Lori, const Vector3d &Oori,
        const Vector4f &color,
        const Vector3d &rSize, const float &HHeight, const Vector3d &ecc, const char side);

    /*!
     * \brief
     * Specialized constructor which attempts to associate this body part representation class with a Android based sensoring device.
     * 
     * \param DeviceID
     * Device identification number.
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
     * \param thorax
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
     * 
     * \param side
     * Specifies on which side of the body part dependence this body part will be linked.
     * 
     */
    Upperleg ( const int &DeviceID,
        const bool KFType, const float &KFW,
        const int &LPF,
        BodyPart *thorax,
        const Vector3d &Aori, const Vector3d &Cori, const Vector3d &Lori, const Vector3d &Oori,
        const Vector4f &color,
        const Vector3d &rSize, const float &HHeight, const Vector3d &ecc, const char side);

    /*!
     * \brief
     * See BodyPart::drawMySelf().
     */
    virtual void drawMySelf(const double &lightX, const double &lightY, const double &lightZ);
    
};

#endif // UPPERLEG_H
