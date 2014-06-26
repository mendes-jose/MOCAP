#include "hand.h"

//Hand::Hand ( const unsigned char &iAdapter, const int &DeviceID,
//    const bool KFType, const float &KFW,
//    const int &LPF,
//    BodyPart *larm,
//    const Vector3d &Aori, const Vector3d &Cori, const Vector3d &Lori, const Vector3d &Oori,
//    const Vector4f &color,
//    const Vector3d &rSize, const float &HHeight, const Vector3d &ecc):
//BodyPart ( iAdapter, DeviceID, KFType, KFW, LPF, larm->pos()-abs(larm->getDim(2))*larm->O(),
//    Aori, Cori, Lori, Oori, color, (Vector3d()<<rSize(0)*HHeight, rSize(1)*HHeight, -1*rSize(2)*HHeight).finished() ),
//    larm(larm), ecc(ecc) {}

Hand::Hand ( const int &DeviceID,
    const bool KFType, const float &KFW,
    const int &LPF,
    BodyPart *larm,
    const Vector3d &Aori, const Vector3d &Cori, const Vector3d &Lori, const Vector3d &Oori,
    const Vector4f &color,
    const Vector3d &rSize, const float &HHeight, const Vector3d &ecc):
BodyPart ( DeviceID, KFType, KFW, LPF, larm->pos()-abs(larm->getDim(2))*larm->O(),
    Aori, Cori, Lori, Oori, color, (Vector3d()<<rSize(0)*HHeight, rSize(1)*HHeight, -1*rSize(2)*HHeight).finished() ),
    larm(larm), ecc(ecc) {}

void Hand::drawMySelf(const double &lightX, const double &lightY, const double &lightZ)
{
    this->light << lightX, lightY, lightZ;
    this->light *= Common::finvsqrt(static_cast<float>(this->light.cwiseProduct(this->light).sum())); 
    
    this->positionUpdate();

    Common::glPyramid ( this->position, this->L(), this->T(), this->O(), this->light, this->dim, this->ecc, this->color );
}
