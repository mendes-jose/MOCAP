#include "lowerarm.h"

//Lowerarm::Lowerarm ( const unsigned char &iAdapter, const int &DeviceID,
//    const bool KFType, const float &KFW,
//    const int &LPF,
//    BodyPart *uarm,
//    const Vector3d &Aori, const Vector3d &Cori, const Vector3d &Lori, const Vector3d &Oori,
//    const Vector4f &color,
//    const Vector3d &rSize, const float &HHeight, const Vector3d &ecc):
//BodyPart ( iAdapter, DeviceID, KFType, KFW, LPF, uarm->pos()-abs(uarm->getDim(2))*uarm->O(),
//    Aori, Cori, Lori, Oori, color, (Vector3d()<<rSize(0)*HHeight, rSize(1)*HHeight, -1*rSize(2)*HHeight).finished() ),
//    uarm(uarm), ecc(ecc) {}

Lowerarm::Lowerarm ( const int &DeviceID,
    const bool KFType, const float &KFW,
    const int &LPF,
    BodyPart *uarm,
    const Vector3d &Aori, const Vector3d &Cori, const Vector3d &Lori, const Vector3d &Oori,
    const Vector4f &color,
    const Vector3d &rSize, const float &HHeight, const Vector3d &ecc):
BodyPart ( DeviceID, KFType, KFW, LPF, uarm->pos()-abs(uarm->getDim(2))*uarm->O(),
    Aori, Cori, Lori, Oori, color, (Vector3d()<<rSize(0)*HHeight, rSize(1)*HHeight, -1*rSize(2)*HHeight).finished() ),
    uarm(uarm), ecc(ecc) {}

void Lowerarm::drawMySelf(const double &lightX, const double &lightY, const double &lightZ)
{
    this->light << lightX, lightY, lightZ;
    this->light *= Common::finvsqrt(static_cast<float>(this->light.cwiseProduct(this->light).sum())); 
    
    this->positionUpdate(); 

    Common::glPyramid ( this->position, this->L(), this->T(), this->O(), this->light, this->dim, this->ecc, this->color );
}
