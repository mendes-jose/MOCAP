#include "upperarm.h"

//Upperarm::Upperarm ( const unsigned char &iAdapter, const int &DeviceID,
//    const bool KFType, const float &KFW,
//    const int &LPF,
//    BodyPart *thorax,
//    const Vector3d &Aori, const Vector3d &Cori, const Vector3d &Lori, const Vector3d &Oori,
//    const Vector4f &color,
//    const Vector3d &rSize, const float &HHeight, const Vector3d &ecc, const char side):
//BodyPart ( iAdapter, DeviceID, KFType, KFW, LPF, thorax->pos() + thorax->getDim(2)*thorax->O() + side*SHOULDERF*thorax->getDim(2)*thorax->T(),
//    Aori, Cori, Lori, Oori, color, (Vector3d()<<rSize(0)*HHeight, rSize(1)*HHeight, -1*rSize(2)*HHeight).finished() ),
//    thorax(thorax), ecc(ecc), side(side) {}


Upperarm::Upperarm ( const int &DeviceID,
    const bool KFType, const float &KFW,
    const int &LPF,
    BodyPart *thorax,
    const Vector3d &Aori, const Vector3d &Cori, const Vector3d &Lori, const Vector3d &Oori,
    const Vector4f &color,
    const Vector3d &rSize, const float &HHeight, const Vector3d &ecc, const char side):
BodyPart ( DeviceID, KFType, KFW, LPF, thorax->pos() + thorax->getDim(2)*thorax->O() + side*SHOULDERF*thorax->getDim(2)*thorax->T(),
    Aori, Cori, Lori, Oori, color, (Vector3d()<<rSize(0)*HHeight, rSize(1)*HHeight, -1*rSize(2)*HHeight).finished() ),
    thorax(thorax), ecc(ecc), side(side) {}


void Upperarm::drawMySelf(const double &lightX, const double &lightY, const double &lightZ)
{
    this->light << lightX, lightY, lightZ;
    this->light *= Common::finvsqrt(static_cast<float>(this->light.cwiseProduct(this->light).sum())); 
    
    this->positionUpdate();

    Common::glPyramid ( this->position, this->L(), this->T(), this->O(), this->light, this->dim, this->ecc, this->color );
}

