#include "head.h"

Head::Head ( const unsigned char &iAdapter, const int &DeviceID,
    const bool KFType, const float &KFW,
    const int &LPF,
    BodyPart *thorax,
    const Vector3d &Aori, const Vector3d &Cori, const Vector3d &Lori, const Vector3d &Oori,
    const Vector4f &color,
    const Vector3d &rSize, const float &HHeight, const Vector3d &ecc):
BodyPart ( iAdapter, DeviceID, KFType, KFW, LPF, thorax->pos() + thorax->getDim(2)*thorax->O(),
    Aori, Cori, Lori, Oori, color, rSize*HHeight ),
    thorax(thorax), ecc(ecc) {}

Head::Head ( const int &DeviceID,
    const bool KFType, const float &KFW,
    const int &LPF,
    BodyPart *thorax,
    const Vector3d &Aori, const Vector3d &Cori, const Vector3d &Lori, const Vector3d &Oori,
    const Vector4f &color,
    const Vector3d &rSize, const float &HHeight, const Vector3d &ecc):
BodyPart ( DeviceID, KFType, KFW, LPF, thorax->pos() + thorax->getDim(2)*thorax->O(),
    Aori, Cori, Lori, Oori, color, rSize*HHeight ),
    thorax(thorax), ecc(ecc) {}

void Head::drawMySelf(const double &lightX, const double &lightY, const double &lightZ)
{
    this->light << lightX, lightY, lightZ;
    this->light *= Common::finvsqrt(static_cast<float>(this->light.cwiseProduct(this->light).sum()));
    
    this->positionUpdate();

    Common::glPyramid ( this->position + NECKF*this->getDim(2)*this->O(), this->L(), this->T(), this->O(), this->light, this->dim, this->ecc, this->color );
    Common::glPyramid ( this->position, this->L(), this->T(), this->O(), this->light, NECKF*this->dim, (Vector3d() << .5,.5,.5).finished(), this->color );
}