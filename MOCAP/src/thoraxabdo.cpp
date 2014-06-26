#include "thoraxabdo.h"

Thoraxabdo::Thoraxabdo ( const unsigned char &iAdapter, const int &DeviceID,
    const bool KFType, const float &KFW,
    const int &LPF,
    const Vector3d &refPos,
    const Vector3d &Aori, const Vector3d &Cori, const Vector3d &Lori, const Vector3d &Oori,
    const Vector4f &color,
    const Vector3d &rSize, const float &HHeight, const Vector3d &ecc):
BodyPart ( iAdapter, DeviceID, KFType, KFW, LPF, refPos, Aori, Cori, Lori, Oori, color, rSize*HHeight),
    ecc(ecc) {}

Thoraxabdo::Thoraxabdo ( const int &DeviceID,
    const bool KFType, const float &KFW,
    const int &LPF,
    const Vector3d &refPos,
    const Vector3d &Aori, const Vector3d &Cori, const Vector3d &Lori, const Vector3d &Oori,
    const Vector4f &color,
    const Vector3d &rSize, const float &HHeight, const Vector3d &ecc):
BodyPart ( DeviceID, KFType, KFW, LPF, refPos, Aori, Cori, Lori, Oori, color, rSize*HHeight ),
    ecc(ecc) {}

void Thoraxabdo::drawMySelf(const double &lightX, const double &lightY, const double &lightZ)
{
    this->light << lightX, lightY, lightZ;
    this->light *= Common::finvsqrt(static_cast<float>(this->light.cwiseProduct(this->light).sum())); 
    
    this->positionUpdate();

    Common::glPyramid ( this->position, this->L(), this->T(), this->O(), this->light, this->dim, this->ecc, this->color );
    Common::glPyramid ( this->position, this->L(), this->O(), -1*this->T(), this->light, PELVISF*this->dim, this->ecc, this->color );
    Common::glPyramid ( this->position, this->L(), -1*this->O(), this->T(), this->light, PELVISF*this->dim, this->ecc, this->color );
    Common::glPyramid ( this->position +dim(2)*this->O(), this->L(), this->O(), -1*this->T(), this->light, SHOULDERF*this->dim, this->ecc, this->color );
    Common::glPyramid ( this->position +dim(2)*this->O(), this->L(), -1*this->O(), this->T(), this->light, SHOULDERF*this->dim, this->ecc, this->color );
}