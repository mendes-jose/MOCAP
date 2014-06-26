#include "bodypart.h"

//BodyPart::BodyPart ( const unsigned char &iAdapter, const int &DeviceID,
//	const bool KFType, const float &KFW, const int &LPF,
//    const Vector3d &position, const Vector3d &Aori, const Vector3d &Cori,
//	const Vector3d &Lori, const Vector3d &Oori, const Vector4f &color,
//	const Vector3d dim):
//	DeviceBox(iAdapter, DeviceID), position(position), color(color),
//		KFT(KFType), W0(KFW), LPFSIZE(LPF), dim(dim)
//	{
//		this->LTOxyz=(Matrix<double,3,3>()<<
//			Lori,Oori.cross(Lori),Oori).finished();
//		this->ABCxyz=(Matrix<double,3,3>()<<
//			Aori,Cori.cross(Aori),Cori).finished();
//	}

BodyPart::BodyPart ( const int &DeviceID, const bool KFType, const float &KFW,
	const int &LPF, const Vector3d &position, const Vector3d &Aori,
	const Vector3d &Cori, const Vector3d &Lori, const Vector3d &Oori,
	const Vector4f &color, const Vector3d dim  ):
	DeviceSPh(DeviceID), position(position), color(color), KFT(KFType),
		W0(KFW), LPFSIZE(LPF), dim(dim)
	{
		this->LTOxyz = (Matrix<double,3,3>() <<
			Lori, Oori.cross(Lori), Oori).finished();
		this->ABCxyz = (Matrix<double,3,3>() <<
			Aori, Cori.cross(Aori), Cori).finished();
	}

void BodyPart::updateUKF ( void *param )
{
    BodyPart *thisBP = static_cast<BodyPart*>(param);

    Matrix3d iNEMO;
    Matrix3d iNEMOPREV;

    Quaternion<double> d ( thisBP->ABCxyz );
    Matrix<double,SDUKF,1> x; x.setZero();
    x.head<4>() << d.w(),d.x(),d.y(),d.z();

    MatrixXd xbuffer(SDUKF,1+thisBP->LPFSIZE); xbuffer << (MatrixXd::Zero(SDUKF,1+thisBP->LPFSIZE).colwise() + x);
    Matrix<double,4,4> Psi;
    Matrix<double,SDUKF,SDUKF> Q, Phi, P, sqrt_P;
    Q.setZero();
    thisBP->processNoiseCovarianceMatrixUKF(PTIMEINIT, x.head<4>(), Q);
    Q.block<ZD/2,ZD/2>(8,8).diagonal() = thisBP->va();
    Q.block<ZD/2,ZD/2>(8+ZD/2,8+ZD/2).diagonal() = thisBP->vm();
    Phi.setZero();
    P = Q;
    Matrix<double,SDUKF,ZD> L;
    Matrix<double,SDUKF,2*SDUKF+1> Sigma;
    Matrix<double,ZD,2*SDUKF+1> Z;
    Matrix<double,ZD,1> Z_bar;
    Matrix<double,SDUKF,1> Sigma_bar;

    Matrix<double,ZD,ZD> R; R.setZero();
    Matrix<double,ZD,ZD> CovZ;
    Matrix<double,SDUKF,ZD> CovXZ;

    const Matrix<double,3,3> LTOabc (thisBP->ABCxyz.inverse() * thisBP->LTOxyz);

    Vector3d m;
    double teta, m_norm;
    const double H_NORM = thisBP->h().norm();
    const double G_NORM = thisBP->g().norm();
    const double TETA_DIP = std::acos( ( thisBP->g().dot( thisBP->h() ) ) / (H_NORM * G_NORM) );
    double a_norm;

    const float W_ = ((1-thisBP->W0)/2.0/SDUKF);

    iNEMO = thisBP->readIMU();

    double t, dt;
    const double t0 = t = Common::getRealTime();
    unsigned int bufferIndex=1;

    while( thisBP->is_online() )
    {
        iNEMOPREV = iNEMO; //save previous
        
        /* *********************************************************************
         * iNEMO READING
         ************* */
        iNEMO = thisBP->readIMU();

        // Compute Elapsed Time
        dt = Common::getRealTime() - t; // seconds
        t = dt + t;

        /* *********************************************************************
         * SELECT SIGMA POINTS
         ******************* */
        sqrt_P = P.llt().matrixL();
        sqrt_P = Common::finvsqrt((1-thisBP->W0)/SDUKF) * sqrt_P;
        Sigma.col(0) = x;
        Sigma.block<SDUKF,SDUKF>(0,1) = sqrt_P.colwise() + x;
        Sigma.block<SDUKF,SDUKF>(0,1+SDUKF) = (-1*sqrt_P).colwise() + x;
        Sigma_bar = thisBP->W0* Sigma.col(0) + W_* Sigma.block<SDUKF,SDUKF*2>(0,1).rowwise().sum();

        /* *********************************************************************
         * PROJECT AHEAD (TIME UPDATE)
         *************************** */
        priorEstimateMatrix( dt, thisBP->gK() * ( iNEMO.col(ANGULAR_SPEED)/2.0 + iNEMOPREV.col(ANGULAR_SPEED)/2.0 - thisBP->gb() )*M_PI/180.0, Psi );    // d/s -> rad/s                   

        Psi  << Psi(0,0), -Psi(1,0), -Psi(2,0), -Psi(3,0),
            Psi(1,0),  Psi(0,0),  Psi(3,0), -Psi(2,0),
            Psi(2,0), -Psi(3,0),  Psi(0,0),  Psi(1,0),
            Psi(3,0),  Psi(2,0), -Psi(1,0),  Psi(0,0);

        Phi.block<4,4>(0,0) = Psi;
        Sigma = Phi*Sigma; //transformed sampling points
        Sigma_bar = Phi*Sigma_bar;

        thisBP->processNoiseCovarianceMatrixUKF(dt, x.head<4>(), Q);

        P = Phi*P*Phi.transpose() + Q;

        /* *********************************************************************
         * SENSORS VALIDATION
         ****************** */
        a_norm = (thisBP->aK() * (iNEMO.col(ACCELERATION) - thisBP->ab())).norm();
        if ( std::abs( a_norm - G_NORM ) < Common::EPSA)
        {
            //std::cout << "ACC\n";
            R.diagonal().head<3>() = thisBP->va(); 
        }
        else
            R.diagonal().head<3>().setConstant(Common::HIGHTCOV);

        m = thisBP->mK() * (iNEMO.col(MAGNETIC_FIELD) - thisBP->mb());
        m_norm = m.norm();
        teta = std::acos( (thisBP->g().dot(thisBP->ABCxyz*m)) / ( G_NORM * m_norm ) ); 
        if ( std::abs( H_NORM - m_norm ) < Common::EPSM  &&  std::abs( teta - TETA_DIP ) < Common::EPSDIP ) 
        {
            //std::cout << "MAG\n";
            R.diagonal().tail<3>() = thisBP->vm();
        }
        else
            R.diagonal().tail<3>().setConstant(Common::HIGHTCOV);

        /* **************************************************************************************
         * MEASUREMENT UNSCENTED TRANSFORMATION
         * ********************************** */
        //#pragma omp parallel for //unnecessary
        for (int i=0; i < 2*SDUKF+1; i++)
            Z.col(i) = thisBP->measurementFunction(Sigma.block<4,1>(0,i));

        Z_bar = thisBP->W0* Z.col(0) + W_* Z.block<ZD,SDUKF*2>(0,1).rowwise().sum();
        CovZ =   R + thisBP->W0*(Z.col(0)-Z_bar)*((Z.col(0)-Z_bar).transpose());
        CovZ +=  W_*(Z.block<ZD,2*SDUKF>(0,1).colwise() - Z_bar.col(0))*((Z.block<ZD,2*SDUKF>(0,1).colwise() - Z_bar).transpose());

        /* ********************************************************************************************
         * CROSS COVARIANCE MATRIX
         * ********************* */
        CovXZ =  thisBP->W0 * (Sigma.col(0) - Sigma_bar)*((Z.col(0)-Z_bar).transpose());
        CovXZ += W_ * (Sigma.block<SDUKF,2*SDUKF>(0,1).colwise() - Sigma_bar)*((Z.block<ZD,2*SDUKF>(0,1).colwise() - Z_bar).transpose());

        /* ********************************************************************************************
         * KALMAN GAIN COMPUTATION
         *********************** */
        L = CovXZ*CovZ.inverse();

        /* ********************************************************************************************
         * CORRECT ESTIMATE (MEASUREMENT UPDATE)
         ************************************* */
        x = Sigma_bar + L * ( (Vector6d() << iNEMO.col(ACCELERATION), iNEMO.col(MAGNETIC_FIELD)).finished() - Z_bar );
        P -= L*CovXZ.transpose();
        // Normalization step
        x.head<4>() = x.head<4>()*Common::finvsqrt(static_cast<float>(x(0)*x(0) + x(1)*x(1) + x(2)*x(2) + x(3)*x(3)));

        /* ********************************************************************************************
         * UPDATE BODY POSITION BASED ON THE QUATERNION
         ******************************************** */
        if ( thisBP->LPFSIZE > 0 )
        {
            xbuffer.col(0) = (x  + xbuffer.block(0,1,SDUKF,thisBP->LPFSIZE).rowwise().sum())/(thisBP->LPFSIZE+1);
            xbuffer.col(0) = xbuffer.col(0)*Common::finvsqrt(static_cast<float>(xbuffer(0,0)*xbuffer(0,0) + xbuffer(1,0)*xbuffer(1,0) + xbuffer(2,0)*xbuffer(2,0) + xbuffer(3,0)*xbuffer(3,0)));
        }
        else
        {
            xbuffer.col(0) = x;
        }

        // Quaternion to DCM
        thisBP->ABCxyz(0,0) = (xbuffer(0,0)*xbuffer(0,0) +
			xbuffer(1,0)*xbuffer(1,0) - xbuffer(2,0)*xbuffer(2,0) -
			xbuffer(3,0)*xbuffer(3,0));
        thisBP->ABCxyz(1,0) = (2*(xbuffer(1,0)*xbuffer(2,0) +
			xbuffer(0,0)*xbuffer(3,0)));
        thisBP->ABCxyz(2,0) = (2*(xbuffer(1,0)*xbuffer(3,0) -
			xbuffer(0,0)*xbuffer(2,0)));
        thisBP->ABCxyz(0,2) = 2*(xbuffer(1,0)*xbuffer(3,0) +
			xbuffer(0,0)*xbuffer(2,0));
        thisBP->ABCxyz(1,2) = 2*(xbuffer(2,0)*xbuffer(3,0) -
			xbuffer(0,0)*xbuffer(1,0));
        thisBP->ABCxyz(2,2) = xbuffer(0,0)*xbuffer(0,0) -
			xbuffer(1,0)*xbuffer(1,0) - xbuffer(2,0)*xbuffer(2,0) +
			xbuffer(3,0)*xbuffer(3,0);
        thisBP->ABCxyz.col(1) =
			thisBP->ABCxyz.col(2).cross(thisBP->ABCxyz.col(0));

        // Device referential to body part referential
        thisBP->LTOxyz = thisBP->ABCxyz * LTOabc;

        // Buffer update
        xbuffer.col(bufferIndex) = x;
        bufferIndex = bufferIndex%thisBP->LPFSIZE +1;
    }
}

void BodyPart::priorEstimateMatrix ( const double &dt, Vector3d w, Matrix<double,4,4> &Psi )
{
    Psi = (0.5*dt*(Matrix<double, 4,4>() <<
        0, -w(0), -w(1), -w(2),
        w(0),     0, -w(2),  w(1),
        w(1),  w(2),     0, -w(0),
        w(2), -w(1),  w(0),     0 ).finished()).exp();
}

void BodyPart::processNoiseCovarianceMatrixUKF ( const double &dt, const Vector4d &d, Matrix<double,SDUKF,SDUKF> &Q )
{
    Matrix<double,4,3> Xi (( Matrix<double,4,3>() <<
        -d(1), -d(2), -d(3), 
        +d(0), -d(3),  d(2),
        +d(3),  d(0), -d(1), 
        -d(2),  d(1),  d(0) ).finished());
    Q.block<4,4>(0,0) = Q.block<4,4>(4,4) = dt*dt/4*(Xi * this->vg().asDiagonal() * Xi.transpose());
}

Matrix<double,ZD,1> BodyPart::measurementFunction ( const Matrix<double,4,1> &x )
{
    return ( Matrix<double,ZD,1>()<<
        this->ab(0) - this->g(0)*(this->inv_aK(0,0)*(x(0)*x(0) + x(1)*x(1) - x(2)*x(2) - x(3)*x(3)) -this->inv_aK(0,1)*(2*x(0)*x(3) - 2*x(1)*x(2)) + this->inv_aK(0,2)*(2*x(0)*x(2) + 2*x(1)*x(3))) - 
        this->g(1)*(this->inv_aK(0,1)*(x(0)*x(0) - x(1)*x(1) + x(2)*x(2) - x(3)*x(3)) + this->inv_aK(0,0)*(2*x(0)*x(3) + 2*x(1)*x(2)) - this->inv_aK(0,2)*(2*x(0)*x(1) - 2*x(2)*x(3))) - 
        this->g(2)*(this->inv_aK(0,2)*(x(0)*x(0) - x(1)*x(1) - x(2)*x(2) + x(3)*x(3)) - this->inv_aK(0,0)*(2*x(0)*x(2) - 2*x(1)*x(3)) + this->inv_aK(0,1)*(2*x(0)*x(1) + 2*x(2)*x(3))),
        this->ab(1) - this->g(0)*(this->inv_aK(0,1)*(x(0)*x(0) + x(1)*x(1) - x(2)*x(2) - x(3)*x(3)) - this->inv_aK(1,1)*(2*x(0)*x(3) - 2*x(1)*x(2)) + this->inv_aK(1,2)*(2*x(0)*x(2) + 2*x(1)*x(3))) - 
        this->g(1)*(this->inv_aK(1,1)*(x(0)*x(0) - x(1)*x(1) + x(2)*x(2) - x(3)*x(3)) + this->inv_aK(0,1)*(2*x(0)*x(3) + 2*x(1)*x(2)) - this->inv_aK(1,2)*(2*x(0)*x(1) - 2*x(2)*x(3))) - 
        this->g(2)*(this->inv_aK(1,2)*(x(0)*x(0) - x(1)*x(1) - x(2)*x(2) + x(3)*x(3)) + this->inv_aK(1,1)*(2*x(0)*x(1) + 2*x(2)*x(3)) - this->inv_aK(0,1)*(2*x(0)*x(2) - 2*x(1)*x(3))),
        this->ab(2) - this->g(0)*(this->inv_aK(0,2)*(x(0)*x(0) + x(1)*x(1) - x(2)*x(2) - x(3)*x(3)) + this->inv_aK(2,2)*(2*x(0)*x(2) + 2*x(1)*x(3)) - this->inv_aK(1,2)*(2*x(0)*x(3) - 2*x(1)*x(2))) - 
        this->g(1)*(this->inv_aK(1,2)*(x(0)*x(0) - x(1)*x(1) + x(2)*x(2) - x(3)*x(3)) - this->inv_aK(2,2)*(2*x(0)*x(1) - 2*x(2)*x(3)) + this->inv_aK(0,2)*(2*x(0)*x(3) + 2*x(1)*x(2))) - 
        this->g(2)*(this->inv_aK(2,2)*(x(0)*x(0) - x(1)*x(1) - x(2)*x(2) + x(3)*x(3)) - this->inv_aK(0,2)*(2*x(0)*x(2) - 2*x(1)*x(3)) + this->inv_aK(1,2)*(2*x(0)*x(1) + 2*x(2)*x(3))),
        this->mb(0) + this->h(0)*(this->inv_mK(0,0)*(x(0)*x(0) + x(1)*x(1) - x(2)*x(2) - x(3)*x(3)) - this->inv_mK(0,1)*(2*x(0)*x(3) - 2*x(1)*x(2)) + this->inv_mK(0,2)*(2*x(0)*x(2) + 2*x(1)*x(3))) + 
        this->h(1)*(this->inv_mK(0,1)*(x(0)*x(0) - x(1)*x(1) + x(2)*x(2) - x(3)*x(3)) + this->inv_mK(0,0)*(2*x(0)*x(3) + 2*x(1)*x(2)) - this->inv_mK(0,2)*(2*x(0)*x(1) - 2*x(2)*x(3))) + 
        this->h(2)*(this->inv_mK(0,2)*(x(0)*x(0) - x(1)*x(1) - x(2)*x(2) + x(3)*x(3)) - this->inv_mK(0,0)*(2*x(0)*x(2) - 2*x(1)*x(3)) + this->inv_mK(0,1)*(2*x(0)*x(1) + 2*x(2)*x(3))),
        this->mb(1) + this->h(0)*(this->inv_mK(0,1)*(x(0)*x(0) + x(1)*x(1) - x(2)*x(2) - x(3)*x(3)) - this->inv_mK(1,1)*(2*x(0)*x(3) - 2*x(1)*x(2)) + this->inv_mK(1,2)*(2*x(0)*x(2) + 2*x(1)*x(3))) + 
        this->h(1)*(this->inv_mK(1,1)*(x(0)*x(0) - x(1)*x(1) + x(2)*x(2) - x(3)*x(3)) + this->inv_mK(0,1)*(2*x(0)*x(3) + 2*x(1)*x(2)) - this->inv_mK(1,2)*(2*x(0)*x(1) - 2*x(2)*x(3))) + 
        this->h(2)*(this->inv_mK(1,2)*(x(0)*x(0) - x(1)*x(1) - x(2)*x(2) + x(3)*x(3)) + this->inv_mK(1,1)*(2*x(0)*x(1) + 2*x(2)*x(3)) - this->inv_mK(0,1)*(2*x(0)*x(2) - 2*x(1)*x(3))),
        this->mb(2) + this->h(0)*(this->inv_mK(0,2)*(x(0)*x(0) + x(1)*x(1) - x(2)*x(2) - x(3)*x(3)) + this->inv_mK(2,2)*(2*x(0)*x(2) + 2*x(1)*x(3)) - this->inv_mK(1,2)*(2*x(0)*x(3) - 2*x(1)*x(2))) + 
        this->h(1)*(this->inv_mK(1,2)*(x(0)*x(0) - x(1)*x(1) + x(2)*x(2) - x(3)*x(3)) - this->inv_mK(2,2)*(2*x(0)*x(1) - 2*x(2)*x(3)) + this->inv_mK(0,2)*(2*x(0)*x(3) + 2*x(1)*x(2))) + 
        this->h(2)*(this->inv_mK(2,2)*(x(0)*x(0) - x(1)*x(1) - x(2)*x(2) + x(3)*x(3)) - this->inv_mK(0,2)*(2*x(0)*x(2) - 2*x(1)*x(3)) + this->inv_mK(1,2)*(2*x(0)*x(1) + 2*x(2)*x(3))) ).finished();
}

Matrix3d& BodyPart::readIMU ()
{
//    if ( this->deviceID > int(Common::LARGESTBOXID) )
        return DeviceSPh::readIMU();
//    else
//        return DeviceBox::readIMU();
}

BodyPart::~BodyPart () {}
