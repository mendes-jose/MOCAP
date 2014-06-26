#include "bodypart.h"

void BodyPart::updateEKF ( void *param )
{
    BodyPart *thisBP = static_cast<BodyPart*>(param);

    Matrix3d iNEMO;
    Matrix3d iNEMOPREV;

    Quaternion<double> d ( thisBP->ABCxyz );
    Matrix<double,SDEKF,1> x; x.setZero();
    x.head<4>() << d.w(),d.x(),d.y(),d.z();

    MatrixXd xbuffer(SDEKF,1+thisBP->LPFSIZE); xbuffer << (MatrixXd::Zero(SDEKF,1+thisBP->LPFSIZE).colwise() + x);
    Matrix<double,4,4> Psi;
    Matrix<double,SDEKF,SDEKF> Phi, P;
    Phi.setZero();
    P = thisBP->processNoiseCovarianceMatrixEKF(PTIMEINIT, x.head<4>());
    Matrix<double,ZD,SDEKF> C;
    Matrix<double,SDEKF,ZD> L;
    Matrix<double,ZD,ZD> R; R.setZero();

    const Matrix<double,3,3> LTOabc (thisBP->ABCxyz.inverse() * thisBP->LTOxyz);

    Vector3d m;
    double teta, m_norm;
    const double H_NORM = thisBP->h().norm();
    const double G_NORM = thisBP->g().norm();
    const double TETA_DIP = std::acos( ( thisBP->g().dot( thisBP->h() ) ) / (H_NORM * G_NORM) );
    double a_norm;

    iNEMO = thisBP->readIMU();

    double t, dt;
    const double t0 = t = Common::getRealTime();
    unsigned int bufferIndex = 1;

    while( thisBP->is_online() )
    {
        iNEMOPREV = iNEMO; //save previous iNEMO data

        /* ******************************************************************
         * iNEMO READING
         ************* */
        iNEMO = thisBP->readIMU();

        // Compute Elapsed Time
        dt = Common::getRealTime() - t; // seconds
        t = dt + t;

        /* ******************************************************************
         * PROJECT AHEAD (TIME UPDATE)
         *************************** */
        priorEstimateMatrix( dt, thisBP->gK() * ( iNEMO.col(ANGULAR_SPEED)/2.0 +
            iNEMOPREV.col(ANGULAR_SPEED)/2.0 - thisBP->gb() )*M_PI/180.0, Psi );    // d/s -> rad/s                   

        Psi  << Psi(0,0), -Psi(1,0), -Psi(2,0), -Psi(3,0),
            Psi(1,0),  Psi(0,0),  Psi(3,0), -Psi(2,0),
            Psi(2,0), -Psi(3,0),  Psi(0,0),  Psi(1,0),
            Psi(3,0),  Psi(2,0), -Psi(1,0),  Psi(0,0);

        x.head<4>() = Psi*x.head<4>();  // (1) project the state ahead in time
        Phi = Psi; // for this case (x = [d]) Phi = Psi but not in general ( as in x = [d|...]' )
        P = Phi*P*Phi.transpose() + thisBP->processNoiseCovarianceMatrixEKF(dt, x.head<4>()); // (2) project the error covariance ahead in time
        
        /* ******************************************************************
        * SENSORS VALIDATION
        ****************** */
        a_norm = (thisBP->aK() * (iNEMO.col(ACCELERATION) - thisBP->ab())).norm();
        if ( std::abs( a_norm - G_NORM ) < Common::EPSA)
        {
            R.diagonal().head<3>() = thisBP->va(); 
        }
        else
            R.diagonal().head<3>().setConstant(Common::HIGHTCOV);

        m = thisBP->mK() * (iNEMO.col(MAGNETIC_FIELD) - thisBP->mb());
        m_norm = m.norm();
        teta = std::acos( (thisBP->g().dot(thisBP->ABCxyz*m)) / ( G_NORM * m_norm ) ); 
        if ( std::abs( H_NORM - m_norm ) < Common::EPSM  &&  std::abs( teta - TETA_DIP ) < Common::EPSDIP ) 
        {
            R.diagonal().tail<3>() = thisBP->vm();
        }
        else
            R.diagonal().tail<3>().setConstant(Common::HIGHTCOV);

        /* ******************************************************************
         * JACOBIAN COMPUTATION
         ******************* */

        C = thisBP->jacobian_h(x);

        /* ******************************************************************
         * KALMAN GAIN COMPUTATION
         *********************** */
        L = P*C.transpose() * (C*P*C.transpose() + R).inverse();

        /* ******************************************************************
         * CORRECT ESTIMATE (MEASUREMENT UPDATE)
         ************************************* */
        x += L * ( (Vector6d() << iNEMO.col(ACCELERATION), iNEMO.col(MAGNETIC_FIELD)).finished() - thisBP->measurementFunction(x) );
        P = (Matrix<double,SDEKF,SDEKF>::Identity() - L*C)*P;

        // Normalization step
        x *= Common::finvsqrt(static_cast<float>(x(0)*x(0) + x(1)*x(1) + x(2)*x(2) + x(3)*x(3)));

        /* ******************************************************************
         * UPDATE BODY ORIENTATION BASED ON THE QUATERNION
         ********************************************** */
        if ( thisBP->LPFSIZE > 0 )
        {
            xbuffer.col(0) = (x  + xbuffer.block( 0, 1, SDEKF, thisBP->LPFSIZE ).rowwise( ).sum()) / (thisBP->LPFSIZE+1);
            xbuffer.col(0) = xbuffer.col(0)*Common::finvsqrt( static_cast<float>(xbuffer(0, 0) * xbuffer(0, 0) + xbuffer(1, 0) * xbuffer(1, 0) + xbuffer(2, 0) * xbuffer(2, 0) + xbuffer(3, 0) * xbuffer(3, 0)));
        }
        else
        {
            xbuffer.col(0) = x;
        }

        // Quaternion to DCM
        thisBP->ABCxyz(0,0) = (xbuffer(0,0)*xbuffer(0,0) + xbuffer(1,0)*xbuffer(1,0) - xbuffer(2,0)*xbuffer(2,0) - xbuffer(3,0)*xbuffer(3,0));
        thisBP->ABCxyz(1,0) = (2*(xbuffer(1,0)*xbuffer(2,0) + xbuffer(0,0)*xbuffer(3,0)));
        thisBP->ABCxyz(2,0) = (2*(xbuffer(1,0)*xbuffer(3,0) - xbuffer(0,0)*xbuffer(2,0)));
        thisBP->ABCxyz(0,2) = 2*(xbuffer(1,0)*xbuffer(3,0) + xbuffer(0,0)*xbuffer(2,0));
        thisBP->ABCxyz(1,2) = 2*(xbuffer(2,0)*xbuffer(3,0) - xbuffer(0,0)*xbuffer(1,0));
        thisBP->ABCxyz(2,2) = xbuffer(0,0)*xbuffer(0,0) - xbuffer(1,0)*xbuffer(1,0) - xbuffer(2,0)*xbuffer(2,0) + xbuffer(3,0)*xbuffer(3,0);
        thisBP->ABCxyz.col(1) = thisBP->ABCxyz.col(2).cross(thisBP->ABCxyz.col(0));

        // Device referential to body part referential
        thisBP->LTOxyz = thisBP->ABCxyz * LTOabc;

        // Buffer update
        xbuffer.col(bufferIndex) = x;
        bufferIndex = bufferIndex%thisBP->LPFSIZE +1;

    }
}


Matrix<double,SDEKF,SDEKF> BodyPart::processNoiseCovarianceMatrixEKF ( const double &dt, const Vector4d &d )
{
    Matrix<double,4,3> Xi (( Matrix<double,4,3>()<<
        -d(1), -d(2), -d(3), 
        +d(0), -d(3),  d(2),
        +d(3),  d(0), -d(1), 
        -d(2),  d(1),  d(0) ).finished());

    return ( Matrix<double,SDEKF,SDEKF>() << dt*dt/4*(Xi * this->vg().asDiagonal() * Xi.transpose()) ).finished();
}

Matrix<double,ZD,SDEKF> BodyPart::jacobian_h (const Vector4d &x)
{
    return 0.5*(Matrix<double,ZD,SDEKF>() <<
        -this->g(0)*(2*this->inv_aK(0,0)*x(0) - 2*this->inv_aK(0,1)*x(3) + 2*this->inv_aK(0,2)*x(2)) - 
        this->g(1)*(2*this->inv_aK(0,0)*x(3) + 2*this->inv_aK(0,1)*x(0) - 2*this->inv_aK(0,2)*x(1)) - 
        this->g(2)*(2*this->inv_aK(0,1)*x(1) - 2*this->inv_aK(0,0)*x(2) + 2*this->inv_aK(0,2)*x(0)), 
        this->g(1)*(2*this->inv_aK(0,1)*x(1) - 2*this->inv_aK(0,0)*x(2) + 2*this->inv_aK(0,2)*x(0)) - 
        this->g(0)*(2*this->inv_aK(0,0)*x(1) + 2*this->inv_aK(0,1)*x(2) + 2*this->inv_aK(0,2)*x(3)) - 
        this->g(2)*(2*this->inv_aK(0,0)*x(3) + 2*this->inv_aK(0,1)*x(0) - 2*this->inv_aK(0,2)*x(1)), 
        this->g(2)*(2*this->inv_aK(0,0)*x(0) - 2*this->inv_aK(0,1)*x(3) + 2*this->inv_aK(0,2)*x(2)) - 
        this->g(1)*(2*this->inv_aK(0,0)*x(1) + 2*this->inv_aK(0,1)*x(2) + 2*this->inv_aK(0,2)*x(3)) - 
        this->g(0)*(2*this->inv_aK(0,1)*x(1) - 2*this->inv_aK(0,0)*x(2) + 2*this->inv_aK(0,2)*x(0)), 
        this->g(0)*(2*this->inv_aK(0,0)*x(3) + 2*this->inv_aK(0,1)*x(0) - 2*this->inv_aK(0,2)*x(1)) - 
        this->g(1)*(2*this->inv_aK(0,0)*x(0) - 2*this->inv_aK(0,1)*x(3) + 2*this->inv_aK(0,2)*x(2)) - 
        this->g(2)*(2*this->inv_aK(0,0)*x(1) + 2*this->inv_aK(0,1)*x(2) + 2*this->inv_aK(0,2)*x(3)),
        -this->g(0)*(2*this->inv_aK(0,1)*x(0) - 2*this->inv_aK(1,1)*x(3) + 2*this->inv_aK(1,2)*x(2)) - 
        this->g(1)*(2*this->inv_aK(1,1)*x(0) + 2*this->inv_aK(0,1)*x(3) - 2*this->inv_aK(1,2)*x(1)) - 
        this->g(2)*(2*this->inv_aK(1,1)*x(1) - 2*this->inv_aK(0,1)*x(2) + 2*this->inv_aK(1,2)*x(0)), 
        this->g(1)*(2*this->inv_aK(1,1)*x(1) - 2*this->inv_aK(0,1)*x(2) + 2*this->inv_aK(1,2)*x(0)) - 
        this->g(0)*(2*this->inv_aK(1,1)*x(2) + 2*this->inv_aK(0,1)*x(1) + 2*this->inv_aK(1,2)*x(3)) - 
        this->g(2)*(2*this->inv_aK(1,1)*x(0) + 2*this->inv_aK(0,1)*x(3) - 2*this->inv_aK(1,2)*x(1)), 
        this->g(2)*(2*this->inv_aK(0,1)*x(0) - 2*this->inv_aK(1,1)*x(3) + 2*this->inv_aK(1,2)*x(2)) - 
        this->g(1)*(2*this->inv_aK(1,1)*x(2) + 2*this->inv_aK(0,1)*x(1) + 2*this->inv_aK(1,2)*x(3)) - 
        this->g(0)*(2*this->inv_aK(1,1)*x(1) - 2*this->inv_aK(0,1)*x(2) + 2*this->inv_aK(1,2)*x(0)), 
        this->g(0)*(2*this->inv_aK(1,1)*x(0) + 2*this->inv_aK(0,1)*x(3) - 2*this->inv_aK(1,2)*x(1)) - 
        this->g(1)*(2*this->inv_aK(0,1)*x(0) - 2*this->inv_aK(1,1)*x(3) + 2*this->inv_aK(1,2)*x(2)) - 
        this->g(2)*(2*this->inv_aK(1,1)*x(2) + 2*this->inv_aK(0,1)*x(1) + 2*this->inv_aK(1,2)*x(3)),
        -this->g(0)*(2*this->inv_aK(2,2)*x(2) + 2*this->inv_aK(0,2)*x(0) - 2*this->inv_aK(1,2)*x(3)) - 
        this->g(1)*(2*this->inv_aK(1,2)*x(0) - 2*this->inv_aK(2,2)*x(1) + 2*this->inv_aK(0,2)*x(3)) - 
        this->g(2)*(2*this->inv_aK(2,2)*x(0) - 2*this->inv_aK(0,2)*x(2) + 2*this->inv_aK(1,2)*x(1)), 
        this->g(1)*(2*this->inv_aK(2,2)*x(0) - 2*this->inv_aK(0,2)*x(2) + 2*this->inv_aK(1,2)*x(1)) - 
        this->g(0)*(2*this->inv_aK(2,2)*x(3) + 2*this->inv_aK(0,2)*x(1) + 2*this->inv_aK(1,2)*x(2)) - 
        this->g(2)*(2*this->inv_aK(1,2)*x(0) - 2*this->inv_aK(2,2)*x(1) + 2*this->inv_aK(0,2)*x(3)), 
        this->g(2)*(2*this->inv_aK(2,2)*x(2) + 2*this->inv_aK(0,2)*x(0) - 2*this->inv_aK(1,2)*x(3)) - 
        this->g(1)*(2*this->inv_aK(2,2)*x(3) + 2*this->inv_aK(0,2)*x(1) + 2*this->inv_aK(1,2)*x(2)) - 
        this->g(0)*(2*this->inv_aK(2,2)*x(0) - 2*this->inv_aK(0,2)*x(2) + 2*this->inv_aK(1,2)*x(1)), 
        this->g(0)*(2*this->inv_aK(1,2)*x(0) - 2*this->inv_aK(2,2)*x(1) + 2*this->inv_aK(0,2)*x(3)) - 
        this->g(1)*(2*this->inv_aK(2,2)*x(2) + 2*this->inv_aK(0,2)*x(0) - 2*this->inv_aK(1,2)*x(3)) - 
        this->g(2)*(2*this->inv_aK(2,2)*x(3) + 2*this->inv_aK(0,2)*x(1) + 2*this->inv_aK(1,2)*x(2)),
        this->h(0)*(2*this->inv_mK(0,0)*x(0) - 2*this->inv_mK(0,1)*x(3) + 2*this->inv_mK(0,2)*x(2)) + 
        this->h(1)*(2*this->inv_mK(0,0)*x(3) + 2*this->inv_mK(0,1)*x(0) - 2*this->inv_mK(0,2)*x(1)) + 
        this->h(2)*(2*this->inv_mK(0,1)*x(1) - 2*this->inv_mK(0,0)*x(2) + 2*this->inv_mK(0,2)*x(0)), 
        this->h(0)*(2*this->inv_mK(0,0)*x(1) + 2*this->inv_mK(0,1)*x(2) + 2*this->inv_mK(0,2)*x(3)) - 
        this->h(1)*(2*this->inv_mK(0,1)*x(1) - 2*this->inv_mK(0,0)*x(2) + 2*this->inv_mK(0,2)*x(0)) + 
        this->h(2)*(2*this->inv_mK(0,0)*x(3) + 2*this->inv_mK(0,1)*x(0) - 2*this->inv_mK(0,2)*x(1)), 
        this->h(0)*(2*this->inv_mK(0,1)*x(1) - 2*this->inv_mK(0,0)*x(2) + 2*this->inv_mK(0,2)*x(0)) + 
        this->h(1)*(2*this->inv_mK(0,0)*x(1) + 2*this->inv_mK(0,1)*x(2) + 2*this->inv_mK(0,2)*x(3)) - 
        this->h(2)*(2*this->inv_mK(0,0)*x(0) - 2*this->inv_mK(0,1)*x(3) + 2*this->inv_mK(0,2)*x(2)), 
        this->h(1)*(2*this->inv_mK(0,0)*x(0) - 2*this->inv_mK(0,1)*x(3) + 2*this->inv_mK(0,2)*x(2)) - 
        this->h(0)*(2*this->inv_mK(0,0)*x(3) + 2*this->inv_mK(0,1)*x(0) - 2*this->inv_mK(0,2)*x(1)) + 
        this->h(2)*(2*this->inv_mK(0,0)*x(1) + 2*this->inv_mK(0,1)*x(2) + 2*this->inv_mK(0,2)*x(3)),
        this->h(0)*(2*this->inv_mK(0,1)*x(0) - 2*this->inv_mK(1,1)*x(3) + 2*this->inv_mK(1,2)*x(2)) + 
        this->h(1)*(2*this->inv_mK(1,1)*x(0) + 2*this->inv_mK(0,1)*x(3) - 2*this->inv_mK(1,2)*x(1)) + 
        this->h(2)*(2*this->inv_mK(1,1)*x(1) - 2*this->inv_mK(0,1)*x(2) + 2*this->inv_mK(1,2)*x(0)), 
        this->h(0)*(2*this->inv_mK(1,1)*x(2) + 2*this->inv_mK(0,1)*x(1) + 2*this->inv_mK(1,2)*x(3)) - 
        this->h(1)*(2*this->inv_mK(1,1)*x(1) - 2*this->inv_mK(0,1)*x(2) + 2*this->inv_mK(1,2)*x(0)) + 
        this->h(2)*(2*this->inv_mK(1,1)*x(0) + 2*this->inv_mK(0,1)*x(3) - 2*this->inv_mK(1,2)*x(1)), 
        this->h(0)*(2*this->inv_mK(1,1)*x(1) - 2*this->inv_mK(0,1)*x(2) + 2*this->inv_mK(1,2)*x(0)) + 
        this->h(1)*(2*this->inv_mK(1,1)*x(2) + 2*this->inv_mK(0,1)*x(1) + 2*this->inv_mK(1,2)*x(3)) - 
        this->h(2)*(2*this->inv_mK(0,1)*x(0) - 2*this->inv_mK(1,1)*x(3) + 2*this->inv_mK(1,2)*x(2)), 
        this->h(1)*(2*this->inv_mK(0,1)*x(0) - 2*this->inv_mK(1,1)*x(3) + 2*this->inv_mK(1,2)*x(2)) - 
        this->h(0)*(2*this->inv_mK(1,1)*x(0) + 2*this->inv_mK(0,1)*x(3) - 2*this->inv_mK(1,2)*x(1)) + 
        this->h(2)*(2*this->inv_mK(1,1)*x(2) + 2*this->inv_mK(0,1)*x(1) + 2*this->inv_mK(1,2)*x(3)),
        this->h(0)*(2*this->inv_mK(2,2)*x(2) + 2*this->inv_mK(0,2)*x(0) - 2*this->inv_mK(1,2)*x(3)) + 
        this->h(1)*(2*this->inv_mK(1,2)*x(0) - 2*this->inv_mK(2,2)*x(1) + 2*this->inv_mK(0,2)*x(3)) + 
        this->h(2)*(2*this->inv_mK(2,2)*x(0) - 2*this->inv_mK(0,2)*x(2) + 2*this->inv_mK(1,2)*x(1)), 
        this->h(0)*(2*this->inv_mK(2,2)*x(3) + 2*this->inv_mK(0,2)*x(1) + 2*this->inv_mK(1,2)*x(2)) - 
        this->h(1)*(2*this->inv_mK(2,2)*x(0) - 2*this->inv_mK(0,2)*x(2) + 2*this->inv_mK(1,2)*x(1)) + 
        this->h(2)*(2*this->inv_mK(1,2)*x(0) - 2*this->inv_mK(2,2)*x(1) + 2*this->inv_mK(0,2)*x(3)), 
        this->h(0)*(2*this->inv_mK(2,2)*x(0) - 2*this->inv_mK(0,2)*x(2) + 2*this->inv_mK(1,2)*x(1)) + 
        this->h(1)*(2*this->inv_mK(2,2)*x(3) + 2*this->inv_mK(0,2)*x(1) + 2*this->inv_mK(1,2)*x(2)) - 
        this->h(2)*(2*this->inv_mK(2,2)*x(2) + 2*this->inv_mK(0,2)*x(0) - 2*this->inv_mK(1,2)*x(3)), 
        this->h(1)*(2*this->inv_mK(2,2)*x(2) + 2*this->inv_mK(0,2)*x(0) - 2*this->inv_mK(1,2)*x(3)) - 
        this->h(0)*(2*this->inv_mK(1,2)*x(0) - 2*this->inv_mK(2,2)*x(1) + 2*this->inv_mK(0,2)*x(3)) + 
        this->h(2)*(2*this->inv_mK(2,2)*x(3) + 2*this->inv_mK(0,2)*x(1) + 2*this->inv_mK(1,2)*x(2)) ).finished();
}