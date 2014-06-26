#ifndef COMMON_H
#define COMMON_H

// BODY PRORORTIONS
#define NECKF 0.2
#define SHOULDERF .4
#define PELVISF .25

#define LEFT 1
#define RIGHT -1
#define UKF true
#define EKF false


#if defined(_WIN32)
#include <Windows.h>
#include "Eigen\Dense"
#include "unsupported\Eigen\MatrixFunctions"

#elif defined(__unix__) || defined(__unix) || defined(unix) || (defined(__APPLE__) && defined(__MACH__))
#include <unistd.h>    /* POSIX flags */
#include <time.h>    /* clock_gettime(), time() */
#include <sys/time.h>    /* gethrtime(), gettimeofday() */
#include "Eigen/Dense"
#include "unsupported/Eigen/MatrixFunctions"

#if defined(__MACH__) && defined(__APPLE__)
#include <mach/mach.h>
#include <mach/mach_time.h>
#endif

#else
#error "Unable to define Common::getRealTime( ) for an unknown OS."
#endif

using namespace std;
#include <gl.h>
#include <glu.h>
#include <glut.h>
#include <iostream>
#include <math.h>

typedef Eigen::Matrix<double, 10, 1 > Vector10d;
typedef Eigen::Matrix<double, 6, 1 > Vector6d;
typedef Eigen::Matrix<double, 9, 1 > Vector9d;

namespace Common
{

    extern unsigned int RECVBOXDATAWAITINGTIME;
    extern unsigned int RECVBOXDATATIMEOUT;
    extern unsigned int RECVBOXDATATIMEOUT;
    extern unsigned int LARGESTBOXID;
    extern unsigned int RECVSMARTPHONEDATAWAITINGTIME;
    extern unsigned int RECVSMARTPHONEDATATIMEOUT;
    extern unsigned int TCPSERVERBASEPORT;
    extern unsigned int TCPSERVERLISTENINGTIMEOUT;
    extern bool SHOWLTOREF;
    extern float EPSA;
    extern float EPSM;
    extern float EPSDIP;
    extern float HIGHTCOV;

    /*!
     * \brief
     * Returns the Fast Inverse Square Root.
     */
    float finvsqrt( float number );

    /*!
     * \brief
     * Returns the real time, in seconds, or -1.0 if an error occurred.
     *
     * Time is measured since an arbitrary and OS-dependent start time.
     * The returned real time is only useful for computing an elapsed time
     * between two calls to this function.
     */
    double getRealTime( );

    /*!
     * \brief
     * Draws a "pyramidish" geometrical object used to represent whole or partial body parts.
     * 
     * \param p
     * Position unitary vector.
     * 
     * \param l
     * Unitary vector representing the longitudinal orientation of the pyramidish object.
     * 
     * \param t
     * Unitary vector representing the transversal orientation of the pyramidish object.
     * 
     * \param o
     * Unitary vector representing the orthogonal orientation of the pyramidish object.
     * 
     * \param light
     * Unitary vector representing the light direction.
     * 
     * \param dim
     * Specifies the tree dimensions of the pyramidish object.
     * 
     * \param eccentricity
     * Specifies the eccentricity on each of the three "lto" orientations.
     * 
     * \param cor
     * Specifies the pyramidish object color.
     * 
     */
    void glPyramid ( const Eigen::Vector3d &p, const Eigen::Vector3d &l, const Eigen::Vector3d &t, const Eigen::Vector3d &o,
        const Eigen::Vector3d &light, const Eigen::Vector3d &dim, const Eigen::Vector3d &eccentricity,
        const Eigen::Vector4f &cor);

}
#endif // COMMON_H
