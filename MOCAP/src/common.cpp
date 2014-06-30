#include "common.h"

unsigned int Common::RECVBOXDATAWAITINGTIME;
unsigned int Common::RECVBOXDATATIMEOUT;
unsigned int Common::LARGESTBOXID;
unsigned int Common::RECVSMARTPHONEDATAWAITINGTIME;
unsigned int Common::RECVSMARTPHONEDATATIMEOUT;
unsigned int Common::TCPSERVERBASEPORT;
unsigned int Common::TCPSERVERLISTENINGTIMEOUT;
bool Common::SHOWLTOREF;
float Common::EPSA;
float Common::EPSM;
float Common::EPSDIP;
float Common::HIGHTCOV;
//unsigned int GH_NSAMPLES;
//Eigen::Vector3f G;
//Eigen::Vector3f H;
//bool ESTIMATEGRAVITYATRUNTIME;
//bool ESTIMATEMAGFIELDATRUNTIME;

float Common::finvsqrt( float number )
{
        long i;
        float x2, y;
        const float threehalfs = 1.5F;
 
        x2 = number * 0.5F;
        y  = number;
        i  = * ( long * ) &y;                       // evil floating point bit level hacking
        i  = 0x5f375a86 - ( i >> 1 );               // Chris Lomont constant
        y  = * ( float * ) &i;
        y  = y * ( threehalfs - ( x2 * y * y ) );   // 1st iteration
//      y  = y * ( threehalfs - ( x2 * y * y ) );   // 2nd iteration, this can be removed
 
        return y;
}

/*
 * Author:  David Robert Nadeau
 * Modified by: José Magno Mendes Filho
 * Site:    http://NadeauSoftware.com/
 * License: Creative Commons Attribution 3.0 Unported License
 *          http://creativecommons.org/licenses/by/3.0/deed.en_US
 */
double Common::getRealTime()
{
#if defined(_WIN32)
    FILETIME tm;
    ULONGLONG t;
#if defined(NTDDI_WIN8) && NTDDI_VERSION >= NTDDI_WIN8
    /* Windows 8, Windows Server 2012 and later. ---------------- */
    GetSystemTimePreciseAsFileTime( &tm );
#else
    /* Windows 2000 and later. ---------------------------------- */
    GetSystemTimeAsFileTime( &tm );
#endif
    t = ((ULONGLONG)tm.dwHighDateTime << 32) | (ULONGLONG)tm.dwLowDateTime;
    return (double)t / 10000000.0;

#elif (defined(__hpux) || defined(hpux)) || ((defined(__sun__) || defined(__sun) || defined(sun)) && (defined(__SVR4) || defined(__svr4__)))
    /* HP-UX, Solaris. ------------------------------------------ */
    return (double)gethrtime( ) / 1000000000.0;

#elif defined(__MACH__) && defined(__APPLE__)
    /* OSX. ----------------------------------------------------- */
    static double timeConvert = 0.0;
    if ( timeConvert == 0.0 )
    {
        mach_timebase_info_data_t timeBase;
        (void)mach_timebase_info( &timeBase );
        timeConvert = (double)timeBase.numer /
            (double)timeBase.denom /
            1000000000.0;
    }
    return (double)mach_absolute_time( ) * timeConvert;

#elif defined(_POSIX_VERSION)
    /* POSIX. --------------------------------------------------- */
#if defined(_POSIX_TIMERS) && (_POSIX_TIMERS > 0)
    {
        struct timespec ts;
#if defined(CLOCK_MONOTONIC_PRECISE)
        /* BSD. --------------------------------------------- */
        const clockid_t id = CLOCK_MONOTONIC_PRECISE;
#elif defined(CLOCK_MONOTONIC_RAW)
        /* Linux. ------------------------------------------- */
        const clockid_t id = CLOCK_MONOTONIC_RAW;
#elif defined(CLOCK_HIGHRES)
        /* Solaris. ----------------------------------------- */
        const clockid_t id = CLOCK_HIGHRES;
#elif defined(CLOCK_MONOTONIC)
        /* AIX, BSD, Linux, POSIX, Solaris. ----------------- */
        const clockid_t id = CLOCK_MONOTONIC;
#elif defined(CLOCK_REALTIME)
        /* AIX, BSD, HP-UX, Linux, POSIX. ------------------- */
        const clockid_t id = CLOCK_REALTIME;
#else
        const clockid_t id = (clockid_t)-1;    /* Unknown. */
#endif /* CLOCK_* */
        if ( id != (clockid_t)-1 && clock_gettime( id, &ts ) != -1 )
            return (double)ts.tv_sec +
                (double)ts.tv_nsec / 1000000000.0;
        /* Fall thru. */
    }
#endif /* _POSIX_TIMERS */

    /* AIX, BSD, Cygwin, HP-UX, Linux, OSX, POSIX, Solaris. ----- */
    struct timeval tm;
    gettimeofday( &tm, NULL );
    return (double)tm.tv_sec + (double)tm.tv_usec / 1000000.0;
#else
    return -1.0;        /* Failed. */
#endif
}


void Common::glPyramid ( const Eigen::Vector3d &p, const Eigen::Vector3d &l, const Eigen::Vector3d &t, const Eigen::Vector3d &o,
    const Eigen::Vector3d &light, const Eigen::Vector3d &dim, const Eigen::Vector3d &eccentricity,
    const Eigen::Vector4f &cor)
{
    float f, dotP;
    Eigen::Vector3d normal;

    // LO
    float inv_mod = Common::finvsqrt(eccentricity.x()*dim.x()*eccentricity.x()*dim.x() + dim.z()*eccentricity.z()*dim.z()*eccentricity.z());
    normal[0] = ( ((dim.z()>0)-(dim.z()<0))* o.x()*eccentricity.x()*dim.x() + l.x()*abs(dim.z())*eccentricity.z() )*inv_mod;
    normal[1] = ( ((dim.z()>0)-(dim.z()<0))* o.y()*eccentricity.x()*dim.x() + l .y()*abs(dim.z())*eccentricity.z() )*inv_mod;
    normal[2] = ( ((dim.z()>0)-(dim.z()<0))* o.z()*eccentricity.x()*dim.x() + l.z()*abs(dim.z())*eccentricity.z() )*inv_mod;

    if ( (dotP = light.dot(normal)) > 0 )
        f = 0.8 * static_cast<float>(dotP) + 0.2;
    else
        f = 0.2;

    glColor4f( cor.x() * f, cor.y() * f, cor.z() * f, cor.w() );

    glBegin( GL_TRIANGLES );

    glVertex3d( p.x() +dim.y()*eccentricity.y()*t.x() +dim.x()*eccentricity.x()*l.x() +dim.z()*eccentricity.z()*o.x(),
                p.y() +dim.y()*eccentricity.y()*t.y() +dim.x()*eccentricity.x()*l.y() +dim.z()*eccentricity.z()*o.y(),
                p.z() +dim.y()*eccentricity.y()*t.z() +dim.x()*eccentricity.x()*l.z() +dim.z()*eccentricity.z()*o.z());// 1/7
    glVertex3d( p.x() -dim.y()*(1-eccentricity.y())*t.x() +dim.x()*eccentricity.x()*l.x() +dim.z()*eccentricity.z()*o.x(),
                p.y() -dim.y()*(1-eccentricity.y())*t.y() +dim.x()*eccentricity.x()*l.y() +dim.z()*eccentricity.z()*o.y(),
                p.z() -dim.y()*(1-eccentricity.y())*t.z() +dim.x()*eccentricity.x()*l.z() +dim.z()*eccentricity.z()*o.z() );// 2/8
    glVertex3d( p.x() +dim.z()*o.x(),
                p.y() +dim.z()*o.y(),
                p.z() +dim.z()*o.z() );// 3
    // -TO
    inv_mod = Common::finvsqrt(dim.y()*(1-eccentricity.y())*dim.y()*(1-eccentricity.y()) + dim.z()*eccentricity.z()*dim.z()*eccentricity.z());
    normal[0] = (((dim.z()>0)-(dim.z()<0))*o.x()*dim.y()*(1-eccentricity.y()) - t.x()*abs(dim.z())*eccentricity.z())*inv_mod;
    normal[1] = (((dim.z()>0)-(dim.z()<0))*o.y()*dim.y()*(1-eccentricity.y()) - t.y()*abs(dim.z())*eccentricity.z())*inv_mod;
    normal[2] = (((dim.z()>0)-(dim.z()<0))*o.z()*dim.y()*(1-eccentricity.y()) - t.z()*abs(dim.z())*eccentricity.z())*inv_mod;

    if ( (dotP = light.dot(normal)) > 0 )
        f = 0.8 * static_cast<float>(dotP) + 0.2;
    else
        f = 0.2;

    glColor4f( cor.x() * f, cor.y() * f, cor.z() * f, cor.w() );

    glVertex3d( p.x() -dim.y()*(1-eccentricity.y())*t.x() +dim.x()*eccentricity.x()*l.x() +dim.z()*eccentricity.z()*o.x(),
                p.y() -dim.y()*(1-eccentricity.y())*t.y() +dim.x()*eccentricity.x()*l.y() +dim.z()*eccentricity.z()*o.y(),
                p.z() -dim.y()*(1-eccentricity.y())*t.z() +dim.x()*eccentricity.x()*l.z() +dim.z()*eccentricity.z()*o.z() );// 2/8
    glVertex3d( p.x() +dim.z()*o.x(),
                p.y() +dim.z()*o.y(),
                p.z() +dim.z()*o.z() );// 3
    glVertex3d( p.x() -dim.y()*(1-eccentricity.y())*t.x() -dim.x()*(1-eccentricity.x())*l.x() +dim.z()*eccentricity.z()*o.x(),
                p.y() -dim.y()*(1-eccentricity.y())*t.y() -dim.x()*(1-eccentricity.x())*l.y() +dim.z()*eccentricity.z()*o.y(),
                p.z() -dim.y()*(1-eccentricity.y())*t.z() -dim.x()*(1-eccentricity.x())*l.z() +dim.z()*eccentricity.z()*o.z() );// 4

    // -LO
    inv_mod = Common::finvsqrt((1-eccentricity.x())*dim.x()*(1-eccentricity.x())*dim.x() + dim.z()*eccentricity.z()*dim.z()*eccentricity.z());
    normal[0] = (((dim.z()>0)-(dim.z()<0))*o.x()*(1-eccentricity.x())*dim.x() - l.x()*abs(dim.z())*eccentricity.z())*inv_mod;
    normal[1] = (((dim.z()>0)-(dim.z()<0))*o.y()*(1-eccentricity.x())*dim.x() - l.y()*abs(dim.z())*eccentricity.z())*inv_mod;
    normal[2] = (((dim.z()>0)-(dim.z()<0))*o.z()*(1-eccentricity.x())*dim.x() - l.z()*abs(dim.z())*eccentricity.z())*inv_mod;

    if ( (dotP = light.dot(normal)) > 0 )
        f = 0.8 * static_cast<float>(dotP) + 0.2;
    else
        f = 0.2;

    glColor4f( cor.x() * f, cor.y() * f, cor.z() * f, cor.w() );

    glVertex3d( p.x() +dim.z()*o.x(),
                p.y() +dim.z()*o.y(),
                p.z() +dim.z()*o.z() );// 3
    glVertex3d( p.x() -dim.y()*(1-eccentricity.y())*t.x() -dim.x()*(1-eccentricity.x())*l.x() +dim.z()*eccentricity.z()*o.x(),
                p.y() -dim.y()*(1-eccentricity.y())*t.y() -dim.x()*(1-eccentricity.x())*l.y() +dim.z()*eccentricity.z()*o.y(),
                p.z() -dim.y()*(1-eccentricity.y())*t.z() -dim.x()*(1-eccentricity.x())*l.z() +dim.z()*eccentricity.z()*o.z() );// 4
    glVertex3d( p.x() +dim.y()*eccentricity.y()*t.x() -dim.x()*(1-eccentricity.x())*l.x() +dim.z()*eccentricity.z()*o.x(),
                p.y() +dim.y()*eccentricity.y()*t.y() -dim.x()*(1-eccentricity.x())*l.y() +dim.z()*eccentricity.z()*o.y(),
                p.z() +dim.y()*eccentricity.y()*t.z() -dim.x()*(1-eccentricity.x())*l.z() +dim.z()*eccentricity.z()*o.z() );// 5

    // -L-O
    inv_mod = Common::finvsqrt((1-eccentricity.x())*dim.x()*(1-eccentricity.x())*dim.x() + dim.z()*(1-eccentricity.z())*dim.z()*(1-eccentricity.z()));
    normal[0] = (((dim.z()>0)-(dim.z()<0))*-o.x()*(1-eccentricity.x())*dim.x() - l.x()*abs(dim.z())*(1-eccentricity.z()))*inv_mod;
    normal[1] = (((dim.z()>0)-(dim.z()<0))*-o.y()*(1-eccentricity.x())*dim.x() - l.y()*abs(dim.z())*(1-eccentricity.z()))*inv_mod;
    normal[2] = (((dim.z()>0)-(dim.z()<0))*-o.z()*(1-eccentricity.x())*dim.x() - l.z()*abs(dim.z())*(1-eccentricity.z()))*inv_mod;

    if ( (dotP = light.dot(normal)) > 0 )
        f = 0.8 * static_cast<float>(dotP) + 0.2;
    else
        f = 0.2;

    glColor4f( cor.x() * f, cor.y() * f, cor.z() * f, cor.w() );

    glVertex3d( p.x() -dim.y()*(1-eccentricity.y())*t.x() -dim.x()*(1-eccentricity.x())*l.x() +dim.z()*eccentricity.z()*o.x(),
                p.y() -dim.y()*(1-eccentricity.y())*t.y() -dim.x()*(1-eccentricity.x())*l.y() +dim.z()*eccentricity.z()*o.y(),
                p.z() -dim.y()*(1-eccentricity.y())*t.z() -dim.x()*(1-eccentricity.x())*l.z() +dim.z()*eccentricity.z()*o.z() );// 4
    glVertex3d( p.x() +dim.y()*eccentricity.y()*t.x() -dim.x()*(1-eccentricity.x())*l.x() +dim.z()*eccentricity.z()*o.x(),
                p.y() +dim.y()*eccentricity.y()*t.y() -dim.x()*(1-eccentricity.x())*l.y() +dim.z()*eccentricity.z()*o.y(),
                p.z() +dim.y()*eccentricity.y()*t.z() -dim.x()*(1-eccentricity.x())*l.z() +dim.z()*eccentricity.z()*o.z() );// 5
    glVertex3d( p.x(),
                p.y(),
                p.z() );// 6
    // T-O
    inv_mod = Common::finvsqrt(dim.y()*eccentricity.y()*dim.y()*eccentricity.y() + dim.z()*(1-eccentricity.z())*dim.z()*(1-eccentricity.z()));
    normal[0] = (((dim.z()>0)-(dim.z()<0))*-o.x()*dim.y()*eccentricity.y() + t.x()*abs(dim.z())*(1-eccentricity.z()))*inv_mod;
    normal[1] = (((dim.z()>0)-(dim.z()<0))*-o.y()*dim.y()*eccentricity.y() + t.y()*abs(dim.z())*(1-eccentricity.z()))*inv_mod;
    normal[2] = (((dim.z()>0)-(dim.z()<0))*-o.z()*dim.y()*eccentricity.y() + t.z()*abs(dim.z())*(1-eccentricity.z()))*inv_mod;

    if ( (dotP = light.dot(normal)) > 0 )
        f = 0.8 * static_cast<float>(dotP) + 0.2;
    else
        f = 0.2;

    glColor4f( cor.x() * f, cor.y() * f, cor.z() * f, cor.w() );

    glVertex3d( p.x() +dim.y()*eccentricity.y()*t.x() -dim.x()*(1-eccentricity.x())*l.x() +dim.z()*eccentricity.z()*o.x(),
                p.y() +dim.y()*eccentricity.y()*t.y() -dim.x()*(1-eccentricity.x())*l.y() +dim.z()*eccentricity.z()*o.y(),
                p.z() +dim.y()*eccentricity.y()*t.z() -dim.x()*(1-eccentricity.x())*l.z() +dim.z()*eccentricity.z()*o.z() );// 5
    glVertex3d( p.x(),
                p.y(),
                p.z() );// 6
    glVertex3d( p.x() +dim.y()*eccentricity.y()*t.x() +dim.x()*eccentricity.x()*l.x() +dim.z()*eccentricity.z()*o.x(),
                p.y() +dim.y()*eccentricity.y()*t.y() +dim.x()*eccentricity.x()*l.y() +dim.z()*eccentricity.z()*o.y(),
                p.z() +dim.y()*eccentricity.y()*t.z() +dim.x()*eccentricity.x()*l.z() +dim.z()*eccentricity.z()*o.z());// 1/7
    
    // L-O
    inv_mod = Common::finvsqrt(eccentricity.x()*dim.x()*eccentricity.x()*dim.x() + dim.z()*(1-eccentricity.z())*dim.z()*(1-eccentricity.z()));
    normal[0] = (((dim.z()>0)-(dim.z()<0))*-o.x()*eccentricity.x()*dim.x() + l.x()*abs(dim.z())*(1-eccentricity.z()))*inv_mod;
    normal[1] = (((dim.z()>0)-(dim.z()<0))*-o.y()*eccentricity.x()*dim.x() + l.y()*abs(dim.z())*(1-eccentricity.z()))*inv_mod;
    normal[2] = (((dim.z()>0)-(dim.z()<0))*-o.z()*eccentricity.x()*dim.x() + l.z()*abs(dim.z())*(1-eccentricity.z()))*inv_mod;

    if ( (dotP = light.dot(normal)) > 0 )
        f = 0.8 * static_cast<float>(dotP) + 0.2;
    else
        f = 0.2;

    glColor4f( cor.x() * f, cor.y() * f, cor.z() * f, cor.w() );

    glVertex3d( p.x(),
                p.y(),
                p.z() );// 6
    glVertex3d( p.x() +dim.y()*eccentricity.y()*t.x() +dim.x()*eccentricity.x()*l.x() +dim.z()*eccentricity.z()*o.x(),
                p.y() +dim.y()*eccentricity.y()*t.y() +dim.x()*eccentricity.x()*l.y() +dim.z()*eccentricity.z()*o.y(),
                p.z() +dim.y()*eccentricity.y()*t.z() +dim.x()*eccentricity.x()*l.z() +dim.z()*eccentricity.z()*o.z());// 1/7
    glVertex3d( p.x() -dim.y()*(1-eccentricity.y())*t.x() +dim.x()*eccentricity.x()*l.x() +dim.z()*eccentricity.z()*o.x(),
                p.y() -dim.y()*(1-eccentricity.y())*t.y() +dim.x()*eccentricity.x()*l.y() +dim.z()*eccentricity.z()*o.y(),
                p.z() -dim.y()*(1-eccentricity.y())*t.z() +dim.x()*eccentricity.x()*l.z() +dim.z()*eccentricity.z()*o.z() );// 2/8

    // TO
    inv_mod = Common::finvsqrt(dim.y()*eccentricity.y()*dim.y()*eccentricity.y() + dim.z()*eccentricity.z()*dim.z()*eccentricity.z());
    normal[0] = (((dim.z()>0)-(dim.z()<0))*o.x()*dim.y()*eccentricity.y() + t.x()*abs(dim.z())*eccentricity.z())*inv_mod;
    normal[1] = (((dim.z()>0)-(dim.z()<0))*o.y()*dim.y()*eccentricity.y() + t.y()*abs(dim.z())*eccentricity.z())*inv_mod;
    normal[2] = (((dim.z()>0)-(dim.z()<0))*o.z()*dim.y()*eccentricity.y() + t.z()*abs(dim.z())*eccentricity.z())*inv_mod;

    if ( (dotP = light.dot(normal)) > 0 )
        f = 0.8 * static_cast<float>(dotP) + 0.2;
    else
        f = 0.2;

    glColor4f( cor.x() * f, cor.y() * f, cor.z() * f, cor.w() );

    glVertex3d( p.x() +dim.y()*eccentricity.y()*t.x() +dim.x()*eccentricity.x()*l.x() +dim.z()*eccentricity.z()*o.x(),
                p.y() +dim.y()*eccentricity.y()*t.y() +dim.x()*eccentricity.x()*l.y() +dim.z()*eccentricity.z()*o.y(),
                p.z() +dim.y()*eccentricity.y()*t.z() +dim.x()*eccentricity.x()*l.z() +dim.z()*eccentricity.z()*o.z());// 1/7
    glVertex3d( p.x() +dim.z()*o.x(),
                p.y() +dim.z()*o.y(),
                p.z() +dim.z()*o.z() );// 3
    glVertex3d( p.x() +dim.y()*eccentricity.y()*t.x() -dim.x()*(1-eccentricity.x())*l.x() +dim.z()*eccentricity.z()*o.x(),
                p.y() +dim.y()*eccentricity.y()*t.y() -dim.x()*(1-eccentricity.x())*l.y() +dim.z()*eccentricity.z()*o.y(),
                p.z() +dim.y()*eccentricity.y()*t.z() -dim.x()*(1-eccentricity.x())*l.z() +dim.z()*eccentricity.z()*o.z() );// 5

    // -T-O
    inv_mod = Common::finvsqrt(dim.y()*(1-eccentricity.y())*dim.y()*(1-eccentricity.y()) + dim.z()*(1-eccentricity.z())*dim.z()*(1-eccentricity.z()));
    normal[0] = (((dim.z()>0)-(dim.z()<0))*-o.x()*dim.y()*(1-eccentricity.y()) - t.x()*abs(dim.z())*(1-eccentricity.z()))*inv_mod;
    normal[1] = (((dim.z()>0)-(dim.z()<0))*-o.y()*dim.y()*(1-eccentricity.y()) - t.y()*abs(dim.z())*(1-eccentricity.z()))*inv_mod;
    normal[2] = (((dim.z()>0)-(dim.z()<0))*-o.z()*dim.y()*(1-eccentricity.y()) - t.z()*abs(dim.z())*(1-eccentricity.z()))*inv_mod;

    if ( (dotP = light.dot(normal)) > 0 )
        f = 0.8 * static_cast<float>(dotP) + 0.2;
    else
        f = 0.2;

    glColor4f( cor.x() * f, cor.y() * f, cor.z() * f, cor.w() );

    glVertex3d( p.x() -dim.y()*(1-eccentricity.y())*t.x() +dim.x()*eccentricity.x()*l.x() +dim.z()*eccentricity.z()*o.x(),
                p.y() -dim.y()*(1-eccentricity.y())*t.y() +dim.x()*eccentricity.x()*l.y() +dim.z()*eccentricity.z()*o.y(),
                p.z() -dim.y()*(1-eccentricity.y())*t.z() +dim.x()*eccentricity.x()*l.z() +dim.z()*eccentricity.z()*o.z() );// 2/8
    glVertex3d( p.x() -dim.y()*(1-eccentricity.y())*t.x() -dim.x()*(1-eccentricity.x())*l.x() +dim.z()*eccentricity.z()*o.x(),
                p.y() -dim.y()*(1-eccentricity.y())*t.y() -dim.x()*(1-eccentricity.x())*l.y() +dim.z()*eccentricity.z()*o.y(),
                p.z() -dim.y()*(1-eccentricity.y())*t.z() -dim.x()*(1-eccentricity.x())*l.z() +dim.z()*eccentricity.z()*o.z() );// 4
    glVertex3d( p.x(),
                p.y(),
                p.z() );// 6

    glEnd();

    //drawLTORef
    if (SHOWLTOREF)
    {
        glBegin( GL_LINES );
        glColor4f( 1.0f, 0.0f, 0.0f, 0.6f ); // RED => O
        glVertex3d( p.x(), p.y(), p.z() );//6
        glVertex3d( p.x()+abs(dim.z()/2)*o.x(), p.y()+abs(dim.z()/2)*o.y(), p.z()+abs(dim.z()/2)*o.z() );//6+d*o
        glColor4f( 0.0f, 1.0f, 0.0f, 0.6f ); // GREEN => T
        glVertex3d( p.x(), p.y(), p.z() );//6
        glVertex3d( p.x()+dim.y()/2*t.x(), p.y()+dim.y()/2*t.y(), p.z()+dim.y()/2*t.z() );//6+d*t
        glColor4f( 0.0f, 0.0f, 1.0f, 0.6f ); // BLUE => L X
        glVertex3d( p.x(), p.y(), p.z() );//6
        glVertex3d( p.x()+dim.x()/2*l.x(), p.y()+dim.x()/2*l.y(), p.z()+dim.x()/2*l.z() );//6+d*l
        glEnd();
    }
}


void Common::getWorkArea(int &height, int &width)
{
#if defined(_WIN32)
  NONCLIENTMETRICS metrics;
  metrics.cbSize = sizeof(metrics);
  RECT rect;
  SystemParametersInfo(SPI_GETNONCLIENTMETRICS, 0, &metrics, 0);
  SystemParametersInfo(SPI_GETWORKAREA, 0, &rect, 0);
	width = rect.right + 2*GetSystemMetrics(SM_CYFRAME);
	height = rect.bottom - 2*GetSystemMetrics(SM_CXFRAME) - metrics.iMenuHeight;

#elif defined(__unix__) || defined(__unix) || defined(unix) || (defined(__APPLE__) && defined(__MACH__))
	Display* disp = XOpenDisplay(NULL);
	Screen*  scrn = DefaultScreenOfDisplay(disp);
	height = scrn->height;
	width  = scrn->width;

//#if defined(__MACH__) && defined(__APPLE__)

#endif
}
