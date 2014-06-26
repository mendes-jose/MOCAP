/*!
 * Author:  José Magno MENDES FILHO
 * Site:    http://www.ensta-paristech.fr/~mendesfilho/public/
 * License: Creative Commons Attribution 3.0 Unported License
 *          http://creativecommons.org/licenses/by/3.0/
 */
#ifndef SCENE_H
#define SCENE_H

#include <iostream>
#include <GL/gl.h>
#include <GL/glu.h>
#include <GL/glut.h>
#include <omp.h>
#include <vector>
#include <atomic>         // atomic_uint
#include <thread>         // std::thread
#include <cstdio>					// sprintf
#include "rapidxml/rapidxml.hpp"
#include "bodypart.h"
#include "thoraxabdo.h"
#include "head.h"
#include "upperarm.h"
#include "lowerarm.h"
#include "upperleg.h"
#include "lowerleg.h"
#include "hand.h"
#include "foot.h"

using namespace std;
using namespace rapidxml;

/* 
 * \brief
 * Pointer to class Scene in order to do a "trampoline".
 *
 * This was a working around for the restriction of using classes methods as glut callback functions.
 * Glut callback functions have to be ordinary functions or static class methods, thus by creating this pointer
 * and several extra callback functions it is possible to call a static method for an instantiated object as if
 * it was really a method over an object call.
 * 
 * \remark
 * This will work only if there is a unique instantiated Scene object at a given moment (which is the case).
 * Other solutions involving mapping and others can be found for multiple instantiated objects.
 */
static class Scene* cInst;

class Scene
{
private:
    bool ESTIMATEGRAVITYATRUNTIME; /**< TODO */
    bool ESTIMATEMAGFIELDATRUNTIME; /**< TODO */
    unsigned char GH_NSAMPLES; /**< TODO */
    Vector3f G; /**< Gravity provided by the user */
    Vector3f H; /**< Geomagnetic Field provided by the user */

    Vector3f BACKGROUNDCOLOR; /**< TODO */
    bool SHOWXYZREF; /**< TODO */
    bool SHOWGROUNDGRID; /**< TODO */
    float GROUNDGRIDSQRSIZE; /**< TODO */
    float GROUNDGRIDTOTALSIZE; /**< TODO */
    Vector3f GROUNDGRIDCOLOR; /**< TODO */

    Vector4f RGBA_OFF; /**< Specifies the color of an offline body part */

    bool isFullScreenON; /**< TODO */

    Vector3f g_eye; /**< TODO */
    Vector3f g_center; /**< TODO */
    Vector3f g_eyeInit; /**< Salves the initial eye position for the reset point of view functionality */
    Vector3f g_centerInit; /**< Salves the initial center position for the reset point of view functionality */
    int g_MouseX, g_MouseY, g_Button; /**< TODO */

    static void drawScene (); /**< Manages the scene components drawing on the screen. This method is used on the glutDisplayFunc and glutIdleFunc */
    static void drawCallback (); /**< TODO */
    static void mouseButton ( int Button, int State, int x, int y ); /**< Manages mouse buttons functionalities */
    static void mouseButtonCallback ( int Button, int State, int x, int y ); /**< TODO */
    static void mouseMove ( int x, int y ); /**< Manages mouse movement functionalities */
    static void mouseMoveCallback ( int x, int y ); /**< TODO */
    static void keyboardAction ( unsigned char key, int x, int y ); /**< Manages keyboard functionalities */
    static void keyboardActionCallback ( unsigned char key, int x, int y );

    /*!
     * \brief
     * Gets tree attributes of a given node parsed from the config.xml file.
     * 
     * \param node
     * The node owning three attributes.
     * 
     * \returns
     * Returns a vector of R3 space containing the three attributes values.
     * 
     * \remarks
     * The names of these three attributes have to be on accord to the config.xml standard.
     */
    inline Vector3f get3attr ( const xml_node<> * node ) const;

    /*!
     * \brief
     * Gets a binary value of a given node parsed from the config.xml file.
     * 
     * \param node
     * The node having a binary value.
     * 
     * \returns
     * Return a boolean value according to the node value.
     * 
     * \remarks
     * The node value have to be on accord with the config.xml standard.
     */
    inline bool getBinValue ( const xml_node<> * node ) const;

    /*!
     * \brief
     * Parses and handles the (already loaded) config.xml file initializing all configuration parameters for the MOCAP execution.
     * 
     * \param xml_copy
     * A chain of characters corresponding to config.xml file. It must terminate with the '\0' character.
     * 
     */
    inline void initFromXML( vector<char> &xml_copy );

public:
    /* 
     * Macro for overload the operator new for fixed-size eigen structures avoiding the misaligning assert problem
     * see: http://eigen.tuxfamily.org/dox/TopicStructHavingEigenMembers.html
     */
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    vector<BodyPart*> *body; /**< Body part chain representing the whole human body */
    Scene(); /**< Override of the default class constructor */
    virtual ~Scene(); /**< Override of the default class destructor */
    
    /*!
     * \brief
     * Manages the window reshape functionality.
     * 
     * \param width
     * New window width.
     * 
     * \param height
     * New window height.
     */
    static void reshape ( int width, int height );

    void glutDisplayF_Draw ();
    void glutIdleF_Draw ();
    void glutMouseF_mouseButton ();
    void glutMotionF_mouseMove ();
    void glutKeyboardF_keyboardAction ();

    /*!
     * \brief
     * Loads the calibration values for each device associated with a body part and manages 
     * the call of each "orientation update" function associated with a online body part.
     * 
     * \param param
     * Pointer to the human body represented by a vector<BodyPart> data structure.
     */
    static void mainLoop ( void* param );
};

#endif // SCENE_H
