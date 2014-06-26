#include "devicesph.h"
#include "scene.h"
#include <thread>         // std::thread

int main( int argc, char** argv )
{
    // Initializations for the smartphone communication (inits sockets for the TCP server)
    DeviceSPh::init();

    // Glut initialization
    glutInit( &argc, argv );

    // By default only the half right side of the screen is used for the visualization. Reshape and fullscreen are enabled.
//    NONCLIENTMETRICS metrics;
//    metrics.cbSize = sizeof(metrics);
//    RECT rect;
//    SystemParametersInfo(SPI_GETNONCLIENTMETRICS, 0, &metrics, 0);
//    SystemParametersInfo(SPI_GETWORKAREA, 0, &rect, 0);

//    glutInitWindowPosition( rect.right/2 + GetSystemMetrics(SM_CYFRAME), 0 ); 
//    glutInitWindowSize( rect.right/2 - 2*GetSystemMetrics(SM_CYFRAME),
//                        rect.bottom - 2*GetSystemMetrics(SM_CXFRAME) - metrics.iMenuHeight);    

    glutInitDisplayMode( GLUT_RGBA | GLUT_DOUBLE | GLUT_DEPTH );
    glutCreateWindow( "3DVisualization" );
    
    // Enabling transparency
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
    glEnable( GL_BLEND );
    glClearColor(0.0,0.0,0.0,0.0);

    // Creating the scene which contains the background and the human body model
    Scene * scene = new Scene();
    // Calling the "scene main loop" to manage the changing of orientation and position of each body part
    std::thread track ( Scene::mainLoop, scene->body );

    scene->glutDisplayF_Draw (); // sets display function as been the Draw function
    scene->glutIdleF_Draw (); // sets idle function as been the Draw function
    scene->glutMouseF_mouseButton ();
    scene->glutMotionF_mouseMove ();
    scene->glutKeyboardF_keyboardAction ();
    glutReshapeFunc ( Scene::reshape ); // sets reshape function
    
    glutMainLoop(); // really bad design, I should call it from a new tread end it when the user chose to.

    // The following is never executed
    delete scene;

    // close all communications
//    DeviceBox::closeAll();
    DeviceSPh::closeAll();
//    system("pause");
    return EXIT_SUCCESS;
}
