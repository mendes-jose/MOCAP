#include "scene.h"

Scene::Scene()
{
    body = new vector<BodyPart*>();

    string input_xml;
    string line;
    ifstream config("config/config.xml");

    // read file into input_xml
    while ( getline(config,line) )
        input_xml += line;

    // make a safe-to-modify copy of input_xml
    // (one should never modify the contents of a std::string directly)
    vector<char> xml_copy(input_xml.begin(), input_xml.end());
    xml_copy.push_back('\0');

    this->initFromXML(xml_copy);

    //OpenGL init
    // Screen is erased to gray
    glClearColor( BACKGROUNDCOLOR.x(), BACKGROUNDCOLOR.z(), BACKGROUNDCOLOR.z(), 1.0f );
    // Tampon Z erased to 1
    glClearDepth( 1.0 );
    // Tampon Z activation
    glEnable( GL_DEPTH_TEST );

    isFullScreenON = false;

    config.close();
}

Scene::~Scene()
{
    cInst = NULL;
    unsigned int i;
    for (i=0; i< this->body->size(); i++)
        delete this->body->at(i);
    delete this->body;
}

void Scene::mainLoop ( void* param ) //Call update for each member of body
{
    vector<BodyPart*> *thisBody = static_cast<vector<BodyPart*>* >(param);
    int bodySize = thisBody->size();

//    HANDLE *hThread = new HANDLE [bodySize];
		std::thread *hThread = new std::thread [bodySize];

    fstream *calibrationFile = new fstream [bodySize];

    MatrixXd imuData(3,bodySize*3);
    MatrixXd Calib(36, bodySize);
    MatrixXd magfield(3, bodySize);
    MatrixXd gravity(3, bodySize);
    gravity.setZero();
    magfield.setZero();

    bool error = false;

    unsigned char nDevicesOnline=0;
    for (int i=0; i<bodySize; i++)
        if ( thisBody->at(i)->is_online() )
            ++nDevicesOnline;

    if (nDevicesOnline > 0)
    {
        std::cout << "\n(A) Beyond this point the devices have to be on"<<
            " their predefined initial orientation and stay that way"<<
            " until you see the message (B).\nPress enter to proceed.\n";
        getchar();
				std::cout << "[...]\n";
    }
    nDevicesOnline=0;

    for ( int j=0; j < bodySize; j++ )
    {
        if ( thisBody->at(j)->is_online() )
        {
            char filename[50];
            char id[4];
            sprintf ( id, "%d", int(thisBody->at(j)->devID()) );
            string line;
std::cerr << "1\n";
            strcpy(filename, "calib/"); 
 	    			strcat(filename, id); strcat(filename, "AccCalib"); strcat(filename, ".txt");
            calibrationFile[j].open(filename, ios::in);
            if ( calibrationFile[j].is_open() )
            {
                for ( int i=0; i<9; i++ )
                {
                    getline(calibrationFile[j], line);
                    Calib(i,j) = atof(line.c_str());
                }
                calibrationFile[j].close();
            }
            else
            {
                std::cerr << "ERROR: Make sure you calibrated the box " << int(thisBody->at(j)->devID()) << endl;
                
                exit ( EXIT_FAILURE );
            }
std::cerr << "2\n";
            strcpy(filename, "calib/"); strcat(filename, id); strcat(filename, "AccVar"); strcat(filename, ".txt");
            calibrationFile[j].open(filename, ios::in);
            if ( calibrationFile[j].is_open() )
            {
                for ( int i=9; i<12; i++ )
                {
                    getline(calibrationFile[j], line);
                    Calib(i,j) = atof(line.c_str());
                }
                calibrationFile[j].close();
            }
            else
            {
                std::cerr << "ERROR: Make sure you calibrated the box " << int(thisBody->at(j)->devID()) << endl;
                
                exit ( EXIT_FAILURE );
            }
std::cerr << "3\n";
            strcpy(filename, "calib/"); strcat(filename, id); strcat(filename, "GyroCalib"); strcat(filename, ".txt");
            calibrationFile[j].open(filename, ios::in);
            if ( calibrationFile[j].is_open() )
            {
                for ( int i=12; i<21; i++ )
                {
                    getline(calibrationFile[j], line);
                    Calib(i,j) = atof(line.c_str());
                }
                calibrationFile[j].close();
            }
            else
            {
                std::cerr << "ERROR: Make sure you calibrated the box " << int(thisBody->at(j)->devID()) << endl;
                
                exit ( EXIT_FAILURE );
            }
std::cerr << "4\n";
            strcpy(filename, "calib/"); strcat(filename, id); strcat(filename, "GyroVar"); strcat(filename, ".txt");
            calibrationFile[j].open(filename, ios::in);
            if ( calibrationFile[j].is_open() )
            {
                for ( int i=21; i<24; i++ )
                {
                    getline(calibrationFile[j], line);
                    Calib(i,j) = atof(line.c_str());
                }
                calibrationFile[j].close();
            }
            else
            {
                std::cerr << "ERROR: Make sure you calibrated the box " << int(thisBody->at(j)->devID()) << endl;
                
                exit ( EXIT_FAILURE );
            }
std::cerr << "5\n";
            strcpy(filename, "calib/"); strcat(filename, id); strcat(filename, "MagCalib"); strcat(filename, ".txt");
            calibrationFile[j].open(filename, ios::in);
            if ( calibrationFile[j].is_open() )
            {
                for ( int i=24; i<33; i++ )
                {
                    getline(calibrationFile[j], line);
                    Calib(i,j) = atof(line.c_str());
                }
                calibrationFile[j].close();
            }
            else
            {
                std::cerr << "ERROR: Make sure you calibrated the box " << int(thisBody->at(j)->devID()) << endl;
                
                exit ( EXIT_FAILURE );
            }
std::cerr << "6\n";
            strcpy(filename, "calib/"); strcat(filename, id); strcat(filename, "MagVar"); strcat(filename, ".txt");
            calibrationFile[j].open(filename, ios::in);
            if ( calibrationFile[j].is_open() )
            {
                for ( int i=33; i<36; i++ )
                {
                    getline(calibrationFile[j], line);
                    Calib(i,j) = atof(line.c_str());
                }
                calibrationFile[j].close();
            }
            else
            {
                std::cerr << "ERROR: Make sure you calibrated the box " << int(thisBody->at(j)->devID()) << endl;
                
                exit ( EXIT_FAILURE );
            }
std::cerr << "7\n";
            Matrix3d gK;
            Vector3d gb;
            Matrix3d mK;
            Vector3d mb;
            if (cInst->ESTIMATEGRAVITYATRUNTIME)
            {
                gK << Calib(3,j),Calib(6,j),Calib(7,j),
                    Calib(6,j),Calib(4,j),Calib(8,j),
                    Calib(7,j),Calib(8,j),Calib(5,j);
                gb << Calib(0,j),Calib(1,j),Calib(2,j);
            }
std::cerr << "8\n";
            if (cInst->ESTIMATEMAGFIELDATRUNTIME)
            {
                mK << Calib(27,j),Calib(30,j),Calib(31,j),
                    Calib(30,j),Calib(28,j),Calib(32,j),
                    Calib(31,j),Calib(32,j),Calib(29,j);
                mb << Calib(24,j),Calib(25,j),Calib(26,j);
            }
std::cerr << "9\n";
            if ( cInst->ESTIMATEMAGFIELDATRUNTIME && cInst->ESTIMATEGRAVITYATRUNTIME )
            {                                
std::cerr << "10\n";
                for (int i=0; i < cInst->GH_NSAMPLES; i++) // mag field
                {
std::cerr << "10.0\n";
                    imuData.block<3,3>(0,3*j) = thisBody->at(j)->readIMU();
std::cerr << "10.1\n";
                    magfield.col(j) += mK * (imuData.col(MAGNETIC_FIELD + 3*j) - mb);
std::cerr << "10.2\n";
                    gravity.col(j) += -1* gK * (imuData.col(ACCELERATION + 3*j) - gb);
std::cerr << "10.3\n";
                }
std::cerr << "10.4\n";
                magfield.col(j) = thisBody->at(j)->ABC2XYZ()*magfield.col(j)/cInst->GH_NSAMPLES;
std::cerr << "10.5\n";
                gravity.col(j) = thisBody->at(j)->ABC2XYZ()*gravity.col(j)/cInst->GH_NSAMPLES;
std::cerr << "10.5\n";
            }
            else if ( cInst->ESTIMATEMAGFIELDATRUNTIME )
            {
std::cerr << "11\n";                          
                for (int i=0; i < cInst->GH_NSAMPLES; i++) // mag field
                {
//                  imuData.block<3,3>(0,3*j) = (thisBody->at(j)->devID()>Common::LARGESTBOXID) ?
//                      (thisBody->at(j)->DeviceSPh::readIMU()):(thisBody->at(j)->DeviceBox::readIMU(MAGNETIC_FIELD));
										imuData.block<3,3>(0,3*j) = thisBody->at(j)->DeviceSPh::readIMU();
                    magfield.col(j) += mK * (imuData.col(MAGNETIC_FIELD + 3*j) - mb);
                }
                magfield.col(j) = thisBody->at(j)->ABC2XYZ()*magfield.col(j)/cInst->GH_NSAMPLES;
                gravity.col(j) = cInst->G.cast<double>();
            }
            else
            {
std::cerr << "12\n";
                for (int i=0; i < cInst->GH_NSAMPLES; i++) // mag field
                {
//                    imuData.block<3,3>(0,3*j) = (thisBody->at(j)->devID()>Common::LARGESTBOXID) ?
//                        (thisBody->at(j)->DeviceSPh::readIMU()):(thisBody->at(j)->DeviceBox::readIMU(ACCELERATION));
										imuData.block<3,3>(0,3*j) = (thisBody->at(j)->DeviceSPh::readIMU());
                    gravity.col(j) += -1* gK * (imuData.col(ACCELERATION + 3*j) - gb);
                }
                gravity.col(j) = thisBody->at(j)->ABC2XYZ()*gravity.col(j)/cInst->GH_NSAMPLES;
                magfield.col(j) = cInst->H.cast<double>();
            }
            std::cerr << "Final do loop, antes das ultimas 3 linhas\n";
            // Save on each device
            thisBody->at(j)->writeCalib( Calib.col(j) );
            thisBody->at(j)->h() = magfield.col(j); // mG
            ++nDevicesOnline;
        }
				else
				{
					std::cerr << "Device " << int(thisBody->at(j)->devID()) << " is offline\n";
				}
    }
    
    gravity.col(0) = gravity.rowwise().sum()/nDevicesOnline;
    for ( int j=0; j < bodySize; j++ )
    {
        if ( thisBody->at(j)->is_online() )
            thisBody->at(j)->g() = gravity.col(0);
    }
    
    delete [] calibrationFile;

    if (nDevicesOnline > 0)
        std::cout << "(B) You may move the boxes now!\n";

    // A thread for each online box has to be created
    int i;
    for ( i=0, nDevicesOnline=0; i < int(bodySize); i++ )
    {
        if ( thisBody->at(i)->is_online() )
        {
            hThread[nDevicesOnline] = (thisBody->at(i)->KFType() == UKF) ?
              std::thread ( BodyPart::updateUKF, (void *) thisBody->at(i) ) :
							std::thread ( BodyPart::updateEKF, (void *) thisBody->at(i) );
            ++nDevicesOnline;
        }
    }

		for ( i=0; i < nDevicesOnline; i++ )
		{
			hThread[i].join();
		}
    std::clog << "All body parts are offline now\n";
}

#pragma managed(pop)

void Scene::initFromXML( vector<char> &xml_copy )
{
    // Body parts temp pointers
    Thoraxabdo *thorax=NULL; // root
    Head *head=NULL;
    Upperarm *urarm=NULL;
    Upperarm *ularm=NULL;
    Lowerarm *lrarm=NULL;
    Lowerarm *llarm=NULL;
    Hand *rhand=NULL;
    Hand *lhand=NULL;
    Upperleg *urleg=NULL;
    Upperleg *ulleg=NULL;
    Lowerleg *lrleg=NULL;
    Lowerleg *llleg=NULL;
    Foot *rfoot=NULL; 
    Foot *lfoot=NULL;

    Vector3d pos, Oori, Lori, Aori, Cori, rSize, ecc, refPos;
    Vector4f color;
    int deviceID, LPF;
    float UKFW, HHeight;
    bool KFType;

    xml_document<> doc;
    doc.parse<0>(&xml_copy[0]);
    xml_node<> *lv0node = doc.first_node("Config");

    for ( xml_node<> *lv1node = lv0node->first_node(); lv1node; lv1node = lv1node->next_sibling() ) //loop over body and other possible nodes of config
    {
        if (lv1node->name() == string("Camera"))
        {
            for ( xml_node<> * lv2node = lv1node->first_node(); lv2node; lv2node = lv2node->next_sibling() )
            {
                if ( lv2node->name() == string("Center") )
                    g_centerInit = g_center = get3attr(lv2node);
                else if ( lv2node->name() == string("Eye") )
                    g_eyeInit = g_eye = get3attr(lv2node);
                else
                {
                    std::cerr << "ERROR: Invalid node name \""<< lv2node->name() <<
                        "\" on the XML file.\nRead notes about the config.xml file to fix it.\n" << endl;
                    system("pause");
                    exit ( EXIT_FAILURE );
                }
            }
        }
        else if (lv1node->name() == string("Environment"))
        {
            for ( xml_node<> * lv2node = lv1node->first_node(); lv2node; lv2node = lv2node->next_sibling() )
            {
                if ( lv2node->name() == string("EstimateGravityAtRunTime") )
                    ESTIMATEGRAVITYATRUNTIME = getBinValue(lv2node);
                else if ( lv2node->name() == string("RelativizeMagFieldForEachDevice") )
                    ESTIMATEMAGFIELDATRUNTIME = getBinValue(lv2node);
                else if (lv2node->name() == string("NumberOfSamplesUsed"))
                    GH_NSAMPLES = atoi(lv2node->first_attribute("s")->value());
                else if (lv2node->name() == string("Gravity"))
                    G = get3attr(lv2node);
                else if (lv2node->name() == string("MagField"))
                    H = get3attr(lv2node);
                else
                {
                    std::cerr << "ERROR: Invalid node name \""<< lv2node->name() <<
                        "\" on the XML file.\nRead notes about the config.xml file to fix it.\n" << endl;
                    system("pause");
                    exit ( EXIT_FAILURE );
                }
            }
        }
        else if (lv1node->name() == string("BackGround"))        
        {
            for ( xml_node<> * lv2node = lv1node->first_node(); lv2node; lv2node = lv2node->next_sibling() )
            {
                if ( lv2node->name() == string("BackGroundColor") )
                    BACKGROUNDCOLOR = get3attr(lv2node);
                else if ( lv2node->name() == string("ShowXYZRef") )
                    SHOWXYZREF = getBinValue(lv2node);
                else if (lv2node->name() == string("ShowGroundGrid"))
                    SHOWGROUNDGRID = getBinValue(lv2node);
                else if (lv2node->name() == string("GroundGridSqrSize"))
                    GROUNDGRIDSQRSIZE = atof(lv2node->first_attribute("s")->value());
                else if (lv2node->name() == string("GroundGridTotalSize"))
                    GROUNDGRIDTOTALSIZE = atof(lv2node->first_attribute("s")->value());
                else if ( lv2node->name() == string("GroundGridColor") )
                    GROUNDGRIDCOLOR = get3attr(lv2node);
                else
                {
                    std::cerr << "ERROR: Invalid node name \""<< lv2node->name() <<
                        "\" on the XML file.\nRead notes about the config.xml file to fix it.\n" << endl;
                    system("pause");
                    exit ( EXIT_FAILURE );
                }
            }
        }

        else if (lv1node->name() == string("DeviceComm"))
        {
            for ( xml_node<> * lv2node = lv1node->first_node(); lv2node; lv2node = lv2node->next_sibling() )
            {
                if (lv2node->name() == string("RecvBoxDataWaitingTime"))
                    Common::RECVBOXDATAWAITINGTIME = atoi(lv2node->first_attribute("s")->value());
                else if (lv2node->name() == string("RecvBoxDataTimeout"))
                    Common::RECVBOXDATATIMEOUT = atoi(lv2node->first_attribute("s")->value());
                else if (lv2node->name() == string("LargestBoxID"))
                    Common::LARGESTBOXID = atoi(lv2node->first_attribute("s")->value());
                else if (lv2node->name() == string("RecvSmartPhoneDataWaitingTime"))
                    Common::RECVSMARTPHONEDATAWAITINGTIME = atoi(lv2node->first_attribute("s")->value());
                else if (lv2node->name() == string("RecvSmartPhoneDataTimeout"))
                    Common::RECVSMARTPHONEDATATIMEOUT = atoi(lv2node->first_attribute("s")->value());
                else if (lv2node->name() == string("TCPServerBasePort"))
                    Common::TCPSERVERBASEPORT = atoi(lv2node->first_attribute("s")->value());
                else if (lv2node->name() == string("TCPServerListeningTimeout"))
                    Common::TCPSERVERLISTENINGTIMEOUT = atoi(lv2node->first_attribute("s")->value());
                else
                {
                    std::cerr << "ERROR: Invalid node name \""<< lv2node->name() <<
                        "\" on the XML file.\nRead notes about the config.xml file to fix it.\n" << endl;
                    system("pause");
                    exit ( EXIT_FAILURE );
                }
            }
        }

        else if (lv1node->name() == string("GeneralFilterSettings"))
        {
            for ( xml_node<> * lv2node = lv1node->first_node(); lv2node; lv2node = lv2node->next_sibling() )
            {
                if (lv2node->name() == string("AccMagValidationGate"))
                    for ( xml_attribute<> *attr = lv2node->first_attribute(); attr; attr = attr->next_attribute() )
                    {
                        if ( attr->name() == string("epsa") )
                            Common::EPSA = atof(attr->value());
                        else if ( attr->name() == string("epsm") )
                            Common::EPSM = atof(attr->value());
                        else if ( attr->name() == string("epsdip") )
                            Common::EPSDIP = atof(attr->value());
                        else
                        {
                            std::cerr << "ERROR: Invalid attribute name \""<< attr->name() <<
                                "\" on the XML file.\nRead notes about the config.xml file to fix it.\n" << endl;
                            system("pause");
                            exit ( EXIT_FAILURE );
                        }
                    }
                    
                else if (lv2node->name() == string("HightCovValue"))
                    Common::HIGHTCOV = atof(lv2node->first_attribute("s")->value());
                else
                {
                    std::cerr << "ERROR: Invalid node name \""<< lv2node->name() <<
                        "\" on the XML file.\nRead notes about the config.xml file to fix it.\n" << endl;
                    system("pause");
                    exit ( EXIT_FAILURE );
                }
            }
        }

        else if ( lv1node->name() == string("Body") )
        {
            for ( xml_node<> * lv2node = lv1node->first_node(); lv2node; lv2node = lv2node->next_sibling() ) //loop over offlinecolor, height, bodyparts
            {
                if ( lv2node->name() == string("OfflineColor") )
                {
                    for ( xml_attribute<> *attr = lv2node->first_attribute(); attr; attr = attr->next_attribute() )
                    {
                        if ( attr->name() == string("r") )
                            RGBA_OFF(0) = atof(attr->value());
                        else if ( attr->name() == string("g") )
                            RGBA_OFF(1) = atof(attr->value());
                        else if ( attr->name() == string("b") )
                            RGBA_OFF(2) = atof(attr->value());
                        else if ( attr->name() == string("a") )
                            RGBA_OFF(3) = atof(attr->value());
                        else
                        {
                            std::cerr << "ERROR: Invalid attribute name \""<<
                                attr->name() <<
                                "\" on the XML file.\nRead notes about the config.xml standard to fix it.\n" << endl;
                            system("pause");
                            exit ( EXIT_FAILURE );
                        }
                    }
                }

                else if ( lv2node->name() == string("Height") )
                    HHeight = atof(lv2node->first_attribute("s")->value());
    
                else if ( lv2node->name() == string("RefPos") )
                    refPos = get3attr(lv2node).cast<double>();

                else if ( lv2node->name() == string("ShowLTORef") )
                    Common::SHOWLTOREF = getBinValue(lv2node);

                else if ( lv2node->name() == string("BodyParts") )
                {
                    unsigned int iAdp = 0;
                    for ( xml_node<> * lv3node = lv2node->first_node(); lv3node; lv3node = lv3node->next_sibling(), iAdp++ ) // loop over head, thorax, ...
                    {
                        for ( xml_node<> * lv4node = lv3node->first_node(); lv4node!=0; lv4node = lv4node->next_sibling() ) // loop over BoxID, Ecc, ...
                        {
                            if ( lv4node->name() == string("DeviceID") )
                                deviceID = atoi(lv4node->first_attribute("s")->value());
                            
                            else if ( lv4node->name() == string("KFType") )
                                KFType = getBinValue(lv4node);

                            else if ( lv4node->name() == string("UKFTune") )
                                UKFW = atof(lv4node->first_attribute("s")->value());

                            else if ( lv4node->name() == string("LPFTune") )
                                LPF = atoi(lv4node->first_attribute("s")->value());

                            else if ( lv4node->name() == string("DeviceInitOri") )
                            {
                                for ( xml_node<> * initOri = lv4node->first_node(); initOri; initOri = initOri->next_sibling() )
                                {
                                    if ( initOri->name() == string("A") )
                                        Aori = get3attr(initOri).cast<double>();
                                    else if ( initOri->name() == string("C") )
                                        Cori = get3attr(initOri).cast<double>();
                                    else
                                    {
                                        std::cerr << "ERROR: Invalid node name \""<<
                                            initOri->name() <<
                                            "\" on the XML file, inside the occurrence number " <<
                                            iAdp+1 << " of the node named \"" <<
                                            lv4node->name() <<
                                            "\".\nRead notes about the config.xml standard to fix it.\n" << endl;
                                        system("pause");
                                        exit (EXIT_FAILURE);
                                    }
                                }
                            }

                            else if ( lv4node->name() == string("BPInitOri") )
                            {
                                for ( xml_node<> * initOri = lv4node->first_node(); initOri; initOri = initOri->next_sibling() )
                                {
                                    if ( initOri->name() == string("L") )
                                        Lori = get3attr(initOri).cast<double>();
                                    else if ( initOri->name() == string("O") )
                                        Oori = get3attr(initOri).cast<double>();
                                    else
                                    {
                                        std::cerr << "ERROR: Invalid node name \""<<
                                            initOri->name() <<
                                            "\" on the XML file, inside the occurrence number " <<
                                            iAdp+1 << " of the node named \"" <<
                                            lv4node->name() <<
                                            "\".\nRead notes about the config.xml standard to fix it.\n" << endl;
                                        system("pause");
                                        exit (EXIT_FAILURE);
                                    }
                                }
                            }

                            else if ( lv4node->name() == string("Color") )
                                color << get3attr(lv4node), 1.0;

                            else if ( lv4node->name() == string("RelatifSize") )
                                rSize = get3attr(lv4node).cast<double>();

                            else if ( lv4node->name() == string("Eccentricity") )
                                ecc = get3attr(lv4node).cast<double>();

                            else
                            {
                                std::cerr << "ERROR: Invalid node name \""<<
                                    lv4node->name() <<
                                    "\" on the XML file, inside the node named \"" <<
                                    lv3node->name() <<
                                    "\".\nRead notes about the config.xml standard to fix it.\n" << endl;
                                system("pause");
                                exit (EXIT_FAILURE);
                            }
                        }

                        if ( lv3node->name() == string("Thorax") )
                        {
//                            if ( deviceID > int(Common::LARGESTBOXID) )
                                thorax = new Thoraxabdo ( deviceID, KFType, UKFW, LPF, refPos, Aori, Cori, Lori, Oori, color, rSize, HHeight, ecc );
//                            else
//                                thorax = new Thoraxabdo ( iAdp, deviceID, KFType, UKFW, LPF, refPos, Aori, Cori, Lori, Oori, color, rSize, HHeight, ecc );
                            if ( !thorax->is_online() )
                                thorax->RGBAcolor() = this->RGBA_OFF;
                            body->push_back(thorax);
                        }
                        else if ( lv3node->name() == string("Head") )
                        {
//                            if ( deviceID > int(Common::LARGESTBOXID) )
                                head = new Head ( deviceID, KFType, UKFW, LPF, thorax, Aori, Cori, Lori, Oori, color, rSize, HHeight, ecc );
//														else
//                                head = new Head ( iAdp, deviceID, KFType, UKFW, LPF, thorax, Aori, Cori, Lori, Oori, color, rSize, HHeight, ecc );
                            if ( !head->is_online() )
                                head->RGBAcolor() = this->RGBA_OFF;
                            body->push_back(head);
                        }
                        else if ( lv3node->name() == string("UpperRightArm") )
                        {
//                            if ( deviceID > int(Common::LARGESTBOXID) )
                                urarm = new Upperarm ( deviceID, KFType, UKFW, LPF, thorax, Aori, Cori, Lori, Oori, color, rSize, HHeight, ecc, RIGHT );
//                            else
//                                urarm = new Upperarm ( iAdp, deviceID, KFType, UKFW, LPF, thorax, Aori, Cori, Lori, Oori, color, rSize, HHeight, ecc, RIGHT );
                            if ( !urarm->is_online() )
                                urarm->RGBAcolor() = this->RGBA_OFF;
                            body->push_back(urarm);
                        }
                        else if ( lv3node->name() == string("LowerRightArm") )
                        {
//                            if ( deviceID > int(Common::LARGESTBOXID) )
                                lrarm = new Lowerarm ( deviceID, KFType, UKFW, LPF, urarm, Aori, Cori, Lori, Oori, color, rSize, HHeight, ecc );
//                            else
//                                lrarm = new Lowerarm ( iAdp, deviceID, KFType, UKFW, LPF, urarm, Aori, Cori, Lori, Oori, color, rSize, HHeight, ecc );
                            if ( !lrarm->is_online() )
                                lrarm->RGBAcolor() = this->RGBA_OFF;
                            body->push_back(lrarm);
                        }
                        else if ( lv3node->name() == string("RightHand") )
                        {
//                            if ( deviceID > int(Common::LARGESTBOXID) )
                                rhand = new Hand ( deviceID, KFType, UKFW, LPF, lrarm, Aori, Cori, Lori, Oori, color, rSize, HHeight, ecc );
//                            else
//                                rhand = new Hand ( iAdp, deviceID, KFType, UKFW, LPF, lrarm, Aori, Cori, Lori, Oori, color, rSize, HHeight, ecc );
                            if ( !rhand->is_online() )
                                rhand->RGBAcolor() = this->RGBA_OFF;
                            body->push_back(rhand);
                        }
                        else if ( lv3node->name() == string("UpperLeftArm") )
                        {
//                            if ( deviceID > int(Common::LARGESTBOXID) )
                                ularm = new Upperarm ( deviceID, KFType, UKFW, LPF, thorax, Aori, Cori, Lori, Oori, color, rSize, HHeight, ecc, LEFT );
//                            else
//                                ularm = new Upperarm ( iAdp, deviceID, KFType, UKFW, LPF, thorax, Aori, Cori, Lori, Oori, color, rSize, HHeight, ecc, LEFT );
                            if ( !ularm->is_online() )
                                ularm->RGBAcolor() = this->RGBA_OFF;
                            body->push_back(ularm);
                        }
                        else if ( lv3node->name() == string("LowerLeftArm") )
                        {
//                            if ( deviceID > int(Common::LARGESTBOXID) )
                                llarm = new Lowerarm ( deviceID, KFType, UKFW, LPF, ularm, Aori, Cori, Lori, Oori, color, rSize, HHeight, ecc );
//                            else
//                                llarm = new Lowerarm ( iAdp, deviceID, KFType, UKFW, LPF, ularm, Aori, Cori, Lori, Oori, color, rSize, HHeight, ecc );
                            if ( !llarm->is_online() )
                                llarm->RGBAcolor() = this->RGBA_OFF;
                            body->push_back(llarm);
                        }
                        else if ( lv3node->name() == string("LeftHand") )
                        {
//                            if ( deviceID > int(Common::LARGESTBOXID) )
                                lhand = new Hand ( deviceID, KFType, UKFW, LPF, llarm, Aori, Cori, Lori, Oori, color, rSize, HHeight, ecc );
//                            else
//                                lhand = new Hand ( iAdp, deviceID, KFType, UKFW, LPF, llarm, Aori, Cori, Lori, Oori, color, rSize, HHeight, ecc );
                            if ( !lhand->is_online() )
                                lhand->RGBAcolor() = this->RGBA_OFF;
                            body->push_back(lhand);
                        }
                        else if ( lv3node->name() == string("UpperLeftLeg") )
                        {
//                            if ( deviceID > int(Common::LARGESTBOXID) )
                                ulleg = new Upperleg ( deviceID, KFType, UKFW, LPF, thorax, Aori, Cori, Lori, Oori, color, rSize, HHeight, ecc, LEFT );
//                            else
//                                ulleg = new Upperleg ( iAdp, deviceID, KFType, UKFW, LPF, thorax, Aori, Cori, Lori, Oori, color, rSize, HHeight, ecc, LEFT );
                            if ( !ulleg->is_online() )
                                ulleg->RGBAcolor() = RGBA_OFF;
                            body->push_back(ulleg);
                        }
                        else if ( lv3node->name() == string("LowerLeftLeg") )
                        {
//                            if ( deviceID > int(Common::LARGESTBOXID) )
                                llleg = new Lowerleg ( deviceID, KFType, UKFW, LPF, ulleg, Aori, Cori, Lori, Oori, color, rSize, HHeight, ecc );
//                            else
//                                llleg = new Lowerleg ( iAdp, deviceID, KFType, UKFW, LPF, ulleg, Aori, Cori, Lori, Oori, color, rSize, HHeight, ecc );
                            if ( !llleg->is_online() )
                                llleg->RGBAcolor() = RGBA_OFF;
                            body->push_back(llleg);
                        }
                        else if ( lv3node->name() == string("LeftFoot") )
                        {
//                            if ( deviceID > int(Common::LARGESTBOXID) )
                                lfoot = new Foot ( deviceID, KFType, UKFW, LPF, llleg, Aori, Cori, Lori, Oori, color, rSize, HHeight, ecc );
//                            else
//                                lfoot = new Foot ( iAdp, deviceID, KFType, UKFW, LPF, llleg, Aori, Cori, Lori, Oori, color, rSize, HHeight, ecc );
                            if ( !lfoot->is_online() )
                                lfoot->RGBAcolor() = RGBA_OFF;
                            body->push_back(lfoot);
                        }
                        else if ( lv3node->name() == string("UpperRightLeg") )
                        {
//                            if ( deviceID > int(Common::LARGESTBOXID) )
                                urleg = new Upperleg ( deviceID, KFType, UKFW, LPF, thorax, Aori, Cori, Lori, Oori, color, rSize, HHeight, ecc, RIGHT );
//                            else
//                                urleg = new Upperleg ( iAdp, deviceID, KFType, UKFW, LPF, thorax, Aori, Cori, Lori, Oori, color, rSize, HHeight, ecc, RIGHT );
                            if ( !urleg->is_online() )
                                urleg->RGBAcolor() = RGBA_OFF;
                            body->push_back(urleg);
                        }
                        else if ( lv3node->name() == string("LowerRightLeg") )
                        {
//                            if ( deviceID > int(Common::LARGESTBOXID) )
                                lrleg = new Lowerleg ( deviceID, KFType, UKFW, LPF, urleg, Aori, Cori, Lori, Oori, color, rSize, HHeight, ecc );
//                            else
//                                lrleg = new Lowerleg ( iAdp, deviceID, KFType, UKFW, LPF, urleg, Aori, Cori, Lori, Oori, color, rSize, HHeight, ecc );
                            if ( !lrleg->is_online() )
                                lrleg->RGBAcolor() = RGBA_OFF;
                            body->push_back(lrleg);
                        }
                        else if ( lv3node->name() == string("RightFoot") )
                        {
//                            if ( deviceID > int(Common::LARGESTBOXID) )
                                rfoot = new Foot ( deviceID, KFType, UKFW, LPF, lrleg, Aori, Cori, Lori, Oori, color, rSize, HHeight, ecc );
//                            else
//                                rfoot = new Foot ( iAdp, deviceID, KFType, UKFW, LPF, lrleg, Aori, Cori, Lori, Oori, color, rSize, HHeight, ecc );
                            if ( !rfoot->is_online() )
                                rfoot->RGBAcolor() = RGBA_OFF;
                            body->push_back(rfoot);
                        }
                        else
                        {
                            std::cerr << "ERROR: Invalid node name \""<<
                                lv3node->name() <<
                                "\" on the XML file."<<
                                "\nRead notes about the config.xml standard to fix it.\n" << endl;
                            system("pause");
                            exit (EXIT_FAILURE);
                        }
                    }// loop over head, thorax etc
                }//endif BodyPart
            }//loop over off line color, height etc
        }//endif Body
    }//loop over Body etc
}

bool Scene::getBinValue ( const xml_node<> * node ) const
{
    if (node->value() == string("yes") || node->value() == string("true") || node->value() == string("UKF"))
        return true;
    else if (node->value() == string("no") || node->value() == string("false") || node->value() == string("EKF"))
        return false;
    else
    {
        std::cerr << "ERROR: Invalid node value \""<< node->value() << "for the node \"" << node->name() <<
            "\" on the XML file.\nRead notes about the config.xml file to fix it.\n" << endl;
        system("pause");
        exit ( EXIT_FAILURE );
    }
}

Vector3f Scene::get3attr ( const xml_node<> * node ) const
{
    Vector3f ret;
    for ( xml_attribute<> *attr = node->first_attribute(); attr; attr = attr->next_attribute() )
    {
        if (attr->name() == string("pitch") || attr->name() == string("r") ||
            attr->name() == string("R") || attr->name() == string("L") || 
            attr->name() == string("l") || attr->name() == string("x") ||
            attr->name() == string("X") )
            ret(0) = atof(attr->value());
        else if (attr->name() == string("g") || attr->name() == string("G") ||
            attr->name() == string("T") || attr->name() == string("t") ||
            attr->name() == string("y") ||  attr->name() == string("Y") ||
            attr->name() == string("yaw"))
            ret(1) = atof(attr->value());
        else if ( attr->name() == string("b") || attr->name() == string("B") ||
            attr->name() == string("O") || attr->name() == string("o") ||
            attr->name() == string("z") ||  attr->name() == string("Z") ||
            attr->name() == string("dist"))
            ret(2) = atof(attr->value());
        else
        {
            std::cerr << "ERROR: Unknown attribute name \"" <<
                attr->name() << "\" on the XML file, on the \"" << node->name() << "\" node. Read notes about this XML standard to fix it.\n";
            system("pause");
            exit (EXIT_FAILURE);
        }
    }
    return ret;
}

void Scene::reshape ( int width, int height )
{
    //EXTERNAL CODE
    // Mode matrice de projection.
    glMatrixMode( GL_PROJECTION );
    // La matrice de projection est la matrice identité
    glLoadIdentity();
    // multiplié par une matrice de perspective.
    gluPerspective( 60.0, (double)width / height, 0.1, 100.0 );
    // On retourne en mode matrice model view.
    glMatrixMode( GL_MODELVIEW );
    // On ajuste la nouvelle taille du viewport.
    glViewport( 0, 0, width, height );
}

void Scene::drawScene ()
{
    vector<BodyPart*> *body = cInst->body;
    float eyeX = cInst->g_eye.z() * cos( cInst->g_eye.x() ) * sin( cInst->g_eye.y() );
    float eyeY = cInst->g_eye.z() * cos( cInst->g_eye.x() ) * cos( cInst->g_eye.y() ) ;
    float eyeZ = cInst->g_eye.z() * sin( cInst->g_eye.x() );

    glClear( GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT );
    glLoadIdentity();
    gluLookAt( eyeX+cInst->g_center.x(), eyeY+cInst->g_center.y(), eyeZ+cInst->g_center.z(), cInst->g_center.x(), cInst->g_center.y(), cInst->g_center.z(), 0.0f, 0.0f, 1.0f );

    // drawGround
    if (cInst->SHOWGROUNDGRID)
    {
        glBegin(GL_LINES);
        glColor3f(cInst->GROUNDGRIDCOLOR.x(), cInst->GROUNDGRIDCOLOR.y(), cInst->GROUNDGRIDCOLOR.z());
        for (float i = -cInst->GROUNDGRIDTOTALSIZE/2; i <= cInst->GROUNDGRIDTOTALSIZE/2; i += cInst->GROUNDGRIDSQRSIZE)
        {
            glVertex3f(i, cInst->GROUNDGRIDTOTALSIZE/2, 0); glVertex3f(i, -cInst->GROUNDGRIDTOTALSIZE/2, 0);
            glVertex3f(cInst->GROUNDGRIDTOTALSIZE/2, i, 0); glVertex3f(-cInst->GROUNDGRIDTOTALSIZE/2, i, 0);
        }
        glEnd();
    }

    //drawXYZRef
    if (cInst->SHOWXYZREF)
    {
        glBegin( GL_LINES );
        glColor4f( 1.0f, 0.0f, 0.0f, 0.6f ); // RED => Z
        glVertex3f( .0f, .0f, .1f );
        glVertex3f( .0f, .0f, 1.8f );
        glColor4f( 0.0f, 1.0f, 0.0f, 0.6f ); // GREEN => Y
        glVertex3f( .0f, .0f, .1f );
        glVertex3f( .0f, 5.0f, .1f );
        glColor4f( 0.0f, 0.0f, 1.0f, 0.6f ); // BLUE => X
        glVertex3f( .0f, .0f, .1f );
        glVertex3f( 6.0f, .0f, .1f );
        glEnd();
    }

    // drawBody
    if ( body->size() > 0 )
    {
        for (int i=0; i < int(body->size()); i++) // CANNOT BE A PARALLEL FOR
        {
            if ( !body->at(i)->is_online() )
                body->at(i)->RGBAcolor() = cInst->RGBA_OFF;

            body->at(i)->drawMySelf(eyeX, eyeY, eyeZ);                
        }
    }

    glutSwapBuffers(); // Show the new scene
}

void Scene::mouseButton ( int Button, int State, int x, int y )
{
    cInst->g_Button = Button;
    cInst->g_MouseX = x;
    cInst->g_MouseY = y;
}

void Scene::mouseMove ( int x, int y )
{
    double DiffX = cInst->g_MouseX - x;
    double DiffY = cInst->g_MouseY - y;

    if( cInst->g_Button == GLUT_LEFT_BUTTON )
    {
        cInst->g_eye(0) = (std::abs(cInst->g_eye(0) - DiffY * .01) < .498*M_PI ) ? (cInst->g_eye(0) - DiffY*.01) : (cInst->g_eye(0));
        cInst->g_eye(1) -= DiffX * 0.01;
    }

    else if( cInst->g_Button == GLUT_MIDDLE_BUTTON )
        cInst->g_eye(2) = (cInst->g_eye(2) * ( 1 + DiffY * 0.01 ) > 0.01) ? (cInst->g_eye(2) * ( 1 + DiffY * 0.01 )) : (cInst->g_eye(2));

    else if( cInst->g_Button == GLUT_RIGHT_BUTTON )
    {
        float vx = static_cast<float>(cos( cInst->g_eye(0) ) * sin( cInst->g_eye(1) ));
        float vy = static_cast<float>(cos( cInst->g_eye(0) ) * cos( cInst->g_eye(1) ));
        float vz = static_cast<float>(sin( cInst->g_eye(0) ));
        float invnorm_a = Common::finvsqrt(vx*vz*vx*vz+vz*vy*vz*vy+vx*vx*vx*vx+2*vx*vx*vy*vy+vy*vy*vy*vy);
        float invnorm_b = Common::finvsqrt(vy*vy+vx*vx);
        float ax = -vx*vz*invnorm_a;
        float ay = -vz*vy*invnorm_a;
        float az = (vx*vx+vy*vy)*invnorm_a;
        float bx = -vy*invnorm_b;
        float by = vx*invnorm_b;
        cInst->g_center.x() += 0.02*(DiffX*bx - DiffY*ax);
        cInst->g_center.y() += 0.02*(DiffX*by - DiffY*ay);
        cInst->g_center.z() += 0.02*(-DiffY*az);
    }
    cInst->g_MouseX = x;
    cInst->g_MouseY = y;
}

void Scene::keyboardAction( unsigned char key, int i, int j )
{
    switch (key)
    {
    case 27:        //ESCAPE fullscreen
        if ( cInst->isFullScreenON )
        {
//            NONCLIENTMETRICS metrics;
//            metrics.cbSize = sizeof(metrics);
//            RECT rect;
//            SystemParametersInfo(SPI_GETNONCLIENTMETRICS, 0, &metrics, 0);
//            SystemParametersInfo(SPI_GETWORKAREA, 0, &rect, 0);

//            glutPositionWindow( rect.right/2 + GetSystemMetrics(SM_CYFRAME),
//                metrics.iMenuHeight + GetSystemMetrics(SM_CXFRAME)); 
//            glutReshapeWindow( rect.right/2 - 2*GetSystemMetrics(SM_CYFRAME),
//                rect.bottom - 2*GetSystemMetrics(SM_CXFRAME) - metrics.iMenuHeight);

            cInst->isFullScreenON = false;
        }
        break;
    case 'f':        //toggle fullscreen
        if ( cInst->isFullScreenON )
        {
//            NONCLIENTMETRICS metrics;
//            metrics.cbSize = sizeof(metrics);
//            RECT rect;
//            SystemParametersInfo(SPI_GETNONCLIENTMETRICS, 0, &metrics, 0);
//            SystemParametersInfo(SPI_GETWORKAREA, 0, &rect, 0);

//            glutPositionWindow( rect.right/2 + GetSystemMetrics(SM_CYFRAME),
//                metrics.iMenuHeight + GetSystemMetrics(SM_CXFRAME)); 
//            glutReshapeWindow( rect.right/2 - 2*GetSystemMetrics(SM_CYFRAME),
//                rect.bottom - 2*GetSystemMetrics(SM_CXFRAME) - metrics.iMenuHeight);
            
            cInst->isFullScreenON = false;
        }
        else
        {
            glutFullScreen();
            cInst->isFullScreenON = true;
        }
        break;
    case 'r':        //reset camera position and orientation
        cInst->g_eye = cInst->g_eyeInit;
        cInst->g_center = cInst->g_centerInit;
    }
}

void Scene::drawCallback ()
{
    cInst->drawScene();
}

void Scene::mouseButtonCallback ( int Button, int State, int x, int y )
{
    cInst->Scene::mouseButton ( Button, State, x, y );
}

void Scene::mouseMoveCallback ( int x, int y )
{
    cInst->mouseMove( x, y );
}

void Scene::keyboardActionCallback ( unsigned char key, int x, int y )
{
    cInst->keyboardAction (key, x, y);
}

void Scene::glutDisplayF_Draw ()
{
    cInst = this;
    glutDisplayFunc(Scene::drawCallback);
}

void Scene::glutIdleF_Draw ()
{
    cInst = this;
    glutIdleFunc(Scene::drawCallback);
}

void Scene::glutMouseF_mouseButton ()
{
    cInst = this;
    glutMouseFunc ( Scene::mouseButtonCallback );
}

void Scene::glutMotionF_mouseMove ()
{
    cInst = this;
    glutMotionFunc ( Scene::mouseMoveCallback );
}

void Scene::glutKeyboardF_keyboardAction ()
{
    cInst = this;
    glutKeyboardFunc ( Scene::keyboardActionCallback );
}
