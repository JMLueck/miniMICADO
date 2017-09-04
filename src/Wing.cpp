#include "Wing.h"

Wing::Wing(node& configXML, Fuselage &myFuselage)
    :
    configXML(configXML),
    myFuselagePt(&myFuselage),
    WingAirfoilFile(configXML["WingAirfoilFile"]),
    AVLresults(4,0.0)
{
    WingArea = configXML["Wingspan"] * configXML["WingMAC"];
    readWingAirfoilFile();
    calcTtoC();
    //ctor
}

void Wing::readWingAirfoilFile()
{
    myRuntimeInfo->out << "Read Wing Geometry of " << WingAirfoilFile << endl;
    //Einlesen der Airfoilgeometrie aus einem CSV File
    string thisRow ="";

    //Open csv file
    ifstream csvread(WingAirfoilFile.c_str());
    if (!csvread)
    {
        myRuntimeInfo->err << "Could not open " << WingAirfoilFile <<endl;
        exit(1);
    }

    vector<string> cells(2,"");// (0) x (1) z
    //Execute once to eliminate heading in csv file
    getline(csvread,thisRow);

    //Read all lines in csv file
    while(getline(csvread,thisRow) )
    {
        //Separate line string into string vector with delimiter ";" --> cells
        cells = split(thisRow, ' ');

        //Auslesen der entsprechenden Werte
        x_coord.push_back(atof(cells.at(0).c_str()));
        z_coord.push_back(atof(cells.at(1).c_str()));
        cells.clear();
    }
}

void Wing::calcTtoC()
{
    vector<double> xdif;
    vector<double> zdif;
    vector<double> thickness;

    for (int i = 0; i < x_coord.size()/2; i++)
    {
        for (int j=0; j<x_coord.size(); j++)
        {
            xdif.push_back(abs(x_coord.at(j) - x_coord.at(i)));
            zdif.push_back(abs(z_coord.at(j) - z_coord.at(i)));
        }
        xdif.erase(std::find(xdif.begin(), xdif.end(), 0));
        zdif.erase(std::find(zdif.begin(), zdif.end(), 0));

        int pos = std::min_element(xdif.begin(), xdif.end()) - xdif.begin();

        thickness.push_back(zdif.at(pos));

        xdif.clear();
        zdif.clear();
    }
    TtoC = thickness.at(std::max_element(thickness.begin(), thickness.end()) - thickness.begin());
}

void Wing::calcWing(double AoA)
{
    buildAVLInputFile();
    buildAVLCommandFile(AoA);
    system("cmd.exe<AVLCommands.txt");
    getchar();
    readAVLResults();
}

void Wing::buildAVLInputFile()
{
      myRuntimeInfo->out << "Building AVL-File for Wing" << endl;

        string AVLInput("UAV_Wing.avl");
        ofstream AVL_input;
        AVL_input.open(AVLInput.c_str());

        if (!AVL_input)
        {
            myRuntimeInfo->err << AVLInput + " konnte nicht geoeffnet werden!";
            exit(1);
        }
        else
        {
            AVL_input << "Configuration \n\n";
            AVL_input << "#Mach \n" << 0 << "\n\n";
            AVL_input << "#IYsym IZsym Zsym \n0 0 0 \n\n";
            AVL_input << "#Sref Cref Bref \n" << WingArea << " " << configXML["WingMAC"] << " " << configXML["Wingspan"] << "\n\n";
            AVL_input << "#Xref Yref Zref \n0.0 0.0 0.0 \n\n";
            AVL_input << "#-----------------------------------\n";
            AVL_input << "SURFACE \nWing \n\n!Nchordwise Cspace Nspanwise Sspace \n12 1.0 40 -1.1 \n\n";
            AVL_input << "INDEX \n1 \n\nYDUPLICATE \n0 \n\nANGLE \n0 \n\nSCALE \n1.0 1.0 1.0 \n\nTRANSLATE \n0 0 0 \n\n";
            AVL_input << "SECTION \n#Xle Yle Zle Chord Ainc Nspanwise Sspace \n0 0 0 " << configXML["WingMAC"] << " 0\nNACA \n0012 \n\n";
            AVL_input << "SECTION \n#Xle Yle Zle Chord Ainc Nspanwise Sspace \n0 " << myFuselagePt->FuselageWidth/2 << " 0 " << configXML["WingMAC"] << " 0\nAFILE \n" << configXML["WingAirfoilFile"] << "\n\n";
            AVL_input << "SECTION \n#Xle Yle Zle Chord Ainc Nspanwise Sspace \n0 " << configXML["Wingspan"]/2 << " 0 " << configXML["WingMAC"] << " 0\nAFILE \n" << configXML["WingAirfoilFile"];
        }

        AVL_input.close();
        myRuntimeInfo->out << "Reference-AVL input file successfully generated" << endl;
}

void Wing::buildAVLCommandFile(double AoA)
{
    string AVLCommands = "AVLCommands.txt";
    ofstream AVL_commands;
    AVL_commands.open(AVLCommands.c_str());

    if (!AVL_commands)
        {
            myRuntimeInfo->err << AVLCommands + " konnte nicht geoeffnet werden!";
            exit(1);
        }
    else
        {
            AVL_commands << "avl.exe \nload UAV_Wing.avl \noper \na a " << AoA;
            AVL_commands << "\nx \nft WingTotalForces.txt \no \n\nquit \n";
        }

    AVL_commands.close();
}

void Wing::readAVLResults()
{
    string WingTotalForcesFile = "WingTotalForces.txt";
    double CDtot;
    double CDind;
    double oswald;
    double CL;
    string line;

    ifstream WingTotalForcesStream(WingTotalForcesFile.c_str());
    if(!WingTotalForcesStream)
    {
        myRuntimeInfo->err <<  "File " << WingTotalForcesFile << " not found" << endl;
    }

    if (WingTotalForcesStream)
    {
        while (getline(WingTotalForcesStream, line))
        {
            int CDtotCol = line.find("CDtot =");
            int CDindCol = line.find("CDind =");
            int OswaldCol = line.find("e =");
            int CLCol = line.find("CLtot =");

            if (CDtotCol != string::npos)
            {
                string CDtot_str = line.substr(CDtotCol + 7);
                CDtot = atof(CDtot_str.c_str());
            }
            if (CDindCol != string::npos)
            {
                string CDind_str = line.substr(CDindCol + 7);
                CDind = atof(CDind_str.c_str());
            }
            if (CLCol != string::npos)
            {
                string CL_str = line.substr(CLCol + 7);
                CL = atof(CL_str.c_str());
            }
            if (OswaldCol != string::npos)
            {
                string oswald_str = line.substr(OswaldCol + 3, line.find("|") - (OswaldCol + 3));
                oswald = atof(oswald_str.c_str());
            }
        }
    }
    else
    {
        myRuntimeInfo->err <<  "Read error Input" << endl;
    }
    WingTotalForcesStream.close();

    AVLresults.at(0) = CL;
    AVLresults.at(1) = CDtot;
    AVLresults.at(2) = CDind;
    AVLresults.at(3) = oswald;
}


Wing::~Wing()
{
    //dtor
}
