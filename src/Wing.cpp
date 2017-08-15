#include "Wing.h"

Wing::Wing(node& configXML, Fuselage &myFuselage)
    :
    configXML(configXML),
    myFuselagePt(&myFuselage),
    WingAirfoilFile(configXML["WingAirfoilFile"])
{
    WingArea = configXML["Wingspan"] * configXML["WingMAC"];

    readWingAirfoilFile();
    calcTtoC();
    buildAVLInputFile();
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
//        cout << zdif.at(pos) << endl;
//        getchar();

        xdif.clear();
        zdif.clear();
    }
    TtoC = thickness.at(std::max_element(thickness.begin(), thickness.end()) - thickness.begin());
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
            AVL_input << "SECTION \n#Xle Yle Zle Chord Ainc Nspanwise Sspace \n0 " << myFuselagePt->FuselageWidth/2 << " 0 " << configXML["WingMAC"] << " 0\nAFILE \n" << configXML["WingAirfoilFile"];
        }

        AVL_input.close();
        myRuntimeInfo->out << "Reference-AVL input file successfully generated" << endl;
}

void Wing::buildAVLCommandFile()
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
            // TODO: finish command file, decide on CL (which flight condition?)
            AVL_commands << "avl.exe \nload UAV_Wing.avl \noper \n";
        }

    AVL_commands.close();

}

// TODO: write function for read out of AVL-results

Wing::~Wing()
{
    //dtor
}
