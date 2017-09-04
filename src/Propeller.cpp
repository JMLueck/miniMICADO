#include "Propeller.h"

Propeller::Propeller(node& configXML, string PropFile, string AirfoilAeroFile, string AirfoilGeoFile)
    :
    configXML(configXML),
    PropFile(PropFile),
    AirfoilAeroFile(AirfoilAeroFile),
    AirfoilGeoFile(AirfoilGeoFile),
    NumberOfBlades(configXML["NumberOfBlades"])
{
    //ctor
    readPropFile();
    readAirfoilGeometryFile();
    readAirfoilAeroFile();
    calcAR();
    calcTtoC();
}

void Propeller::calcAR()
{
    AR = Radius.back()/configXML["MACofBlades"];
}

void Propeller::readPropFile()
{
    myRuntimeInfo->out << "Read Propeller Geometry of " << PropFile << endl;
    //Einlesen der Propellerblattgeometrie aus einem CSV File
    string thisRow ="";

    //Open csv file
    ifstream csvread(PropFile.c_str());
    if (!csvread)
    {
        myRuntimeInfo->err << "Could not open " << PropFile <<endl;
        exit(1);
    }

    vector<string> cells(3,"");// (0) Radius (1) Chord length (2) Beta
    //Execute once to eliminate heading in csv file
    getline(csvread,thisRow);

    //Read all lines in csv file
    while(getline(csvread,thisRow) )
    {
        //Separate line string into string vector with delimiter ";" --> cells
        cells = split(thisRow, ';');

        //Auslesen der entsprechenden Werte
        Radius.push_back(atof(cells.at(0).c_str()));
        Chord.push_back(atof(cells.at(1).c_str()));
        theta.push_back(PI /180 * (atof(cells.at(2).c_str())));
        cells.clear();

    }
    dx.push_back(Radius.at(0));
    for (int i = 1; i < Radius.size(); i++)
    {
        dx.push_back(Radius.at(i) - Radius.at(i-1));
    }
    NumberOfRadialStations = Radius.size();
    Area = PI * Radius.back();
}

void Propeller::readAirfoilAeroFile()
{
    myRuntimeInfo->out << "Read Propeller Airfoil Aero Data of " << AirfoilAeroFile << endl;
    //Einlesen der Propellerblattgeometrie aus einem CSV File
    string thisRow ="";

    //Open csv file
    ifstream csvread(AirfoilAeroFile.c_str());
    if (!csvread)
    {
        myRuntimeInfo->err << "Could not open " << AirfoilAeroFile <<endl;
        exit(1);
    }

    vector<string> cells(4,"");// (0) Re (1) AoA (2) CL (3) CD
    //Execute once to eliminate heading in csv file
    getline(csvread,thisRow);

    //Read all lines in csv file
    while(getline(csvread,thisRow) )
    {
        //Separate line string into string vector with delimiter ";" --> cells
        cells = split(thisRow, ';');

        //Auslesen der entsprechenden Werte
        Reynoldsnumber.push_back(atof(cells.at(0).c_str()));
        AngleOfAttack.push_back(PI/180*atof(cells.at(1).c_str()));
        CL.push_back(atof(cells.at(2).c_str()));
        CD.push_back(atof(cells.at(3).c_str()));
        cells.clear();
    }
}

void Propeller::readAirfoilGeometryFile()
{
    myRuntimeInfo->out << "Read Propeller Airfoil Geometry of " << AirfoilGeoFile << endl;
    //Einlesen der Airfoilgeometrie aus einem CSV File
    string thisRow ="";

    //Open csv file
    ifstream csvread(AirfoilGeoFile.c_str());
    if (!csvread)
    {
        myRuntimeInfo->err << "Could not open " << AirfoilGeoFile <<endl;
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

void Propeller::calcTtoC()
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

double Propeller::getCL(double Re, double AoA)
{
    // Search for four lines of polar closest to the actual Re and AoA
    vector<int> interpolateLines;
    for (int i = 0; i < Reynoldsnumber.size(); i++)
    {
        if ( abs(Re - Reynoldsnumber.at(i)) < 50000 && abs(AoA - AngleOfAttack.at(i)) < 0.25*PI/180)
        {
            interpolateLines.push_back(i);
        }
    }
    // if Reynoldsnumber and AoA are matched exactly --> no interpolation needed
    if (interpolateLines.size() == 1)
    {
        double currentCL = CL.at(interpolateLines.at(0));
        return currentCL;
    }
    // if Reynoldsnumber or AoA is matched exactly --> only interpolation between the other
    else if (interpolateLines.size() == 2)
    {
        // Re is matched
        if (Reynoldsnumber.at(interpolateLines.at(0)) == Reynoldsnumber.at(interpolateLines.at(1)) )
        {
            double currentCL = CL.at(interpolateLines.at(0)) + ( CL.at(interpolateLines.at(1)) - CL.at(interpolateLines.at(0)) ) / ( AngleOfAttack.at(interpolateLines.at(1)) - AngleOfAttack.at(interpolateLines.at(0)) ) * (AoA - AngleOfAttack.at(interpolateLines.at(0)));
            return currentCL;
        }
        // AoA is matched
        else
        {
            double currentCL = CL.at(interpolateLines.at(0)) + ( CL.at(interpolateLines.at(1)) - CL.at(interpolateLines.at(0)) ) / ( Reynoldsnumber.at(interpolateLines.at(1)) - Reynoldsnumber.at(interpolateLines.at(0)) ) * (Re - Reynoldsnumber.at(interpolateLines.at(0)));
            return currentCL;
        }
    }
    // if neither Re nor AoA is matched --> interpolated between both
    else if (interpolateLines.size() == 4)
    {
        // Interpolate AoA
        double CL_tmp1 = CL.at(interpolateLines.at(0)) + ( CL.at(interpolateLines.at(1)) - CL.at(interpolateLines.at(0)) ) / ( AngleOfAttack.at(interpolateLines.at(1)) - AngleOfAttack.at(interpolateLines.at(0)) ) * (AoA - AngleOfAttack.at(interpolateLines.at(0)));
        double CL_tmp2 = CL.at(interpolateLines.at(2)) + ( CL.at(interpolateLines.at(3)) - CL.at(interpolateLines.at(2)) ) / ( AngleOfAttack.at(interpolateLines.at(3)) - AngleOfAttack.at(interpolateLines.at(2)) ) * (AoA - AngleOfAttack.at(interpolateLines.at(2)));


        // Interpolate Re
        double currentCL = CL_tmp1 + (CL_tmp2 - CL_tmp1) / ( Reynoldsnumber.at(interpolateLines.at(2)) - Reynoldsnumber.at(interpolateLines.at(0)) ) * ( Re - Reynoldsnumber.at(interpolateLines.at(0)) );
        return currentCL;
    }
    // if Re or AoA is out of Range of Polar-File --> interpolateLines is empty
    else
    {
        double currentCL;
        double CL2max = 1.19 * (1-pow(TtoC,2)) * (0.65 + 0.35 * pow(E,-(pow((9/AR),2.3))));
        double RCL2 = 1.632 - CL2max;
        double N2 = 1 + (CL2max/RCL2);

        if ( (AoA*180/PI) < 92)
        {
            currentCL = -0.032 * ((AoA*180/PI) - 92) - RCL2 * pow((92-(AoA*180/PI))/52,N2);
        }
        else
        {
            currentCL = -0.032 * ((AoA*180/PI) - 92) - RCL2 * pow(((AoA*180/PI)-92)/51,N2);
        }
        return currentCL;
      }
}

double Propeller::getCD(double Re, double AoA)
{
    // Search for four lines of polar closest to the actual Re and AoA
    vector<int> interpolateLines;
    for (int i = 0; i < Reynoldsnumber.size(); i++)
    {
        if ( abs(Re - Reynoldsnumber.at(i)) < 50000 && abs(AoA - AngleOfAttack.at(i)) < 0.25*PI/180)
        {
            interpolateLines.push_back(i);
        }
    }
    // if Reynoldsnumber and AoA are matched exactly --> no interpolation needed
    if (interpolateLines.size() == 1)
    {
        double currentCD = CD.at(interpolateLines.at(0));
        return currentCD;
    }
    // if Reynoldsnumber or AoA is matched exactly --> only interpolation between the other
    else if (interpolateLines.size() == 2)
    {
        // Re is matched
        if (Reynoldsnumber.at(interpolateLines.at(0)) == Reynoldsnumber.at(interpolateLines.at(1)) )
        {
            double currentCD = CD.at(interpolateLines.at(0)) + ( CD.at(interpolateLines.at(1)) - CD.at(interpolateLines.at(0)) ) / ( AngleOfAttack.at(interpolateLines.at(1)) - AngleOfAttack.at(interpolateLines.at(0)) ) * (AoA - AngleOfAttack.at(interpolateLines.at(0)));
            return currentCD;
        }
        // AoA is matched
        else
        {
            double currentCD = CD.at(interpolateLines.at(0)) + ( CD.at(interpolateLines.at(1)) - CD.at(interpolateLines.at(0)) ) / ( Reynoldsnumber.at(interpolateLines.at(1)) - Reynoldsnumber.at(interpolateLines.at(0)) ) * (Re - Reynoldsnumber.at(interpolateLines.at(0)));
            return currentCD;
        }
    }
    // if neither Re nor AoA is matched --> interpolated between both
    else if (interpolateLines.size() == 4)
    {
        // Interpolate AoA
        double CD_tmp1 = CD.at(interpolateLines.at(0)) + ( CD.at(interpolateLines.at(1)) - CD.at(interpolateLines.at(0)) ) / ( AngleOfAttack.at(interpolateLines.at(1)) - AngleOfAttack.at(interpolateLines.at(0)) ) * (AoA - AngleOfAttack.at(interpolateLines.at(0)));
        double CD_tmp2 = CD.at(interpolateLines.at(2)) + ( CD.at(interpolateLines.at(3)) - CD.at(interpolateLines.at(2)) ) / ( AngleOfAttack.at(interpolateLines.at(3)) - AngleOfAttack.at(interpolateLines.at(2)) ) * (AoA - AngleOfAttack.at(interpolateLines.at(2)));

        // Interpolate Re
        double currentCD = CD_tmp1 + (CD_tmp2 - CD_tmp1) / ( Reynoldsnumber.at(interpolateLines.at(2)) - Reynoldsnumber.at(interpolateLines.at(0)) ) * ( Re - Reynoldsnumber.at(interpolateLines.at(0)) );
        return currentCD;
    }
    // if Re or AoA is out of Range of Polar-File --> interpolateLines is empty
    else
    {
        double CD1max = CD.back(); // CD is assumed to be fairly independent from RE --> just take the last AoA
        double CD2max = 2.3 * pow(E,-(pow((0.65*TtoC),0.9))) * (0.52 + 0.48 * pow(E,-(pow((6.5/AR),1.1))));
        double AoA_0 = 0; // Platzhalter

        double currentCD = CD1max + (CD2max - CD1max) * sin(PI/2*(AoA - AngleOfAttack.back())/(PI/2 - AngleOfAttack.back()));

        return currentCD;
    }
}

Propeller::~Propeller()
{
    //dtor
}
