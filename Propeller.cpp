#include "Propeller.h"

Propeller::Propeller(string PropFile, string AirfoilFile)
    :
    PropFile(PropFile),
    AirfoilFile(AirfoilFile),
    NumberOfBlades(3)
{
    //ctor
    readPropFile();
    readAirfoilFile();
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

void Propeller::readAirfoilFile()
{
    myRuntimeInfo->out << "Read Propeller Airfoil of " << AirfoilFile << endl;
    //Einlesen der Propellerblattgeometrie aus einem CSV File
    string thisRow ="";

    //Open csv file
    ifstream csvread(AirfoilFile.c_str());
    if (!csvread)
    {
        myRuntimeInfo->err << "Could not open " << AirfoilFile <<endl;
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
        AngleOfAttack.push_back(atof(cells.at(1).c_str()));
        CL.push_back(atof(cells.at(2).c_str()));
        CD.push_back(atof(cells.at(3).c_str()));
        cells.clear();
    }
}

double Propeller::getCL(double Re, double AoA)
{
    // Search for four lines of polar closest to the actual Re and AoA
    vector<int> interpolateLines;
    for (int i = 0; i < Reynoldsnumber.size(); i++)
    {
        if ( abs(Re - Reynoldsnumber.at(i)) < 50000 && abs(AoA - AngleOfAttack.at(i)) < 0.25)
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
        return 0; // TODO: does this make sense?
    }
}

double Propeller::getCD(double Re, double AoA)
{
    // Search for four lines of polar closest to the actual Re and AoA
    vector<int> interpolateLines;
    for (int i = 0; i < Reynoldsnumber.size(); i++)
    {
        if ( abs(Re - Reynoldsnumber.at(i)) < 50000 && abs(AoA - AngleOfAttack.at(i)) < 0.25)
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
        return 1; // TODO: does this make sense?
    }
}

Propeller::~Propeller()
{
    //dtor
}
