#include "Aerodynamics.h"

using namespace std;

/** Calculation of required power and energy --> possibly X_Rotor integration here**/

Aerodynamics::Aerodynamics()
{
    //ctor
}


/******************************** X-Rotor Functions **************************************/
void Aerodynamics::buildXRotorCommandFile(double altitude, double AoA, double thrust)
{
    const string xRotorCom = "xRotorCommands.txt";
    ofstream xRotorComStream(xRotorCom.c_str());

    xRotorComStream << "xrotor.exe" << endl;
    xRotorComStream << "load ecruzer1.prop" << endl;
    xRotorComStream << "atmo" << endl << (altitude/1000) << endl;
    xRotorComStream << "oper" << endl;
    xRotorComStream << "angle" << endl << AoA << endl;
    xRotorComStream << "thrust" << endl << thrust << endl << "p" << endl; // p for fixed pitch
    xRotorComStream << "write xRotorResults.dat" << endl << "o" << endl; // o for overwrite old file

    xRotorComStream.close();
}

void Aerodynamics::calcXRotor()
{
    system("cmd.exe<xRotorCommands.txt");
}

void Aerodynamics::readXRotorResults()
{
    string resultfile = "xRotorResults.dat";
    string line;

    ifstream xRotorResultsStream(resultfile.c_str());

    while (getline(xRotorResultsStream, line))
    {
        int powerCol = line.find("power(W)   :");

        if (powerCol != string::npos)
        {
            string power_str = line.substr(powerCol + 12, (line.find("torque") - (powerCol + 12)));
            power = atof(power_str.c_str());
        }
    }
}

double Aerodynamics::getRotorPower(double altitude, double AoA, double thrust)
{
    buildXRotorCommandFile(altitude, AoA, thrust);
    calcXRotor();
    readXRotorResults();

    return power;
}


/******************************** Drag Calculation **************************************/

vector<double> Aerodynamics::getDrag(double v_hor, double v_vert, double MTOW, double delta, double altitude)
{
    atmosphere myatmo;
    double alpha;

    vector<double> getDragResults (2,0.0);

    double v_res = sqrt(pow(v_hor,2) + pow(v_vert,2));
    // catch alpha for vertical flight
    if (v_hor != 0)
    {
        alpha = atan(v_vert/v_hor); // angle of effective velocity to horizon in degree
    }
    else
    {
        alpha = 90;
    }

    // f is a function of alpha and delta -> iteration needed
    double f_0 = (0.75 + 0.25 * cos(alpha)) * 0.004 * pow(MTOW,(2/3)); // this is still questionable
    double f_1 = f_0 / 300; // this is still questionable
    double f = f_0 + f_1 * (alpha + delta); // should be a parable of some sort...

    double drag = 0.5 * myatmo.getDensity(altitude) * pow(v_res,2) * f;

    getDragResults.at(0) = alpha;
    getDragResults.at(1) = drag;

    return getDragResults;
}

/******************************** Thrust Calculation **************************************/

vector<double> Aerodynamics::calcThrust(double a_hor, double a_vert, double m, double alpha, double drag)
{
    vector<double> calcThrustResults (2,0.0);

    double thrust_x = m * a_hor + drag * cos(alpha);
    double thrust_y = m * 9.81 + m * a_vert + drag * sin(alpha);

    double thrust = sqrt(pow(thrust_x,2) + pow(thrust_y,2));
    double delta = acos(thrust_y / thrust);

    calcThrustResults.at(0) = delta;
    calcThrustResults.at(1) = thrust;

    return calcThrustResults;
}

/******************************** Thrust and Drag Iteration *******************************/

vector<double> Aerodynamics::iterateThrustAndDrag(double v_hor, double v_vert, double a_hor, double a_vert, double m, double MTOW, double altitude)
{
    /** initialize **/
    double delta = 0;
    double eps = 1;
    double tmpThrustOld = 1;

    vector<double> iterateThrustAndDragResults (4,0.0);

    while ( eps > 0.005 )
    {
        vector<double> tmpDragResults = getDrag(v_hor, v_vert, MTOW, delta, altitude);
        vector<double> tmpThrust = calcThrust(a_hor, a_vert, m, tmpDragResults.at(0), tmpDragResults.at(1));

        eps = (tmpThrust.at(1) - tmpThrustOld)/tmpThrustOld;
        tmpThrustOld = tmpThrust.at(1);
        delta = tmpThrust.at(0);

        iterateThrustAndDragResults.at(0) = tmpDragResults.at(0); // alpha
        iterateThrustAndDragResults.at(1) = tmpDragResults.at(1); // drag
        iterateThrustAndDragResults.at(2) = tmpThrust.at(0); //delta
        iterateThrustAndDragResults.at(3) = tmpThrust.at(1); //thrust
    }
    return iterateThrustAndDragResults;
}

Aerodynamics::~Aerodynamics()
{
    //dtor
}
