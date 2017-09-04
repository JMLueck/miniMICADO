#ifndef AERODYNAMICS_H
#define AERODYNAMICS_H

#include <string>
#include <iostream>
#include <vector>
#include <fstream>
#include <cstdlib>
#include <sstream>
#include "tgmath.h"
#include "atmosphere.h"
#include "node.h"
#include "Wing.h"

using namespace std;

class Aerodynamics
{
    public:
        Aerodynamics(node& configXML, Wing &myWing);
        virtual ~Aerodynamics();

        double power;

        vector<double> getDrag(double v_hor, double v_vert, double MTOW, double gamma, double altitude);
        vector<double> calcThrust(double a_Hor, double a_Vert, double m, double alpha, double drag);
        vector<double> iterateThrustAndDrag(double v_hor, double v_vert, double a_hor, double a_vert, double m, double MTOW, double altitude);

    protected:

    private:
        node& configXML;
        Wing *myWingPt;
};

#endif // AERODYNAMICS_H
