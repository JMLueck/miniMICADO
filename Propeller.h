#ifndef PROPELLER_H
#define PROPELLER_H

#define PI 3.14159265358979323846

#include <string>
#include <iostream>
#include <vector>
#include <math.h>
#include "node.h"
#include "functions.h"


class Propeller
{
    public:
        Propeller(string PropFile, string AirfoilFile);
        virtual ~Propeller();

        int NumberOfRadialStations;
        double Area;

        /** Geometry **/
        string PropFile;
        int NumberOfBlades;
        vector<double> Radius;
        vector<double> Chord;
        vector<double> theta;
        vector<double> dx;

        /** Polar **/
        string AirfoilFile;
        vector<double> Reynoldsnumber;
        vector<double> AngleOfAttack;
        vector<double> CL;
        vector<double> CD;

        void readPropFile();
        void readAirfoilFile();
        double getCL(double Re, double AoA);
        double getCD(double Re, double AoA);

    protected:

    private:
};

#endif // PROPELLER_H
