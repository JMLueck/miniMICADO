#ifndef PROPELLER_H
#define PROPELLER_H

#define PI 3.14159265358979323846
#define E 2.718281828459045

#include <string>
#include <iostream>
#include <vector>
#include <math.h>
#include "node.h"
#include "functions.h"


class Propeller
{
    public:
        Propeller(node& configXML, string PropFile, string AirfoilAeroFile, string AirfoilGeoFile);
        virtual ~Propeller();

        int NumberOfRadialStations;
        double Area;
        double AR;

        /** Prop-Geometry **/
        string PropFile;
        int NumberOfBlades;
        vector<double> Radius;
        vector<double> Chord;
        vector<double> theta;
        vector<double> dx;

        /** Airfoil Geometry **/
        string AirfoilGeoFile;
        vector<double> x_coord;
        vector<double> z_coord;
        double TtoC;

        /** Polar **/
        string AirfoilAeroFile;
        vector<double> Reynoldsnumber;
        vector<double> AngleOfAttack;
        vector<double> CL;
        vector<double> CD;

        void readPropFile();
        void readAirfoilGeometryFile();
        void readAirfoilAeroFile();
        void calcTtoC();
        void calcAR();
        double getCL(double Re, double AoA);
        double getCD(double Re, double AoA);

    protected:

    private:
        node& configXML;
};

#endif // PROPELLER_H
