#ifndef WING_H
#define WING_H

#include <math.h>
#include <vector>
//#include <stdlib.h>
#include "node.h"
#include "functions.h"
#include "Fuselage.h"

class Wing
{
    public:
        Wing(node& configXML, Fuselage &myFuselage);
        virtual ~Wing();

        double WingArea;

        string WingAirfoilFile;
        vector<double> x_coord;
        vector<double> z_coord;
        double TtoC;

        vector<double> AVLresults;

        void readWingAirfoilFile();
        void calcTtoC();
        void calcWing(double AoA);
        void buildAVLInputFile();
        void buildAVLCommandFile(double AoA);
        void readAVLResults();

    protected:

    private:
        node& configXML;
        Fuselage *myFuselagePt;
};

#endif // WING_H
