#ifndef WING_H
#define WING_H

#include <math.h>
#include <vector>
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

        void readWingAirfoilFile();
        void calcTtoC();
        void buildAVLInputFile();
        void buildAVLCommandFile();

    protected:

    private:
        node& configXML;
        Fuselage *myFuselagePt;
};

#endif // WING_H
