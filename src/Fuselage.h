#ifndef FUSELAGE_H
#define FUSELAGE_H

#define PI 3.14159265358979323846

#include <math.h>
#include <vector>
#include "node.h"
#include "functions.h"

class Fuselage
{
    public:
        Fuselage(node& configXML);
        virtual ~Fuselage();

        double FuselageLength;
        double FuselageWidth;
        double FuselageWettedArea;

    protected:

    private:
        node& configXML;
};

#endif // FUSELAGE_H
