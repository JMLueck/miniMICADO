#ifndef BLADEELEMENTTHEORY_H
#define BLADEELEMENTTHEORY_H

#define PI 3.14159265358979323846
#include <math.h>
#include "functions.h"
#include "atmosphere.h"
#include "Propeller.h"

class BladeElementTheory
{
    public:
        BladeElementTheory(Propeller &myProp);
        virtual ~BladeElementTheory();

        Propeller *myPropPt;

        double v_res;
        double alpha;
        double v_i;
        double dThrust_r_psi;
        double dLift_r_psi;

        double omega;
        double dQ_r_psi;
        double dQ_r_psi_add;
        double dQ_r_average;
        double dThrust_r_psi_add;
        double dThrust_r_average;
        double v_i_r_average;
        double thrust_calculated;

        double Moment;
        double dDrag_r_psi_add;
        double dDrag_r_psi;
        double dDrag_r_average;

        double v_parallel;
        double v_perpendicular;
        double phi;
        double v_tot;
        double AoA;
        double Re;

        double Power;

        vector<double> calcBET(double v_hor, double v_vert, double altitude, double thrust, double delta, double omega_start);

    protected:

    private:
};

#endif // BLADEELEMENTTHEORY_H
