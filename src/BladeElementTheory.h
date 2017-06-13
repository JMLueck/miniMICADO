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
        BladeElementTheory();
        virtual ~BladeElementTheory();

        double v_res;
        double alpha;
        double v_i;
        double dThrust_r_psi;

        double omega;
        double dThrust_r_psi_add;
        double dThrust_r_average;
        double v_i_r_add;
        double v_i_r_average;
        double thrust_calculated;

        double Moment;
        double dDrag_r_psi_add;
        double dDrag_r_psi;
        double dDrag_r_average;

        double v_parallel;
        double v_perpendicular;
        double AoA;
        double Re;

        double Power;

        void calcBET(double v_hor, double v_vert, double altitude, double thrust, double delta);

    protected:

    private:
};

#endif // BLADEELEMENTTHEORY_H
