#include "BladeElementTheory.h"

BladeElementTheory::BladeElementTheory()
{
    //ctor
}

void BladeElementTheory::calcBET(double v_hor, double v_vert, double altitude, double thrust, double delta)
{
    atmosphere myatmo;
    Propeller myProp("TestProp.csv","TestAirfoil.csv");
    // init
    double eps = 1;
    double eps1 = 1;
    double omega = 30;
    double v_i_old = thrust / (2 * myatmo.getDensity(altitude) * myProp.Area);

    v_res = sqrt(pow(v_hor,2) + pow(v_vert,2)); // effective velocity
    alpha = atan(v_vert/v_hor) * 180 / PI; // angle of effective velocity to horizon in degree

    while (eps > 0.01)
    {
        thrust_calculated = 0;
        Moment = 0;

        dThrust_r_psi_add = 0;
        dDrag_r_psi_add = 0;
        v_i_r_add = 0;
        for (int i = 0; i < myProp.NumberOfRadialStations; i++)
        {
            for (int psi=0; psi<360; psi++)
            {
                while (eps1 > 0.01)
                {
                    // induced velocity of blade section at given psi (iteration?)
                    v_i = (thrust / (2 * myatmo.getDensity(altitude) * myProp.Area *
                                     sqrt(pow(v_res*cos(alpha+delta),2) + pow(v_res*sin(alpha+delta)+v_i_old,2))))
                          * (1 + 1.2 * myProp.Radius.at(i) * cos(psi)); // inflow model (Leishman p.116)
                    eps1 = (v_i - v_i_old) / v_i_old;
                    v_i_old = v_i;
                }

                v_parallel = omega * myProp.Radius.at(i) + v_res * cos(delta+alpha) * sin(psi);
                v_perpendicular = v_res * sin(delta+alpha) + v_i;
                AoA = myProp.theta.at(i) - atan( v_perpendicular / v_parallel );
                Re = myatmo.getDensity(altitude) *  v_parallel * myProp.Chord.at(i) / myatmo.getViscosity(altitude);

                // thrust of blade section at given psi with given omega
                dThrust_r_psi = 0.5 * myatmo.getDensity(altitude)
                                * pow( v_parallel, 2 )
                                * myProp.getCL(Re,AoA)
                                * myProp.Chord.at(i) * myProp.dx.at(i);

                dDrag_r_psi = 0.5 * myatmo.getDensity(altitude)
                              * pow( v_parallel, 2 )
                              * myProp.getCD(Re,AoA)
                              * myProp.Chord.at(i) * myProp.dx.at(i);

                // thrust added up over one revolution
                dThrust_r_psi_add += dThrust_r_psi;
                dDrag_r_psi_add += dDrag_r_psi;
                v_i_r_add += v_i;
            }
            // average thrust of rotor blade section over one revolution
            dThrust_r_average = dThrust_r_psi_add / 360;
            dDrag_r_average = dDrag_r_psi_add / 360;
            v_i_r_average = v_i_r_add / 360;

            thrust_calculated += dThrust_r_average * myProp.NumberOfBlades;
            Moment += dDrag_r_average * myProp.Radius.at(i);
        }
        if (thrust_calculated > thrust)
        {
            omega--;
        }
        else if (thrust_calculated < thrust)
        {
            omega++;
        }
        eps = abs((thrust_calculated - thrust) / thrust);
    }
    Power = omega * Moment;
    cout << Power << endl;
}

BladeElementTheory::~BladeElementTheory()
{
    //dtor
}
