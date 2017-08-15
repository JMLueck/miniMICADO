#include "BladeElementTheory.h"

BladeElementTheory::BladeElementTheory(Propeller &myProp)
    :
    myPropPt(&myProp)
{
    //ctor
}

vector<double> BladeElementTheory::calcBET(double v_hor, double v_vert, double altitude, double thrust, double delta, double omega_start)
{
    atmosphere myatmo;
    // init
    double eps = 1;
    double eps1 = 1;
    double omega = omega_start;
    double v_i_old = thrust / (2 * myatmo.getDensity(altitude) * myPropPt->Area);

    v_res = sqrt(pow(v_hor,2) + pow(v_vert,2)); // effective velocity
    // catch alpha for vertical flight
    if (v_hor != 0)
    {
        alpha = atan(v_vert/v_hor); // angle of effective velocity to horizon in rad
    }
    else
    {
        alpha = PI/2;
    }

    while (eps > 0.1)
    {
        thrust_calculated = 0;
        Moment = 0;

        dThrust_r_psi_add = 0;
        dDrag_r_psi_add = 0;
        for (int i = 0; i < myPropPt->NumberOfRadialStations; i++)
        {
            for (int psi=0; psi<360; psi++)
            {
                while (eps1 > 0.01)
                {
                    // induced velocity of blade section at given psi (iteration?)
                    v_i = (thrust / (2 * myatmo.getDensity(altitude) * myPropPt->Area *
                                     sqrt(pow(v_res*cos(alpha+delta),2) + pow(v_res*sin(alpha+delta)+v_i_old,2))))
                          * (1 + 1.2 * myPropPt->Radius.at(i) * cos(psi*PI/180)); // inflow model (Leishman p.116)
                    eps1 = (v_i - v_i_old) / v_i_old;
                    v_i_old = v_i;
                }

                v_parallel = omega * myPropPt->Radius.at(i) + v_res * cos(delta+alpha) * sin(psi*PI/180);
                v_perpendicular = v_res * sin(delta+alpha) + v_i;
                phi = atan( v_perpendicular / v_parallel );
                v_tot = sqrt(pow(v_parallel,2) + pow(v_perpendicular,2));
                AoA = myPropPt->theta.at(i) - phi;
                Re = myatmo.getDensity(altitude) *  v_parallel * myPropPt->Chord.at(i) / myatmo.getViscosity(altitude);

                // thrust of blade section at given psi with given omega
                dLift_r_psi = 0.5 * myatmo.getDensity(altitude)
                                * pow( v_tot, 2 )
                                * myPropPt->getCL(Re,AoA)
                                * myPropPt->Chord.at(i) * myPropPt->dx.at(i);

                dDrag_r_psi = 0.5 * myatmo.getDensity(altitude)
                              * pow( v_tot, 2 )
                              * myPropPt->getCD(Re,AoA)
                              * myPropPt->Chord.at(i) * myPropPt->dx.at(i);

                dThrust_r_psi = dLift_r_psi * cos(phi) - dDrag_r_psi * sin(phi);
                dQ_r_psi = dLift_r_psi * sin(phi) + dDrag_r_psi * cos(phi);

                // thrust added up over one revolution
                dThrust_r_psi_add += dThrust_r_psi;
                dQ_r_psi_add += dQ_r_psi;

            }
            // average thrust of rotor blade section over one revolution
            dThrust_r_average = dThrust_r_psi_add / 360;
            dQ_r_average = dQ_r_psi_add / 360;
            dThrust_r_psi_add = 0;
            dQ_r_psi_add = 0;

            thrust_calculated += dThrust_r_average * myPropPt->NumberOfBlades;
            Moment += dQ_r_average * myPropPt->NumberOfBlades * myPropPt->Radius.at(i);
        }
        eps = abs((thrust_calculated - thrust) / thrust_calculated);
        if (thrust_calculated > thrust && eps > 0.1)
        {
            omega = omega - eps;
        }
        else if (thrust_calculated < thrust && eps > 0.1)
        {
            omega = omega + eps;
        }
        cout << omega << endl;
    }

    omega_start = omega;
    Power = omega * Moment;

    vector<double> calcBETResults (2,0.0);
    calcBETResults.at(0) = omega_start;
    calcBETResults.at(1) = Power;

    return calcBETResults;
}

BladeElementTheory::~BladeElementTheory()
{
    //dtor
}
