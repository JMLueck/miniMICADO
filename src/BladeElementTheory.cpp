#include "BladeElementTheory.h"

BladeElementTheory::BladeElementTheory(node& configXML, Propeller &myProp)
    :
    configXML(configXML),
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
                Ma = v_tot / myatmo.getSpeedOfSound(altitude);

                // Tip Loss
                double f = (configXML["NumberOfBlades"] * (1-myPropPt->Radius.at(i)))/(2*sin(phi));
                double F = 2 * acos(pow(E,-f)) / PI;

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
    }

    omega_start = omega;
    Power = omega * Moment;

    /** Multikopter Configuration - Interference **/
    if (configXML["NumberOfRotors"] > 1)
    {
        double k_mu;
        double mu = v_tot/omega*myPropPt->Radius.back();
        double gamma_0 = atan(configXML["VerticalRotorSeperation"]/configXML["HorizontalRotorSeperation"]);
        double h_rr = sin(alpha + delta + phi + gamma_0) * sqrt(pow(configXML["VerticalRotorSeperation"],2) + pow(configXML["HorizontalRotorSeperation"],2));
        if (mu<0.1)
        {k_mu = 0.41 + 0.59*sin(5*PI*mu);}
        else{k_mu = 1;}
        double m_tilde = k_mu * (2/PI) * acos((h_rr/configXML["HorizontalRotorSeperation"]) - (h_rr/configXML["HorizontalRotorSeperation"])*sqrt(1-pow((h_rr/configXML["HorizontalRotorSeperation"]),2)));

        if(isnan(m_tilde))
        {
            cout << k_mu << endl;
            cout << gamma_0 << endl;
            cout << h_rr << endl;
            getchar();
        }

        if (configXML["NumberOfRotors"] == 2 || configXML["NumberOfRotors"] == 4)
        {Power = Power * configXML["NumberOfRotors"] * (1+m_tilde);}
        else if (configXML["NumberOfRotors"] == 6)
        {Power = Power * configXML["NumberOfRotors"] * (6+8*m_tilde);}
        else if (configXML["NumberOfRotors"] == 8)
        {Power = Power * configXML["NumberOfRotors"] * (8+12*m_tilde);}
        else{cout << "Multicopter-Configuration with " << configXML["NumberOfRotors"] << " Rotors is not supported";}
    }

    vector<double> calcBETResults (2,0.0);
    calcBETResults.at(0) = omega_start;
    calcBETResults.at(1) = Power;

    return calcBETResults;
}

BladeElementTheory::~BladeElementTheory()
{
    //dtor
}
