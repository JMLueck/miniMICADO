#ifndef MASSESTIMATION_H
#define MASSESTIMATION_H

#define PI 3.14159265358979323846

#include "node.h"
#include "math.h"
#include "Wing.h"

class MassEstimation
{
    public:
        MassEstimation(node& configXML, Fuselage &myFuselage, Wing &myWing);
        virtual ~MassEstimation();

        double airfoilDist;
        double WingSkinMass;
        double WingBeamMass;

        double VehicleMass;
        double FuselageMass;
        double RotorMass;
        double MotorMass;
        double BatteryMass;
        double WingMass;
        double WireMass;
        double AvionicsMass;
        double LandingGearMass;
        double ServomotorMass;


        void doMassEstimation(double P_max, double MissionEnergy);
        void estimateFuselageMass();
        void estimateRotorMass();
        void estimateMotorMass(double P_max);
        void estimateBatteryMass(double P_max, double MissionEnergy);
        void estimateWingMass();
        void estimateWireMass(double P_max);
        void estimateAvionicsMass();
        void estimateLandingGearMass();
        void estimateServomotorMass();

        void writeResults(int LoopNumber);

    protected:

    private:
        node& configXML;
        Wing *myWingPt;
        Fuselage *myFuselagePt;
};

#endif // MASSESTIMATION_H
