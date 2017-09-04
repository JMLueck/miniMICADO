#include "MassEstimation.h"

MassEstimation::MassEstimation(node& configXML, Fuselage &myFuselage, Wing &myWing)
    :
    configXML(configXML),
    myFuselagePt(&myFuselage),
    myWingPt(&myWing)
{
    //ctor
}

void MassEstimation::doMassEstimation(double P_max, double MissionEnergy)
{
    estimateFuselageMass();
    estimateRotorMass();
    estimateMotorMass(P_max);
    estimateBatteryMass(P_max, MissionEnergy);
    estimateWireMass(P_max);
    estimateAvionicsMass();
    estimateLandingGearMass();

    VehicleMass = configXML["PayloadMass"] + FuselageMass + RotorMass + MotorMass + BatteryMass + WireMass + AvionicsMass + LandingGearMass;

    if (configXML["AddWing"] == 1)
    {
        estimateWingMass();
        VehicleMass =  VehicleMass + WingMass;
    }
    if (configXML["Tiltrotor"] == 1)
    {
        estimateServomotorMass();
        VehicleMass =  VehicleMass + ServomotorMass;
    }
}

void MassEstimation::estimateFuselageMass()
{
    FuselageMass = myFuselagePt->FuselageWettedArea * configXML["FuselageSkinThickness"]/1000 * configXML["FuselageSkinDensity"];
}

void MassEstimation::estimateRotorMass()
{
    RotorMass = configXML["NumberOfRotors"] * (0.5465 * pow(configXML["RotorDiameter"],3) - 0.1026 * pow(configXML["RotorDiameter"],2) + 0.01679 * configXML["RotorDiameter"] + 0.0009706);

    // Account for Tailrotormass (size is set to one quarter of mainrotor)
    if (configXML["NumberOfRotors"] == 1)
    {
        RotorMass = RotorMass + (0.5465 * pow(configXML["RotorDiameter"]/4,3) - 0.1026 * pow(configXML["RotorDiameter"]/4,2) + 0.01679 * configXML["RotorDiameter"]/4 + 0.0009706);
    }
}

void MassEstimation::estimateMotorMass(double P_max)
{
    MotorMass = configXML["NumberOfRotors"] * (0.02573 + 0.0002075 * P_max);

    // Account for Tailrotor-Motormass with P_max set to 10% of total power (see Mission Analysis)
    if (configXML["NumberOfRotors"] == 1)
    {
        MotorMass = MotorMass + (0.02573 + 0.0002075 * 0.1*P_max);
    }
}

void MassEstimation::estimateBatteryMass(double P_max, double MissionEnergy)
{
    BatteryMass = std::max(P_max/1000/configXML["PowerDensity"], MissionEnergy/3600/configXML["EnergyDensity"]);
}

void MassEstimation::estimateWingMass()
{
    for (int i=1; i<myWingPt->x_coord.size()-1; i++)
    {
        airfoilDist += sqrt(pow((myWingPt->x_coord.at(i+1) - myWingPt->x_coord.at(i))*configXML["WingMAC"],2) + pow((myWingPt->z_coord.at(i+1) - myWingPt->z_coord.at(i))*configXML["WingMAC"],2));
    }
    WingSkinMass = airfoilDist * configXML["Wingspan"] * configXML["WingSkinThickness"]/1000 * configXML["WingSkinDensity"];

    double BendingMoment_max = (VehicleMass*9.81*configXML["Wingspan"])/(PI);
    double BeamHeigth = myWingPt->TtoC *  configXML["WingMAC"];
    WingBeamMass = 11 * configXML["WingBeamDensity"] * pow(BeamHeigth,2) * (configXML["Wingspan"]-myFuselagePt->FuselageWidth) / 32;

    WingMass = WingSkinMass + WingBeamMass;
}

void MassEstimation::estimateAvionicsMass()
{
    double m_ESC = configXML["NumberOfRotors"] * 0.0137;
    double m_FC = 0.0085;
    double m_Rec = 0.008;
    double m_IMU = 3 * 0.0136; // 3 IMUs for rotational speeds around 3 axis
    double m_GPS = 0.0469;
    double m_Radar = 4 * 0.0083;

    AvionicsMass = m_ESC + m_FC + m_Rec + m_IMU + m_GPS + m_Radar;
}

void MassEstimation::estimateLandingGearMass()
{
    LandingGearMass = 0.17265;
}

void MassEstimation::estimateWireMass(double P_max) // from Vahana Code
{
    double PowerWireDensity = 0.00001; // [kg/m/W]
    double SensorWireDensity = 0.0046; // [kg/m]
    double WiresPerBundle = 6;
    double PowerWireLength = configXML["NumberOfRotors"] * (myFuselagePt->FuselageLength/2 + myFuselagePt->FuselageWidth/2) + configXML["Wingspan"]/2;
    double SensorWireLength = PowerWireLength + 10 * myFuselagePt->FuselageLength + 4 * configXML["Wingspan"];
    double PowerWireMass = PowerWireDensity * P_max * PowerWireLength;
    double SensorWireMass = SensorWireDensity * WiresPerBundle * SensorWireLength;

    WireMass = PowerWireMass + SensorWireMass;
}

void MassEstimation::estimateServomotorMass()
{
    double Momentum = (this->RotorMass + this->MotorMass) * 0.05;
    ServomotorMass = 0.04473 * Momentum + 0.01495;
}

void MassEstimation::writeResults(int LoopNumber)
{
    const string MEResult_file = "Results\\MassEstimationResults_Loop" + num2Str(LoopNumber) + ".csv";
    ofstream MEResults;
    MEResults.open(MEResult_file.c_str());
    MEResults << "Payload;Fuselage;Rotor;Motor;Battery;Wing;Wires;Avionics;Landing Gear;Vehicle"<< endl;
    MEResults <<configXML["PayloadMass"] << ";";
    MEResults << FuselageMass << ";";
    MEResults << RotorMass << ";";
    MEResults << MotorMass << ";";
    MEResults << BatteryMass << ";";
    MEResults << WingMass << ";";
    MEResults << WireMass << ";";
    MEResults << AvionicsMass << ";";
    MEResults << LandingGearMass << ";";
    MEResults << VehicleMass << ";";
    MEResults << endl;
    MEResults.close();
}

MassEstimation::~MassEstimation()
{
    //dtor
}
