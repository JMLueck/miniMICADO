#include "MissionAnalysis.h"
#include "functions.h"
#include "Aerodynamics.h"
#include "BladeElementTheory.h"


/** Analysis of Power and Energy Consumption **/
/** Possibly implementation of 6-DOF equations here **/

MissionAnalysis::MissionAnalysis(node& configXML, MassEstimation &myMassEstimation, Wing &myWing, Aerodynamics &myAero)
    :
    configXML(configXML),
    myMassEstimationPt(&myMassEstimation),
    myWingPt(&myWing),
    myAeroPt(&myAero),
    Segments(configXML["NumberOfSegments"], vector<double>(12,0.0)),
    SegmentsTime(configXML["NumberOfSegments"], 0.0),
    MissionTime(0.0),
    MissionResults(6,vector<double>(0,0.0))
    //Waypoints(7, vector<double>((int(MissionTime)*configXML["IterationTimeFactor"]+1),0.0))
{
    calcMissionSegments();
    calcMissionTime();
    calcSegmentsTimeAndWay();
    Waypoints = calcMissionWaypoints();
}

void MissionAnalysis::calcMissionSegments()
{
    for (int i = 1; i <= configXML["NumberOfSegments"]; i++ )
    {
        SegmentMap.insert(make_pair(("Segment" + num2Str(i)), Segment(configXML,i)));
    }
}

void MissionAnalysis::calcMissionTime()
{
    for (int i = 1; i <= SegmentMap.size(); i++)
    {
        MissionTime += SegmentMap.at("Segment" + num2Str(i)).Time;
        SegmentsTime.at(i-1) = MissionTime;
    }
}


void MissionAnalysis::calcSegmentsTimeAndWay()
{
    for (int i = 0; i < configXML["NumberOfSegments"]; i++)
    {
        if (SegmentMap.at("Segment" + num2Str(i+1)).Type == "Vertical")
        {
            Segments.at(i).at(0) = 0;
            Segments.at(i).at(1) = SegmentMap.at("Segment" + num2Str(i+1)).Time_PhaseOne;
            Segments.at(i).at(2) = 0;
            Segments.at(i).at(3) = SegmentMap.at("Segment" + num2Str(i+1)).VerticalWay_PhaseOne;
            Segments.at(i).at(4) = 0;
            Segments.at(i).at(5) = SegmentMap.at("Segment" + num2Str(i+1)).Time_PhaseTwo;
            Segments.at(i).at(6) = 0;
            Segments.at(i).at(7) = SegmentMap.at("Segment" + num2Str(i+1)).VerticalWay_PhaseTwo;
            Segments.at(i).at(8) = 0;
            Segments.at(i).at(9) = SegmentMap.at("Segment" + num2Str(i+1)).Time_PhaseThree;
            Segments.at(i).at(10) = 0;
            Segments.at(i).at(11) = SegmentMap.at("Segment" + num2Str(i+1)).VerticalWay_PhaseThree;
        }
        else if (SegmentMap.at("Segment" + num2Str(i+1)).Type == "Aslope")
        {
            Segments.at(i).at(0) = SegmentMap.at("Segment" + num2Str(i+1)).Time_PhaseOne_Hor;
            Segments.at(i).at(1) = SegmentMap.at("Segment" + num2Str(i+1)).Time_PhaseOne_Vert;
            Segments.at(i).at(2) = SegmentMap.at("Segment" + num2Str(i+1)).HorizontalWay_PhaseOne;
            Segments.at(i).at(3) = SegmentMap.at("Segment" + num2Str(i+1)).VerticalWay_PhaseOne;
            Segments.at(i).at(4) = SegmentMap.at("Segment" + num2Str(i+1)).Time_PhaseTwo_Hor;
            Segments.at(i).at(5) = SegmentMap.at("Segment" + num2Str(i+1)).Time_PhaseTwo_Vert;
            Segments.at(i).at(6) = SegmentMap.at("Segment" + num2Str(i+1)).HorizontalWay_PhaseTwo;
            Segments.at(i).at(7) = SegmentMap.at("Segment" + num2Str(i+1)).VerticalWay_PhaseTwo;
            Segments.at(i).at(8) = SegmentMap.at("Segment" + num2Str(i+1)).Time_PhaseThree_Hor;
            Segments.at(i).at(9) = SegmentMap.at("Segment" + num2Str(i+1)).Time_PhaseThree_Vert;
            Segments.at(i).at(10) = SegmentMap.at("Segment" + num2Str(i+1)).HorizontalWay_PhaseThree;
            Segments.at(i).at(11) = SegmentMap.at("Segment" + num2Str(i+1)).VerticalWay_PhaseThree;

        }
        else if (SegmentMap.at("Segment" + num2Str(i+1)).Type == "Hover")
        {
            Segments.at(i).at(0) = 0;
            Segments.at(i).at(1) = 0;
            Segments.at(i).at(2) = 0;
            Segments.at(i).at(3) = 0;
            Segments.at(i).at(4) = 0;
            Segments.at(i).at(5) = 0;
            Segments.at(i).at(6) = 0;
            Segments.at(i).at(7) = 0;
            Segments.at(i).at(8) = 0;
            Segments.at(i).at(9) = 0;
            Segments.at(i).at(10) = 0;
            Segments.at(i).at(11) = 0;

        }
        else if (SegmentMap.at("Segment" + num2Str(i+1)).Type == "Horizontal")
        {
            Segments.at(i).at(0) = SegmentMap.at("Segment" + num2Str(i+1)).Time_PhaseOne;
            Segments.at(i).at(1) = 0;
            Segments.at(i).at(2) = SegmentMap.at("Segment" + num2Str(i+1)).HorizontalWay_PhaseOne;
            Segments.at(i).at(3) = 0;
            Segments.at(i).at(4) = SegmentMap.at("Segment" + num2Str(i+1)).Time_PhaseTwo;
            Segments.at(i).at(5) = 0;
            Segments.at(i).at(6) = SegmentMap.at("Segment" + num2Str(i+1)).HorizontalWay_PhaseTwo;
            Segments.at(i).at(7) = 0;
            Segments.at(i).at(8) = SegmentMap.at("Segment" + num2Str(i+1)).Time_PhaseThree;
            Segments.at(i).at(9) = 0;
            Segments.at(i).at(10) = SegmentMap.at("Segment" + num2Str(i+1)).HorizontalWay_PhaseThree;
            Segments.at(i).at(11) = 0;
        }

    }
}


int MissionAnalysis::getSegmentNumber(double currentTime)
{
    for (int j = 0; j<SegmentsTime.size(); j++)
    {
        if (currentTime <= SegmentsTime.at(j))
        {
            return j;
        }
    }
}

int MissionAnalysis::getVerticalPhaseNumber(double currentTime)
{
    int j = getSegmentNumber(currentTime);

    if (j > 0)
    {
        if (SegmentMap.at("Segment" + num2Str(j+1)).Type == "Vertical" || SegmentMap.at("Segment" + num2Str(j+1)).Type == "Aslope")
        {
            if (currentTime >= (SegmentsTime.at(j-1) + Segments.at(j).at(1)) && currentTime <= (SegmentsTime.at(j) - Segments.at(j).at(9)))
            {
                return 5;   // middle phase
            }
            if (currentTime >= SegmentsTime.at(j-1) && currentTime <= (SegmentsTime.at(j) - Segments.at(j).at(9) - Segments.at(j).at(5)))
            {
                return 1;   // first phase
            }
            if (currentTime >= (SegmentsTime.at(j-1) + Segments.at(j).at(1) + Segments.at(j).at(5)) && currentTime <= (SegmentsTime.at(j)))
            {
                return 9; // last phase
            }
        }
        else
        {
            return 12;
        }
    }
    else
    {
        if (currentTime >= (0 + Segments.at(j).at(1)) && currentTime <= (SegmentsTime.at(j) - Segments.at(j).at(9)))
        {
            return 5;
        }
        else if (currentTime >= 0  && currentTime <= (SegmentsTime.at(j) - Segments.at(j).at(9) - Segments.at(j).at(5)))
        {
            return 1;
        }
        else if (currentTime >= (0 + Segments.at(j).at(1) + Segments.at(j).at(5))  && currentTime <= SegmentsTime.at(j))
        {
            return 9;
        }
    }
}

int MissionAnalysis::getHorizontalPhaseNumber(double currentTime)
{
    int j = getSegmentNumber(currentTime);

    if (j > 0)
    {
        if (SegmentMap.at("Segment" + num2Str(j+1)).Type == "Horizontal" || SegmentMap.at("Segment" + num2Str(j+1)).Type == "Aslope")
        {
            if (currentTime >= (SegmentsTime.at(j-1) + Segments.at(j).at(0)) && currentTime <= (SegmentsTime.at(j) - Segments.at(j).at(8)))
            {
                return 4;   // middle phase
            }
            if (currentTime >= SegmentsTime.at(j-1) && currentTime <= (SegmentsTime.at(j) - Segments.at(j).at(8) - Segments.at(j).at(4)))
            {
                return 0;   // first phase
            }
            if (currentTime >= (SegmentsTime.at(j-1) + Segments.at(j).at(0) + Segments.at(j).at(4)) && currentTime <= (SegmentsTime.at(j)))
            {
                return 8; // last phase
            }
        }
        else
        {
            return 12;
        }
    }
    else
    {
        if (SegmentMap.at("Segment" + num2Str(j+1)).Type == "Aslope")
        {
            if (currentTime >= (0 + Segments.at(j).at(0)) && currentTime <= (SegmentsTime.at(j) - Segments.at(j).at(8)))
            {
                return 4;
            }
            if (currentTime >= 0  && currentTime <= (SegmentsTime.at(j) - Segments.at(j).at(8) - Segments.at(j).at(4)))
            {
                return 0;
            }
            if (currentTime >= (0 + Segments.at(j).at(0) + Segments.at(j).at(4))  && currentTime <= SegmentsTime.at(j))
            {
                return 8;
            }
        }
        else
        {
            return 12;
        }
    }
}

vector<vector<double>> MissionAnalysis::calcMissionWaypoints()
{
    vector<vector<double>> Waypoints (7, vector<double>((int(MissionTime)*configXML["IterationTimeFactor"]+1),0.0));
    LastSegment = false;
    double PreviousVerticalSpeed = 0;
    double PreviousHorizontalSpeed = 0;
    double PreviousVerticalPosition = 0;
    double PreviousHorizontalPosition = 0;
    MissionTimeStep = MissionTime / (configXML["IterationTimeFactor"]*int(MissionTime)); // close to 1sec and evenly distributed

    /** initialize step one **/
    Waypoints.at(0).at(0) = 0; // time
    Waypoints.at(1).at(0) = configXML["Segment@1/HorizontalAcceleration"]; // horizontal acceleration
    Waypoints.at(2).at(0) = configXML["Segment@1/VerticalAcceleration"]; // vertical acceleration
    Waypoints.at(3).at(0) = 0; // horizontal velocity
    Waypoints.at(4).at(0) = 0; // vertical velocity
    Waypoints.at(5).at(0) = 0; // horizontal position
    Waypoints.at(6).at(0) = 0; // vertical position

    for (int i = 1; i <= (int(MissionTime)*configXML["IterationTimeFactor"]); i++)
    {
        Waypoints.at(0).at(i) = MissionTimeStep * i;

        int j = getSegmentNumber(Waypoints.at(0).at(i));
        int v = getVerticalPhaseNumber(Waypoints.at(0).at(i));
        int w = getHorizontalPhaseNumber(Waypoints.at(0).at(i));

        if ((j+1) == configXML["NumberOfSegments"])
        {
            LastSegment = true;
        }

        if (SegmentMap.at("Segment" + num2Str(j+1)).Type == "Vertical")
        {
            Waypoints.at(1).at(i) = 0;
            Waypoints.at(3).at(i) = 0;
            for (int z = 0; z<j; z++)
            {
                Waypoints.at(5).at(i) += SegmentMap.at("Segment" + num2Str(z+1)).HorizontalDistance;
            }

            if (configXML["Segment@" + num2Str(j+1) + "/VerticalSpeed"] > 0)
            {
                if (v == 1)
                {
                    Waypoints.at(2).at(i) = SegmentMap.at("Segment" + num2Str(j+1)).VerticalAcceleration;
                    Waypoints.at(4).at(i) = Waypoints.at(2).at(i) * (Waypoints.at(0).at(i)-Waypoints.at(0).at(i-1)) + PreviousVerticalSpeed;
                    Waypoints.at(6).at(i) = 0.5 * Waypoints.at(2).at(i) * pow((Waypoints.at(0).at(i)-Waypoints.at(0).at(i-1)),2) + Waypoints.at(4).at(i-1) * (Waypoints.at(0).at(i)-Waypoints.at(0).at(i-1)) + PreviousVerticalPosition;
                }
                else if (v == 5)
                {
                    Waypoints.at(2).at(i) = 0;
                    Waypoints.at(4).at(i) = SegmentMap.at("Segment" + num2Str(j+1)).VerticalVelocity;
                    Waypoints.at(6).at(i) = Waypoints.at(4).at(i) * (Waypoints.at(0).at(i)-Waypoints.at(0).at(i-1)) + PreviousVerticalPosition;
                }
                else if (v == 9)
                {
                    if(configXML["Segment@" + num2Str(j+2) + "/VerticalSpeed"] > configXML["Segment@" + num2Str(j+1) + "/VerticalSpeed"])
                    {
                        Waypoints.at(2).at(i) = SegmentMap.at("Segment" + num2Str(j+1)).VerticalAcceleration;
                        Waypoints.at(4).at(i) = Waypoints.at(2).at(i) * (Waypoints.at(0).at(i)-Waypoints.at(0).at(i-1)) + PreviousVerticalSpeed;
                        Waypoints.at(6).at(i) = 0.5 * Waypoints.at(2).at(i) * pow((Waypoints.at(0).at(i)-Waypoints.at(0).at(i-1)),2) + Waypoints.at(4).at(i-1) * (Waypoints.at(0).at(i)-Waypoints.at(0).at(i-1)) + PreviousVerticalPosition;
                    }
                    else
                    {
                        Waypoints.at(2).at(i) = -SegmentMap.at("Segment" + num2Str(j+1)).VerticalAcceleration;
                        Waypoints.at(4).at(i) = Waypoints.at(2).at(i) * (Waypoints.at(0).at(i)-Waypoints.at(0).at(i-1)) + PreviousVerticalSpeed;
                        Waypoints.at(6).at(i) = 0.5 * Waypoints.at(2).at(i) * pow((Waypoints.at(0).at(i)-Waypoints.at(0).at(i-1)),2) + Waypoints.at(4).at(i-1) * (Waypoints.at(0).at(i)-Waypoints.at(0).at(i-1)) + PreviousVerticalPosition;
                    }
                }
            }
            else
            {
                if (v == 1)
                {
                    Waypoints.at(2).at(i) = -SegmentMap.at("Segment" + num2Str(j+1)).VerticalAcceleration;
                    Waypoints.at(4).at(i) = Waypoints.at(2).at(i) * (Waypoints.at(0).at(i)-Waypoints.at(0).at(i-1)) + PreviousVerticalSpeed;
                    Waypoints.at(6).at(i) = 0.5 * Waypoints.at(2).at(i) * pow((Waypoints.at(0).at(i)-Waypoints.at(0).at(i-1)),2) + Waypoints.at(4).at(i-1) * (Waypoints.at(0).at(i)-Waypoints.at(0).at(i-1)) + PreviousVerticalPosition;
                }
                else if (v == 5)
                {
                    Waypoints.at(2).at(i) = 0;
                    Waypoints.at(4).at(i) = SegmentMap.at("Segment" + num2Str(j+1)).VerticalVelocity;
                    Waypoints.at(6).at(i) = Waypoints.at(4).at(i) * (Waypoints.at(0).at(i)-Waypoints.at(0).at(i-1)) + PreviousVerticalPosition;
                }
                else if (v == 9)
                {
                    if (LastSegment != true)
                    {
                        if(configXML["Segment@" + num2Str(j+2) + "/VerticalSpeed"] < configXML["Segment@" + num2Str(j+1) + "/VerticalSpeed"])
                        {
                            Waypoints.at(2).at(i) = -SegmentMap.at("Segment" + num2Str(j+1)).VerticalAcceleration;
                            Waypoints.at(4).at(i) = Waypoints.at(2).at(i) * (Waypoints.at(0).at(i)-Waypoints.at(0).at(i-1)) + PreviousVerticalSpeed;
                            Waypoints.at(6).at(i) = 0.5 * Waypoints.at(2).at(i) * pow((Waypoints.at(0).at(i)-Waypoints.at(0).at(i-1)),2) + Waypoints.at(4).at(i-1) * (Waypoints.at(0).at(i)-Waypoints.at(0).at(i-1)) + PreviousVerticalPosition;
                        }
                        else
                        {
                            Waypoints.at(2).at(i) = SegmentMap.at("Segment" + num2Str(j+1)).VerticalAcceleration;
                            Waypoints.at(4).at(i) = Waypoints.at(2).at(i) * (Waypoints.at(0).at(i)-Waypoints.at(0).at(i-1)) + PreviousVerticalSpeed;
                            Waypoints.at(6).at(i) = 0.5 * Waypoints.at(2).at(i) * pow((Waypoints.at(0).at(i)-Waypoints.at(0).at(i-1)),2) + Waypoints.at(4).at(i-1) * (Waypoints.at(0).at(i)-Waypoints.at(0).at(i-1)) + PreviousVerticalPosition;
                        }
                    }
                    else
                    {
                        Waypoints.at(2).at(i) = SegmentMap.at("Segment" + num2Str(j+1)).VerticalAcceleration;
                        Waypoints.at(4).at(i) = Waypoints.at(2).at(i) * (Waypoints.at(0).at(i)-Waypoints.at(0).at(i-1)) + PreviousVerticalSpeed;
                        Waypoints.at(6).at(i) = 0.5 * Waypoints.at(2).at(i) * pow((Waypoints.at(0).at(i)-Waypoints.at(0).at(i-1)),2) + Waypoints.at(4).at(i-1) * (Waypoints.at(0).at(i)-Waypoints.at(0).at(i-1)) + PreviousVerticalPosition;
                    }
                }
            }
        }

        else if (SegmentMap.at("Segment" + num2Str(j+1)).Type == "Horizontal")
        {
            Waypoints.at(2).at(i) = 0;
            Waypoints.at(4).at(i) = 0;
            Waypoints.at(6).at(i) = 0; //initialisieren
            for (int z = 0; z<j; z++)
            {
                if(SegmentMap.at("Segment" + num2Str(z+1)).VerticalVelocity > 0)
                {
                    Waypoints.at(6).at(i) += SegmentMap.at("Segment" + num2Str(z+1)).VerticalDistance;
                }
                else
                {
                    Waypoints.at(6).at(i) -= SegmentMap.at("Segment" + num2Str(z+1)).VerticalDistance;
                }
            }

            if(w == 0)
            {
                Waypoints.at(1).at(i) = SegmentMap.at("Segment" + num2Str(j+1)).HorizontalAcceleration;
                Waypoints.at(3).at(i) = Waypoints.at(1).at(i) * (Waypoints.at(0).at(i)-Waypoints.at(0).at(i-1)) + PreviousHorizontalSpeed;
                Waypoints.at(5).at(i) = 0.5 * Waypoints.at(1).at(i) * pow((Waypoints.at(0).at(i)-Waypoints.at(0).at(i-1)),2) + Waypoints.at(3).at(i-1) * (Waypoints.at(0).at(i)-Waypoints.at(0).at(i-1)) + PreviousHorizontalPosition;
            }
            else if (w == 4)
            {
                Waypoints.at(1).at(i) = 0;
                Waypoints.at(3).at(i) = SegmentMap.at("Segment" + num2Str(j+1)).HorizontalVelocity;
                Waypoints.at(5).at(i) = Waypoints.at(3).at(i) * (Waypoints.at(0).at(i)-Waypoints.at(0).at(i-1)) + PreviousHorizontalPosition;
            }
            else if (w == 8)
            {
                if(configXML["Segment@" + num2Str(j+2) + "/HorizontalSpeed"] > configXML["Segment@" + num2Str(j+1) + "/HorizontalSpeed"])
                {
                    Waypoints.at(1).at(i) = SegmentMap.at("Segment" + num2Str(j+1)).HorizontalAcceleration;
                    Waypoints.at(3).at(i) = Waypoints.at(1).at(i) * (Waypoints.at(0).at(i)-Waypoints.at(0).at(i-1)) + PreviousHorizontalSpeed;
                    Waypoints.at(5).at(i) = 0.5 * Waypoints.at(1).at(i) * pow((Waypoints.at(0).at(i)-Waypoints.at(0).at(i-1)),2) + Waypoints.at(3).at(i-1) * (Waypoints.at(0).at(i)-Waypoints.at(0).at(i-1)) + PreviousHorizontalPosition;
                }
                else
                {
                    Waypoints.at(1).at(i) = -SegmentMap.at("Segment" + num2Str(j+1)).HorizontalAcceleration;
                    Waypoints.at(3).at(i) = Waypoints.at(1).at(i) * (Waypoints.at(0).at(i)-Waypoints.at(0).at(i-1)) + PreviousHorizontalSpeed;
                    Waypoints.at(5).at(i) = 0.5 * Waypoints.at(1).at(i) * pow((Waypoints.at(0).at(i)-Waypoints.at(0).at(i-1)),2) + Waypoints.at(3).at(i-1) * (Waypoints.at(0).at(i)-Waypoints.at(0).at(i-1)) + PreviousHorizontalPosition;
                }
            }

        }

        else if (SegmentMap.at("Segment" + num2Str(j+1)).Type == "Hover")
        {
            Waypoints.at(1).at(i) = 0;
            Waypoints.at(2).at(i) = 0;
            Waypoints.at(3).at(i) = 0;
            Waypoints.at(4).at(i) = 0;
            Waypoints.at(5).at(i) = PreviousHorizontalPosition;
            Waypoints.at(6).at(i) = PreviousVerticalPosition;
        }

        else if (SegmentMap.at("Segment" + num2Str(j+1)).Type == "Aslope")
        {
            if (configXML["Segment@" + num2Str(j+1) + "/VerticalSpeed"] > 0)
            {
                if (v == 1)
                {
                    if(SegmentMap.at("Segment" + num2Str(j+1)).ExceptionAslopeVert == true)
                    {
                        Waypoints.at(2).at(i) = 0;
                        Waypoints.at(4).at(i) = 0;
                        Waypoints.at(6).at(i) = PreviousVerticalPosition;
                    }
                    else
                    {
                        Waypoints.at(2).at(i) = SegmentMap.at("Segment" + num2Str(j+1)).VerticalAcceleration;
                        Waypoints.at(4).at(i) = Waypoints.at(2).at(i) * (Waypoints.at(0).at(i)-Waypoints.at(0).at(i-1)) + PreviousVerticalSpeed;
                        Waypoints.at(6).at(i) = 0.5 * Waypoints.at(2).at(i) * pow((Waypoints.at(0).at(i)-Waypoints.at(0).at(i-1)),2) + Waypoints.at(4).at(i-1) * (Waypoints.at(0).at(i)-Waypoints.at(0).at(i-1)) + PreviousVerticalPosition;
                    }
                }
                else if (v == 5)
                {
                    Waypoints.at(2).at(i) = 0;
                    Waypoints.at(4).at(i) = SegmentMap.at("Segment" + num2Str(j+1)).VerticalVelocity;
                    Waypoints.at(6).at(i) = Waypoints.at(4).at(i) * (Waypoints.at(0).at(i)-Waypoints.at(0).at(i-1)) + PreviousVerticalPosition;
                }
                else if (v == 9)
                {
                    if(SegmentMap.at("Segment" + num2Str(j+1)).ExceptionAslopeVert == true)
                    {
                        Waypoints.at(2).at(i) = SegmentMap.at("Segment" + num2Str(j+1)).VerticalAcceleration;
                        Waypoints.at(4).at(i) = Waypoints.at(2).at(i) * (Waypoints.at(0).at(i)-Waypoints.at(0).at(i-1)) + PreviousVerticalSpeed;
                        Waypoints.at(6).at(i) = 0.5 * Waypoints.at(2).at(i) * pow((Waypoints.at(0).at(i)-Waypoints.at(0).at(i-1)),2) + Waypoints.at(4).at(i-1) * (Waypoints.at(0).at(i)-Waypoints.at(0).at(i-1)) + PreviousVerticalPosition;
                    }
                    else
                    {
                        if(configXML["Segment@" + num2Str(j+2) + "/VerticalSpeed"] > configXML["Segment@" + num2Str(j+1) + "/VerticalSpeed"])
                        {
                            Waypoints.at(2).at(i) = SegmentMap.at("Segment" + num2Str(j+1)).VerticalAcceleration;
                            Waypoints.at(4).at(i) = Waypoints.at(2).at(i) * (Waypoints.at(0).at(i)-Waypoints.at(0).at(i-1)) + PreviousVerticalSpeed;
                            Waypoints.at(6).at(i) = 0.5 * Waypoints.at(2).at(i) * pow((Waypoints.at(0).at(i)-Waypoints.at(0).at(i-1)),2) + Waypoints.at(4).at(i-1) * (Waypoints.at(0).at(i)-Waypoints.at(0).at(i-1)) + PreviousVerticalPosition;
                        }
                        else
                        {
                            if(SegmentMap.at("Segment" + num2Str(j+1)).Time_Vert == SegmentMap.at("Segment" + num2Str(j+1)).Time_PhaseThree_Vert)
                            {
                                Waypoints.at(2).at(i) = -SegmentMap.at("Segment" + num2Str(j+1)).VerticalAcceleration;
                                Waypoints.at(4).at(i) = Waypoints.at(2).at(i) * (Waypoints.at(0).at(i)-Waypoints.at(0).at(i-1)) + PreviousVerticalSpeed;
                                Waypoints.at(6).at(i) = 0.5 * Waypoints.at(2).at(i) * pow((Waypoints.at(0).at(i)-Waypoints.at(0).at(i-1)),2) + Waypoints.at(4).at(i-1) * (Waypoints.at(0).at(i)-Waypoints.at(0).at(i-1)) + PreviousVerticalPosition;
                                if(Waypoints.at(4).at(i) <= 0)
                                {
                                    Waypoints.at(2).at(i) = 0;
                                    Waypoints.at(4).at(i) = 0;
                                    Waypoints.at(6).at(i) = PreviousVerticalPosition;
                                }
                            }
                            else
                            {
                                Waypoints.at(2).at(i) = -SegmentMap.at("Segment" + num2Str(j+1)).VerticalAcceleration;
                                Waypoints.at(4).at(i) = Waypoints.at(2).at(i) * (Waypoints.at(0).at(i)-Waypoints.at(0).at(i-1)) + PreviousVerticalSpeed;
                                Waypoints.at(6).at(i) = 0.5 * Waypoints.at(2).at(i) * pow((Waypoints.at(0).at(i)-Waypoints.at(0).at(i-1)),2) + Waypoints.at(4).at(i-1) * (Waypoints.at(0).at(i)-Waypoints.at(0).at(i-1)) + PreviousVerticalPosition;
                            }
                        }
                    }
                }

                if(w == 0)
                {
                    if(SegmentMap.at("Segment" + num2Str(j+1)).ExceptionAslopeHor == true)
                    {
                        Waypoints.at(1).at(i) = 0;
                        Waypoints.at(3).at(i) = 0;
                        Waypoints.at(5).at(i) = PreviousHorizontalPosition;
                    }
                    else
                    {
                        Waypoints.at(1).at(i) = SegmentMap.at("Segment" + num2Str(j+1)).HorizontalAcceleration;
                        Waypoints.at(3).at(i) = Waypoints.at(1).at(i) * (Waypoints.at(0).at(i)-Waypoints.at(0).at(i-1)) + PreviousHorizontalSpeed;
                        Waypoints.at(5).at(i) = 0.5 * Waypoints.at(1).at(i) * pow((Waypoints.at(0).at(i)-Waypoints.at(0).at(i-1)),2) + Waypoints.at(3).at(i-1) * (Waypoints.at(0).at(i)-Waypoints.at(0).at(i-1)) + PreviousHorizontalPosition;
                    }
                }
                else if (w == 4)
                {
                    Waypoints.at(1).at(i) = 0;
                    Waypoints.at(3).at(i) = SegmentMap.at("Segment" + num2Str(j+1)).HorizontalVelocity;
                    Waypoints.at(5).at(i) = Waypoints.at(3).at(i) * (Waypoints.at(0).at(i)-Waypoints.at(0).at(i-1)) + PreviousHorizontalPosition;
                }
                else if (w == 8)
                {
                    if(SegmentMap.at("Segment" + num2Str(j+1)).ExceptionAslopeHor == true)
                    {
                        Waypoints.at(1).at(i) = SegmentMap.at("Segment" + num2Str(j+1)).HorizontalAcceleration;
                        Waypoints.at(3).at(i) = Waypoints.at(1).at(i) * (Waypoints.at(0).at(i)-Waypoints.at(0).at(i-1)) + PreviousHorizontalSpeed;
                        Waypoints.at(5).at(i) = 0.5 * Waypoints.at(1).at(i) * pow((Waypoints.at(0).at(i)-Waypoints.at(0).at(i-1)),2) + Waypoints.at(3).at(i-1) * (Waypoints.at(0).at(i)-Waypoints.at(0).at(i-1)) + PreviousHorizontalPosition;
                    }
                    else
                    {
                        if(configXML["Segment@" + num2Str(j+2) + "/HorizontalSpeed"] > configXML["Segment@" + num2Str(j+1) + "/HorizontalSpeed"])
                        {
                            Waypoints.at(1).at(i) = SegmentMap.at("Segment" + num2Str(j+1)).HorizontalAcceleration;
                            Waypoints.at(3).at(i) = Waypoints.at(1).at(i) * (Waypoints.at(0).at(i)-Waypoints.at(0).at(i-1)) + PreviousHorizontalSpeed;
                            Waypoints.at(5).at(i) = 0.5 * Waypoints.at(1).at(i) * pow((Waypoints.at(0).at(i)-Waypoints.at(0).at(i-1)),2) + Waypoints.at(3).at(i-1) * (Waypoints.at(0).at(i)-Waypoints.at(0).at(i-1)) + PreviousHorizontalPosition;
                        }
                        else
                        {
                            if(SegmentMap.at("Segment" + num2Str(j+1)).Time_Hor == SegmentMap.at("Segment" + num2Str(j+1)).Time_PhaseThree_Hor)
                            {
                                Waypoints.at(1).at(i) = -SegmentMap.at("Segment" + num2Str(j+1)).HorizontalAcceleration;
                                Waypoints.at(3).at(i) = Waypoints.at(1).at(i) * (Waypoints.at(0).at(i)-Waypoints.at(0).at(i-1)) + PreviousHorizontalSpeed;
                                Waypoints.at(5).at(i) = 0.5 * Waypoints.at(1).at(i) * pow((Waypoints.at(0).at(i)-Waypoints.at(0).at(i-1)),2) + Waypoints.at(3).at(i-1) * (Waypoints.at(0).at(i)-Waypoints.at(0).at(i-1)) + PreviousHorizontalPosition;
                                if(Waypoints.at(3).at(i) <= 0)
                                {
                                    Waypoints.at(1).at(i) = 0;
                                    Waypoints.at(3).at(i) = 0;
                                    Waypoints.at(5).at(i) = PreviousHorizontalPosition;
                                }
                            }
                            else
                            {
                                Waypoints.at(1).at(i) = -SegmentMap.at("Segment" + num2Str(j+1)).HorizontalAcceleration;
                                Waypoints.at(3).at(i) = Waypoints.at(1).at(i) * (Waypoints.at(0).at(i)-Waypoints.at(0).at(i-1)) + PreviousHorizontalSpeed;
                                Waypoints.at(5).at(i) = 0.5 * Waypoints.at(1).at(i) * pow((Waypoints.at(0).at(i)-Waypoints.at(0).at(i-1)),2) + Waypoints.at(3).at(i-1) * (Waypoints.at(0).at(i)-Waypoints.at(0).at(i-1)) + PreviousHorizontalPosition;
                            }
                        }
                    }
                }
            }
            else
            {
                if (v == 1)
                {
                    if(SegmentMap.at("Segment" + num2Str(j+1)).ExceptionAslopeVert == true)
                    {
                        Waypoints.at(2).at(i) = 0;
                        Waypoints.at(4).at(i) = 0;
                        Waypoints.at(6).at(i) = PreviousVerticalPosition;
                    }
                    else
                    {
                        Waypoints.at(2).at(i) = -SegmentMap.at("Segment" + num2Str(j+1)).VerticalAcceleration;
                        Waypoints.at(4).at(i) = Waypoints.at(2).at(i) * (Waypoints.at(0).at(i)-Waypoints.at(0).at(i-1)) + PreviousVerticalSpeed;
                        Waypoints.at(6).at(i) = 0.5 * Waypoints.at(2).at(i) * pow((Waypoints.at(0).at(i)-Waypoints.at(0).at(i-1)),2) + Waypoints.at(4).at(i-1) * (Waypoints.at(0).at(i)-Waypoints.at(0).at(i-1)) + PreviousVerticalPosition;
                    }
                }
                else if (v == 5)
                {
                    Waypoints.at(2).at(i) = 0;
                    Waypoints.at(4).at(i) = SegmentMap.at("Segment" + num2Str(j+1)).VerticalVelocity;
                    Waypoints.at(6).at(i) = Waypoints.at(4).at(i) * (Waypoints.at(0).at(i)-Waypoints.at(0).at(i-1)) + PreviousVerticalPosition;
                }
                else if (v == 9)
                {
                    if (LastSegment != true)
                    {
                        if(configXML["Segment@" + num2Str(j+2) + "/VerticalSpeed"] <= configXML["Segment@" + num2Str(j+1) + "/VerticalSpeed"])
                        {
                            Waypoints.at(2).at(i) = -SegmentMap.at("Segment" + num2Str(j+1)).VerticalAcceleration;
                            Waypoints.at(4).at(i) = Waypoints.at(2).at(i) * (Waypoints.at(0).at(i)-Waypoints.at(0).at(i-1)) + PreviousVerticalSpeed;
                            Waypoints.at(6).at(i) = 0.5 * Waypoints.at(2).at(i) * pow((Waypoints.at(0).at(i)-Waypoints.at(0).at(i-1)),2) + Waypoints.at(4).at(i-1) * (Waypoints.at(0).at(i)-Waypoints.at(0).at(i-1)) + PreviousVerticalPosition;
                        }
                        else
                        {
                            if(SegmentMap.at("Segment" + num2Str(j+1)).Time_Vert == SegmentMap.at("Segment" + num2Str(j+1)).Time_PhaseThree_Vert)
                            {
                                Waypoints.at(2).at(i) = SegmentMap.at("Segment" + num2Str(j+1)).VerticalAcceleration;
                                Waypoints.at(4).at(i) = Waypoints.at(2).at(i) * (Waypoints.at(0).at(i)-Waypoints.at(0).at(i-1)) + PreviousVerticalSpeed;
                                Waypoints.at(6).at(i) = 0.5 * Waypoints.at(2).at(i) * pow((Waypoints.at(0).at(i)-Waypoints.at(0).at(i-1)),2) + Waypoints.at(4).at(i-1) * (Waypoints.at(0).at(i)-Waypoints.at(0).at(i-1)) + PreviousVerticalPosition;
                                if(Waypoints.at(4).at(i) >= 0)
                                {
                                    Waypoints.at(2).at(i) = 0;
                                    Waypoints.at(4).at(i) = 0;
                                    Waypoints.at(6).at(i) = PreviousVerticalPosition;
                                }
                            }
                            else
                            {
                                Waypoints.at(2).at(i) = SegmentMap.at("Segment" + num2Str(j+1)).VerticalAcceleration;
                                Waypoints.at(4).at(i) = Waypoints.at(2).at(i) * (Waypoints.at(0).at(i)-Waypoints.at(0).at(i-1)) + PreviousVerticalSpeed;
                                Waypoints.at(6).at(i) = 0.5 * Waypoints.at(2).at(i) * pow((Waypoints.at(0).at(i)-Waypoints.at(0).at(i-1)),2) + Waypoints.at(4).at(i-1) * (Waypoints.at(0).at(i)-Waypoints.at(0).at(i-1)) + PreviousVerticalPosition;
                            }
                        }
                    }
                    else
                    {
                        if(SegmentMap.at("Segment" + num2Str(j+1)).ExceptionAslopeVert == true)
                        {
                            Waypoints.at(2).at(i) = -SegmentMap.at("Segment" + num2Str(j+1)).VerticalAcceleration;
                            Waypoints.at(4).at(i) = Waypoints.at(2).at(i) * (Waypoints.at(0).at(i)-Waypoints.at(0).at(i-1)) + PreviousVerticalSpeed;
                            Waypoints.at(6).at(i) = 0.5 * Waypoints.at(2).at(i) * pow((Waypoints.at(0).at(i)-Waypoints.at(0).at(i-1)),2) + Waypoints.at(4).at(i-1) * (Waypoints.at(0).at(i)-Waypoints.at(0).at(i-1)) + PreviousVerticalPosition;
                        }
                        else
                        {
                            if(SegmentMap.at("Segment" + num2Str(j+1)).Time_Vert == SegmentMap.at("Segment" + num2Str(j+1)).Time_PhaseThree_Vert)
                            {
                                Waypoints.at(2).at(i) = SegmentMap.at("Segment" + num2Str(j+1)).VerticalAcceleration;
                                Waypoints.at(4).at(i) = Waypoints.at(2).at(i) * (Waypoints.at(0).at(i)-Waypoints.at(0).at(i-1)) + PreviousVerticalSpeed;
                                Waypoints.at(6).at(i) = 0.5 * Waypoints.at(2).at(i) * pow((Waypoints.at(0).at(i)-Waypoints.at(0).at(i-1)),2) + Waypoints.at(4).at(i-1) * (Waypoints.at(0).at(i)-Waypoints.at(0).at(i-1)) + PreviousVerticalPosition;
                                if(Waypoints.at(4).at(i) >= 0)
                                {
                                    Waypoints.at(2).at(i) = 0;
                                    Waypoints.at(4).at(i) = 0;
                                    Waypoints.at(6).at(i) = PreviousVerticalPosition;
                                }
                            }
                            else
                            {
                                Waypoints.at(2).at(i) = SegmentMap.at("Segment" + num2Str(j+1)).VerticalAcceleration;
                                Waypoints.at(4).at(i) = Waypoints.at(2).at(i) * (Waypoints.at(0).at(i)-Waypoints.at(0).at(i-1)) + PreviousVerticalSpeed;
                                Waypoints.at(6).at(i) = 0.5 * Waypoints.at(2).at(i) * pow((Waypoints.at(0).at(i)-Waypoints.at(0).at(i-1)),2) + Waypoints.at(4).at(i-1) * (Waypoints.at(0).at(i)-Waypoints.at(0).at(i-1)) + PreviousVerticalPosition;
                            }
                        }
                    }
                }

                if(w == 0)
                {
                    if(SegmentMap.at("Segment" + num2Str(j+1)).ExceptionAslopeHor == true)
                    {
                        Waypoints.at(1).at(i) = 0;
                        Waypoints.at(3).at(i) = 0;
                        Waypoints.at(5).at(i) = PreviousHorizontalPosition;
                    }
                    else
                    {
                        Waypoints.at(1).at(i) = SegmentMap.at("Segment" + num2Str(j+1)).HorizontalAcceleration;
                        Waypoints.at(3).at(i) = Waypoints.at(1).at(i) * (Waypoints.at(0).at(i)-Waypoints.at(0).at(i-1)) + PreviousHorizontalSpeed;
                        Waypoints.at(5).at(i) = 0.5 * Waypoints.at(1).at(i) * pow((Waypoints.at(0).at(i)-Waypoints.at(0).at(i-1)),2) + Waypoints.at(3).at(i-1) * (Waypoints.at(0).at(i)-Waypoints.at(0).at(i-1)) + PreviousHorizontalPosition;
                    }
                }
                else if (w == 4)
                {
                    Waypoints.at(1).at(i) = 0;
                    Waypoints.at(3).at(i) = SegmentMap.at("Segment" + num2Str(j+1)).HorizontalVelocity;
                    Waypoints.at(5).at(i) = Waypoints.at(3).at(i) * (Waypoints.at(0).at(i)-Waypoints.at(0).at(i-1)) + PreviousHorizontalPosition;
                }
                else if (w == 8)
                {
                    if (LastSegment != true)
                    {
                        if(configXML["Segment@" + num2Str(j+2) + "/HorizontalSpeed"] >= configXML["Segment@" + num2Str(j+1) + "/HorizontalSpeed"])
                        {
                            Waypoints.at(1).at(i) = SegmentMap.at("Segment" + num2Str(j+1)).HorizontalAcceleration;
                            Waypoints.at(3).at(i) = Waypoints.at(1).at(i) * (Waypoints.at(0).at(i)-Waypoints.at(0).at(i-1)) + PreviousHorizontalSpeed;
                            Waypoints.at(5).at(i) = 0.5 * Waypoints.at(1).at(i) * pow((Waypoints.at(0).at(i)-Waypoints.at(0).at(i-1)),2) + Waypoints.at(3).at(i-1) * (Waypoints.at(0).at(i)-Waypoints.at(0).at(i-1)) + PreviousHorizontalPosition;
                        }
                        else
                        {
                            if(SegmentMap.at("Segment" + num2Str(j+1)).Time_Hor == SegmentMap.at("Segment" + num2Str(j+1)).Time_PhaseThree_Hor)
                            {
                                Waypoints.at(1).at(i) = -SegmentMap.at("Segment" + num2Str(j+1)).HorizontalAcceleration;
                                Waypoints.at(3).at(i) = Waypoints.at(1).at(i) * (Waypoints.at(0).at(i)-Waypoints.at(0).at(i-1)) + PreviousHorizontalSpeed;
                                Waypoints.at(5).at(i) = 0.5 * Waypoints.at(1).at(i) * pow((Waypoints.at(0).at(i)-Waypoints.at(0).at(i-1)),2) + Waypoints.at(3).at(i-1) * (Waypoints.at(0).at(i)-Waypoints.at(0).at(i-1)) + PreviousHorizontalPosition;
                                if(Waypoints.at(4).at(i) <= 0)
                                {
                                    Waypoints.at(1).at(i) = 0;
                                    Waypoints.at(3).at(i) = 0;
                                    Waypoints.at(5).at(i) = PreviousHorizontalPosition;
                                }
                            }
                            else
                            {
                                Waypoints.at(1).at(i) = -SegmentMap.at("Segment" + num2Str(j+1)).HorizontalAcceleration;
                                Waypoints.at(3).at(i) = Waypoints.at(1).at(i) * (Waypoints.at(0).at(i)-Waypoints.at(0).at(i-1)) + PreviousHorizontalSpeed;
                                Waypoints.at(5).at(i) = 0.5 * Waypoints.at(1).at(i) * pow((Waypoints.at(0).at(i)-Waypoints.at(0).at(i-1)),2) + Waypoints.at(3).at(i-1) * (Waypoints.at(0).at(i)-Waypoints.at(0).at(i-1)) + PreviousHorizontalPosition;
                            }

                        }
                    }
                    else
                    {
                        if(SegmentMap.at("Segment" + num2Str(j+1)).ExceptionAslopeHor == true)
                        {
                            Waypoints.at(1).at(i) = SegmentMap.at("Segment" + num2Str(j+1)).HorizontalAcceleration;
                            Waypoints.at(3).at(i) = Waypoints.at(1).at(i) * (Waypoints.at(0).at(i)-Waypoints.at(0).at(i-1)) + PreviousHorizontalSpeed;
                            Waypoints.at(5).at(i) = 0.5 * Waypoints.at(1).at(i) * pow((Waypoints.at(0).at(i)-Waypoints.at(0).at(i-1)),2) + Waypoints.at(4).at(i-1) * (Waypoints.at(0).at(i)-Waypoints.at(0).at(i-1)) + PreviousHorizontalPosition;
                        }
                        else
                        {
                            if(SegmentMap.at("Segment" + num2Str(j+1)).Time_Hor == SegmentMap.at("Segment" + num2Str(j+1)).Time_PhaseThree_Hor)
                            {
                                Waypoints.at(1).at(i) = -SegmentMap.at("Segment" + num2Str(j+1)).HorizontalAcceleration;
                                Waypoints.at(3).at(i) = Waypoints.at(1).at(i) * (Waypoints.at(0).at(i)-Waypoints.at(0).at(i-1)) + PreviousHorizontalSpeed;
                                Waypoints.at(5).at(i) = 0.5 * Waypoints.at(1).at(i) * pow((Waypoints.at(0).at(i)-Waypoints.at(0).at(i-1)),2) + Waypoints.at(3).at(i-1) * (Waypoints.at(0).at(i)-Waypoints.at(0).at(i-1)) + PreviousHorizontalPosition;
                                if(Waypoints.at(4).at(i) <= 0)
                                {
                                    Waypoints.at(1).at(i) = 0;
                                    Waypoints.at(3).at(i) = 0;
                                    Waypoints.at(5).at(i) = PreviousHorizontalPosition;
                                }
                            }
                            else
                            {
                                Waypoints.at(1).at(i) = -SegmentMap.at("Segment" + num2Str(j+1)).HorizontalAcceleration;
                                Waypoints.at(3).at(i) = Waypoints.at(1).at(i) * (Waypoints.at(0).at(i)-Waypoints.at(0).at(i-1)) + PreviousHorizontalSpeed;
                                Waypoints.at(5).at(i) = 0.5 * Waypoints.at(1).at(i) * pow((Waypoints.at(0).at(i)-Waypoints.at(0).at(i-1)),2) + Waypoints.at(3).at(i-1) * (Waypoints.at(0).at(i)-Waypoints.at(0).at(i-1)) + PreviousHorizontalPosition;
                            }
                        }
                    }
                }
            }
        }

        PreviousVerticalSpeed = Waypoints.at(4).at(i);
        PreviousVerticalPosition = Waypoints.at(6).at(i);
        PreviousHorizontalSpeed = Waypoints.at(3).at(i);
        PreviousHorizontalPosition = Waypoints.at(5).at(i);

        if(i == (int(MissionTime)*configXML["IterationTimeFactor"]))
        {
            Waypoints.at(1).at(i) = 0; // horizontal acceleration
            Waypoints.at(2).at(i) = 0; // vertical acceleration
            Waypoints.at(3).at(i) = 0; // horizontal velocity
            Waypoints.at(4).at(i) = 0; // vertical velocity

            Waypoints.at(5).at(i) = 0; //initialisieren
            for (int z = 0; z<=j; z++)
            {
                Waypoints.at(5).at(i) += SegmentMap.at("Segment" + num2Str(z+1)).HorizontalDistance;
            }

            Waypoints.at(6).at(i) = 0; // vertical position

            break;
        }
    }


    const string waypoints_file = "Waypoints.csv";
    ofstream waypoints;
    waypoints.open(waypoints_file.c_str());
    for (int i = 0; i < Waypoints.at(0).size(); i++)
    {
        waypoints << Waypoints.at(0).at(i) << ";";
        waypoints << Waypoints.at(1).at(i) << ";";
        waypoints << Waypoints.at(2).at(i) << ";";
        waypoints << Waypoints.at(3).at(i) << ";";
        waypoints << Waypoints.at(4).at(i) << ";";
        waypoints << Waypoints.at(5).at(i) << ";";
        waypoints << Waypoints.at(6).at(i);
        waypoints << endl;
    }
    waypoints.close();

    string plotCommand = "gnuplot\\gnuplot.exe \ plots\\Mission.plt";
    handleChildProcess(plotCommand, "");
    myRuntimeInfo->out << "Mission Profile is calculated and plotted" << endl;

    return Waypoints;
}

void MissionAnalysis::writeResults(int LoopNumber)
{
    const string missionResult_file = "Results\\MissionResults_Loop" + num2Str(LoopNumber) + ".csv";
    ofstream missionResults;
    missionResults.open(missionResult_file.c_str());
    missionResults << "Time;Horizontal Acceleration;Vertical Acceleration;Horizontal Velocity;Vertical Velocity;Horizontal Position;Vertical Position;Power;Thrust;Drag;Delta;Alpha;MissionEnergy" << endl;
    for (int i = 0; i < Waypoints.at(0).size(); i++)
    {
        missionResults << Waypoints.at(0).at(i) << ";";
        missionResults << Waypoints.at(1).at(i) << ";";
        missionResults << Waypoints.at(2).at(i) << ";";
        missionResults << Waypoints.at(3).at(i) << ";";
        missionResults << Waypoints.at(4).at(i) << ";";
        missionResults << Waypoints.at(5).at(i) << ";";
        missionResults << Waypoints.at(6).at(i) << ";";
        missionResults << MissionResults.at(0).at(i) << ";";
        missionResults << MissionResults.at(1).at(i) << ";";
        missionResults << MissionResults.at(2).at(i) << ";";
        missionResults << MissionResults.at(3).at(i) << ";";
        missionResults << MissionResults.at(4).at(i) << ";";
        missionResults << MissionResults.at(5).at(i) << ";";
        missionResults << endl;
    }
    missionResults.close();
}

void MissionAnalysis::doMissionAnalysis ()
{
    myRuntimeInfo->out << "Performing Mission Analysis" << endl;
    Propeller myProp(configXML, "TestProp.csv","TestAirfoil.csv","clarky.dat");
    //Aerodynamics myAero(configXML);
    BladeElementTheory myBET(configXML, myProp);

    double eps = 1;
    double LoopNumber = 1;

    myMassEstimationPt->doMassEstimation(0,0); //Initial guess with P_max = E_M = 0;
    myMassEstimationPt->writeResults(LoopNumber);

    do
    {
        //vector<vector<double>> MissionResults(6, vector<double>(Waypoints.at(0).size(), 0.0));
        double TimeStepEnergy = 0;
        double omega_start = 210;
        double m = myMassEstimationPt->VehicleMass;
        double MTOW = myMassEstimationPt->VehicleMass;
        double MTOW_old = MTOW;

        for (int i = 0; i < Waypoints.at(0).size(); i++)
        {
            cout << "Calculating Waypoint " << i+1 << "/" << (Waypoints.at(0).size());
            cout << '\r';

            vector<double> iterateThrustAndDragResults = myAeroPt->iterateThrustAndDrag(Waypoints.at(3).at(i), Waypoints.at(4).at(i), Waypoints.at(1).at(i), Waypoints.at(2).at(i), m, MTOW, Waypoints.at(6).at(i)); // alpha, drag, delta, thrust

            /** Compound Configuration - Wing **/
            if (configXML["AddWing"] == 1)
            {
                double AoA = iterateThrustAndDragResults.at(0)+iterateThrustAndDragResults.at(2);
                if (AoA <= (15*PI/180) && AoA >= (-5*PI/180))
                {
                    myWingPt->calcWing(AoA+3);
                    atmosphere myatmo;
                    double v_res = sqrt(pow(Waypoints.at(3).at(i),2) + pow(Waypoints.at(4).at(i),2));
                    double WingLift = 0.5 * myatmo.getDensity(Waypoints.at(6).at(i)) * pow(v_res,2) * myWingPt->WingArea * myWingPt->AVLresults.at(0);
                    cout << AoA << endl << myWingPt->AVLresults.at(0) << endl << v_res << endl << WingLift << endl;
                    getchar();
                    calcBETResults = myBET.calcBET(Waypoints.at(3).at(i), Waypoints.at(4).at(i),  Waypoints.at(6).at(i), ((iterateThrustAndDragResults.at(3)-WingLift)/configXML["NumberOfRotors"]), iterateThrustAndDragResults.at(2), omega_start);
                }
                else
                {
                    calcBETResults = myBET.calcBET(Waypoints.at(3).at(i), Waypoints.at(4).at(i),  Waypoints.at(6).at(i), (iterateThrustAndDragResults.at(3)/configXML["NumberOfRotors"]), iterateThrustAndDragResults.at(2), omega_start);
                }

            }
            else
            {
                calcBETResults = myBET.calcBET(Waypoints.at(3).at(i), Waypoints.at(4).at(i),  Waypoints.at(6).at(i), (iterateThrustAndDragResults.at(3)/configXML["NumberOfRotors"]), iterateThrustAndDragResults.at(2), omega_start);
            }

            omega_start = calcBETResults.at(0);
            double currentPower = calcBETResults.at(1);

            /** Helicopter Configuration - Tailrotor **/
            if (configXML["NumberOfRotors"] == 1)
            {
                currentPower = 1.1 * currentPower;
            }

            TimeStepEnergy += currentPower * MissionTimeStep;
            MissionResults.at(0).push_back(currentPower);
            MissionResults.at(1).push_back(iterateThrustAndDragResults.at(3)); //thrust
            MissionResults.at(2).push_back(iterateThrustAndDragResults.at(1)); //drag
            MissionResults.at(3).push_back(iterateThrustAndDragResults.at(2)); //delta
            MissionResults.at(4).push_back(iterateThrustAndDragResults.at(0)); //alpha
            MissionResults.at(5).push_back(TimeStepEnergy);
        }
        cout << endl;

        this->writeResults(LoopNumber);
        LoopNumber++;
        myMassEstimationPt->doMassEstimation(MissionResults.at(0).at(std::max_element(MissionResults.at(0).begin(), MissionResults.at(0).end())-MissionResults.at(0).begin()), MissionResults.at(5).back());
        myMassEstimationPt->writeResults(LoopNumber);
        eps = abs(myMassEstimationPt->VehicleMass - MTOW_old)/MTOW_old;
        MissionResults.at(0).clear();
        MissionResults.at(1).clear();
        MissionResults.at(2).clear();
        MissionResults.at(3).clear();
        MissionResults.at(4).clear();
        MissionResults.at(5).clear();
    }
    while(eps > 0.1);
}











































MissionAnalysis::~MissionAnalysis()
{
    //dtor
}
