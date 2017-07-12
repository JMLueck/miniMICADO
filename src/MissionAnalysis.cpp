#include "MissionAnalysis.h"
#include "functions.h"
#include "Aerodynamics.h"
#include "BladeElementTheory.h"


/** Analysis of Power and Energy Consumption **/
/** Possibly implementation of 6-DOF equations here **/

MissionAnalysis::MissionAnalysis(node& configXML)
    :
    configXML(configXML),
    Segments(configXML["NumberOfSegments"], vector<double>(12,0.0)),
    SegmentsTime(configXML["NumberOfSegments"], 0.0),
    MissionTime(0.0)
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
    double MissionTimeStep = MissionTime / (configXML["IterationTimeFactor"]*int(MissionTime)); // close to 1sec and evenly distributed

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
        waypoints << Waypoints.at(0).at(i) << ",";
        waypoints << Waypoints.at(1).at(i) << ",";
        waypoints << Waypoints.at(2).at(i) << ",";
        waypoints << Waypoints.at(3).at(i) << ",";
        waypoints << Waypoints.at(4).at(i) << ",";
        waypoints << Waypoints.at(5).at(i) << ",";
        waypoints << Waypoints.at(6).at(i) <<endl;
        waypoints << endl;
    }
    waypoints.close();

    string plotCommand = "gnuplot\\gnuplot.exe \ plots\\Mission.plt";
    handleChildProcess(plotCommand, "");
    myRuntimeInfo->out << "Mission Profile is calculated and plotted" << endl;

    return Waypoints;
}

double MissionAnalysis::getPower(Propeller myProp, double v_hor, double v_vert, double altitude, double a_hor, double a_vert, double m, double MTOW)
{
    Aerodynamics myAero;
    BladeElementTheory myBET(myProp);

    vector<double> iterateThrustAndDragResults = myAero.iterateThrustAndDrag(v_hor, v_vert, a_hor, a_vert, m, MTOW, altitude); // alpha, drag, delta, thrust
    double Power = myBET.calcBET(v_hor, v_vert, altitude, iterateThrustAndDragResults.at(3), iterateThrustAndDragResults.at(2));

    return Power;
}

void MissionAnalysis::doMissionAnalysis ()
{
    myRuntimeInfo->out << "Performing Mission Analysis" << endl;
    Propeller myProp("TestProp.csv","TestAirfoil.csv");

    for (int i = 0; i < Waypoints.at(0).size(); i++)
    {
        double currentTime = Waypoints.at(0).at(i);
        double currentHorizontalAcceleration = Waypoints.at(1).at(i);
        double currentVerticalAcceleration = Waypoints.at(2).at(i);
        double currentHorizontalVelocity = Waypoints.at(3).at(i);
        double currentVerticalVelocity = Waypoints.at(4).at(i);
        double currentHorizontalPosition = Waypoints.at(5).at(i);
        double currentVerticalPosition = Waypoints.at(6).at(i);

        double m = 10; // hier getMass einfügen
        double MTOW = 15;

        double currentPower = getPower(myProp, currentHorizontalVelocity, currentVerticalVelocity, currentVerticalPosition, currentHorizontalAcceleration, currentVerticalAcceleration, m, MTOW);
        cout << currentPower << endl;
    }
}











































MissionAnalysis::~MissionAnalysis()
{
    //dtor
}
