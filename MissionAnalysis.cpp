#include "MissionAnalysis.h"

/** Analysis of Power and Energy Consumption **/
/** Possibly implementation of 6-DOF equations here **/

MissionAnalysis::MissionAnalysis(node& configXML)
    :
    configXML(configXML),
    Segments(configXML["NumberOfSegments"], vector<double>(12,0.0)),
    SegmentsTime(configXML["NumberOfSegments"], 0.0),
    SegmentsTimeVert(configXML["NumberOfSegments"], 0.0),
    MissionTime(0.0)
{
    calcMissionSegments();
    calcMissionTime();
    calcMissionWaypoints();
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
        if (currentTime >= (SegmentsTime.at(j-1) + Segments.at(j).at(1)) && currentTime <= (SegmentsTime.at(j-1) + SegmentsTimeVert.at(j) - Segments.at(j).at(9)))
        {
            return 5;   // middle phase
        }
        if (currentTime >= SegmentsTime.at(j-1) && currentTime <= (SegmentsTime.at(j-1) + SegmentsTimeVert.at(j) - Segments.at(j).at(9) - Segments.at(j).at(5)))
        {
            return 1;   // first phase
        }
        if (currentTime >= (SegmentsTime.at(j-1) + Segments.at(j).at(1) + Segments.at(j).at(5)) && currentTime <= (SegmentsTime.at(j)+ SegmentsTimeVert.at(j)))
        {
            return 9;
        }
    }
    else
    {
        if (currentTime >= (0 + Segments.at(j).at(1)) && currentTime <= (0 + SegmentsTimeVert.at(j) - Segments.at(j).at(9)))
        {
            return 5;
        }
        if (currentTime >= 0  && currentTime <= (0 + SegmentsTimeVert.at(j) - Segments.at(j).at(9) - Segments.at(j).at(5)))
        {
            return 1;
        }
        if (currentTime >= (0 + Segments.at(j).at(1) + Segments.at(j).at(5))  && currentTime <= (0 + SegmentsTimeVert.at(j) - Segments.at(j).at(9)))
        {
            return 9;
        }
    }
}

void MissionAnalysis::calcMissionWaypoints()
{
    double TimeSegHor = 0;
    double TimeSegVert = 0;
    double MissionTimeStep = MissionTime / int(MissionTime); // close to 1sec and evenly distributed
    vector<vector<double>> Waypoints (7, vector<double>(int(MissionTime),0.0));

    /** initialize step one **/
    Waypoints.at(0).at(0) = 0; // time
    Waypoints.at(1).at(0) = configXML["Segment@1/HorizontalAcceleration"]; // horizontal acceleration
    Waypoints.at(2).at(0) = configXML["Segment@1/VerticalAcceleration"]; // vertical acceleration
    Waypoints.at(3).at(0) = 0; // horizontal velocity
    Waypoints.at(4).at(0) = 0; // vertical velocity
    Waypoints.at(5).at(0) = 0; // horizontal position
    Waypoints.at(6).at(0) = 0; // vertical position

    for (int i = 1; i <= int(MissionTime); i++)
    {
        Waypoints.at(0).at(i) = MissionTimeStep * i;

        int j = getSegmentNumber(Waypoints.at(0).at(i));
        int v = getVerticalPhaseNumber(Waypoints.at(0).at(i));

        // horizontal acceleration
        if (configXML["Segment@"+num2Str(j+1)+"/HorizontalAcceleration"] == 0)
        {
            Waypoints.at(1).at(i) = 0;
        }
        else
        {
            if (Waypoints.at(0).at(i) > (SegmentsTime.at(j) - Segments.at(j).at(8))) // third phase of current segment
            {
                Waypoints.at(1).at(i) = configXML["Segment@"+num2Str(j+1)+"/HorizontalAcceleration"];
            }
        }

        // vertical acceleration

        if ( Segments.at(j).at(v) == 0 || getVerticalPhaseNumber(Waypoints.at(0).at(i)) == 5 )
        {
            Waypoints.at(2).at(i) = 0;
        }
        else
        {
            if (j == (configXML["NumberOfSegments"]-1))
            {
                if ( (v=1 && configXML["Segment@"+num2Str(j)+"/VerticalSpeed"] < configXML["Segment@"+num2Str(j+1)+"/VerticalSpeed"]) || (v=9 && configXML["Segment@"+num2Str(j+1)+"/VerticalSpeed"] < 0) )
                {
                    Waypoints.at(2).at(i) = configXML["Segment@"+num2Str(j+1)+"/VerticalAcceleration"];
                }
                else
                {
                    Waypoints.at(2).at(i) = -configXML["Segment@"+num2Str(j+1)+"/VerticalAcceleration"];
                }
            }
            else if  (j == 0)
            {
                if ( (v=1 && 0 < configXML["Segment@"+num2Str(j+1)+"/VerticalSpeed"]) || (v=9 && configXML["Segment@"+num2Str(j+1)+"/VerticalSpeed"] < configXML["Segment@"+num2Str(j+2)+"/VerticalSpeed"]) )
                {
                    Waypoints.at(2).at(i) = configXML["Segment@"+num2Str(j+1)+"/VerticalAcceleration"];;
                }
                else
                {
                    Waypoints.at(2).at(i) = -configXML["Segment@"+num2Str(j+1)+"/VerticalAcceleration"];
                }
            }
            else
            {
                if ( (v=1 && configXML["Segment@"+num2Str(j)+"/VerticalSpeed"] < configXML["Segment@"+num2Str(j+1)+"/VerticalSpeed"]) || (v=9 && configXML["Segment@"+num2Str(j+1)+"/VerticalSpeed"] < configXML["Segment@"+num2Str(j+2)+"/VerticalSpeed"]) )
                {
                    Waypoints.at(2).at(i) = configXML["Segment@"+num2Str(j+1)+"/VerticalAcceleration"];;
                }
                else
                {
                    Waypoints.at(2).at(i) = -configXML["Segment@"+num2Str(j+1)+"/VerticalAcceleration"];
                }
            }
        }

        Waypoints.at(3).at(i) = Waypoints.at(3).at(i-1) + Waypoints.at(1).at(i) * MissionTimeStep;

        if (getVerticalPhaseNumber(Waypoints.at(0).at(i)) == 5)
        {
            Waypoints.at(4).at(i) = configXML["Segment@"+num2Str(j+1)+"/VerticalSpeed"];
        }
        else
        {
            Waypoints.at(4).at(i) = Waypoints.at(4).at(i-1) + Waypoints.at(2).at(i) * MissionTimeStep;
        }

        Waypoints.at(5).at(i) = 0.5 * Waypoints.at(1).at(i) * pow(MissionTimeStep,2) + Waypoints.at(3).at(i) * MissionTimeStep + Waypoints.at(5).at(i-1);
        Waypoints.at(6).at(i) = 0.5 * Waypoints.at(2).at(i) * pow(MissionTimeStep,2) + Waypoints.at(4).at(i) * MissionTimeStep + Waypoints.at(6).at(i-1);
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
        waypoints << Waypoints.at(6).at(i) << ";";
        waypoints << endl;
    }
    waypoints.close();

}



//void MissionAnalysis::doMissionAnalysis()
//{
//    calcMissionWaypoints()
//}


MissionAnalysis::~MissionAnalysis()
{
    //dtor
}
