#ifndef MISSIONANALYSIS_H
#define MISSIONANALYSIS_H

#include <vector>
#include "node.h"
#include "functions.h"
#include "Segment.h"

using namespace std;

class MissionAnalysis
{
    public:

        vector<vector<double>> Segments;
        vector<double> SegmentsTime;
        //vector<double> SegmentsTimeVert;

        double MissionTime;

        vector<vector<double>> Waypoints;
        map<string,Segment> SegmentMap;

        MissionAnalysis(node& configXML);
        virtual ~MissionAnalysis();

        void calcMissionSegments();
        void calcMissionWaypoints();
        void calcMissionTime();
        void calcSegmentsTimeAndWay();

        int getSegmentNumber(double currentTime);
        int getVerticalPhaseNumber(double currentTime);
        int getHorizontalPhaseNumber(double currentTime);

        bool LastSegment;

    protected:

    private:
        node& configXML;
};

#endif // MISSIONANALYSIS_H
