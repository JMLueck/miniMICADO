#ifndef MISSIONANALYSIS_H
#define MISSIONANALYSIS_H

#include <vector>
#include "node.h"
#include "functions.h"
#include "Segment.h"
#include "Propeller.h"

using namespace std;

class MissionAnalysis
{
    public:

        vector<vector<double>> Segments;
        vector<double> SegmentsTime;

        double MissionTime;

        vector<vector<double>> Waypoints;
        vector<vector<double>> MissionResults;
        map<string,Segment> SegmentMap;

        MissionAnalysis(node& configXML);
        virtual ~MissionAnalysis();

        void calcMissionSegments();
        vector<vector<double>> calcMissionWaypoints();
        void calcMissionTime();
        void calcSegmentsTimeAndWay();

        int getSegmentNumber(double currentTime);
        int getVerticalPhaseNumber(double currentTime);
        int getHorizontalPhaseNumber(double currentTime);
        void doMissionAnalysis();

        bool LastSegment;

    protected:

    private:
        node& configXML;
};

#endif // MISSIONANALYSIS_H
