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
        double getPower(Propeller myProp, double v_hor, double v_vert, double altitude, double a_hor, double a_vert, double m, double MTOW);

        bool LastSegment;

    protected:

    private:
        node& configXML;
};

#endif // MISSIONANALYSIS_H
