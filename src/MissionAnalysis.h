#ifndef MISSIONANALYSIS_H
#define MISSIONANALYSIS_H

#include <vector>
#include "node.h"
#include "functions.h"
#include "Segment.h"
#include "Propeller.h"
#include "Wing.h"
#include "MassEstimation.h"
#include "Aerodynamics.h"

using namespace std;

class MissionAnalysis
{
    public:

        MissionAnalysis(node& configXML, MassEstimation &myMassEstimation, Wing &myWing, Aerodynamics &myAero);
        virtual ~MissionAnalysis();

        vector<vector<double>> Segments;
        vector<double> SegmentsTime;

        double MissionTime;
        double MissionTimeStep;

        vector<vector<double>> Waypoints;
        vector<vector<double>> MissionResults;
        map<string,Segment> SegmentMap;

        void calcMissionSegments();
        vector<vector<double>> calcMissionWaypoints();
        void calcMissionTime();
        void calcSegmentsTimeAndWay();

        int getSegmentNumber(double currentTime);
        int getVerticalPhaseNumber(double currentTime);
        int getHorizontalPhaseNumber(double currentTime);
        void doMissionAnalysis();
        void writeResults(int LoopNumber);

        vector<double> calcBETResults;

        bool LastSegment;

    protected:

    private:
        node& configXML;
        MassEstimation *myMassEstimationPt;
        Wing *myWingPt;
        Aerodynamics *myAeroPt;
};

#endif // MISSIONANALYSIS_H
