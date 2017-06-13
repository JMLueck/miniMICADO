#ifndef SEGMENT_H
#define SEGMENT_H

#include <math.h>
#include <vector>
#include "node.h"
#include "functions.h"

using namespace std;

class Segment
{
    public:
        Segment(node& configXML, int SegmentNumber);
        virtual ~Segment();

        int Number;
        string Type;
        double HoverTime;
        double HorizontalDistance;
        double VerticalDistance;
        double HorizontalVelocity;
        double VerticalVelocity;
        double HorizontalAcceleration;
        double VerticalAcceleration;
        double SlopeVelocity;
        double SlopeVelocity1;
        double SlopeVelocity2;
        double Time_PhaseOne;
        double Time_PhaseTwo;
        double Time_PhaseThree;
        double VerticalWay_PhaseOne;
        double VerticalWay_PhaseTwo;
        double VerticalWay_PhaseThree;
        double HorizontalWay_PhaseOne;
        double HorizontalWay_PhaseTwo;
        double HorizontalWay_PhaseThree;
        double Way_PhaseOne_Hor;
        double Way_PhaseTwo_Hor;
        double Way_PhaseThree_Hor;
        double Time_PhaseOne_Hor;
        double Time_PhaseTwo_Hor;
        double Time_PhaseThree_Hor;
        double Time_Hor;
        double Way_PhaseOne_Vert;
        double Way_PhaseTwo_Vert;
        double Way_PhaseThree_Vert;
        double Time_PhaseOne_Vert;
        double Time_PhaseTwo_Vert;
        double Time_PhaseThree_Vert;
        double Time_Vert;
        double Time;


        void getSegmentType();
        void readConfigXML();
        void getSegmentTimeAndWay();

        double calculateSlopeVelocity(double VelocityStart, double VelocityEnd, double Acceleration, double MissionTime, double MissionWay);

    protected:

    private:
        node& configXML;
};

#endif // SEGMENT_H
