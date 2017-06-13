#include "Segment.h"

Segment::Segment(node& configXML, int SegmentNumber)
    :
    configXML(configXML),
    Number(SegmentNumber),
    Type(""),
    HoverTime(0),
    HorizontalDistance(0),
    VerticalDistance(0),
    HorizontalVelocity(0),
    VerticalVelocity(0),
    HorizontalAcceleration(0),
    VerticalAcceleration(0),
    SlopeVelocity(0),
    SlopeVelocity1(0),
    SlopeVelocity2(0),
    Time_PhaseOne(0),
    Time_PhaseTwo(0),
    Time_PhaseThree(0),
    VerticalWay_PhaseOne(0),
    VerticalWay_PhaseTwo(0),
    VerticalWay_PhaseThree(0),
    HorizontalWay_PhaseOne(0),
    HorizontalWay_PhaseTwo(0),
    HorizontalWay_PhaseThree(0),
    Way_PhaseOne_Hor(0),
    Way_PhaseTwo_Hor(0),
    Way_PhaseThree_Hor(0),
    Time_PhaseOne_Hor(0),
    Time_PhaseTwo_Hor(0),
    Time_PhaseThree_Hor(0),
    Time_Hor(0),
    Way_PhaseOne_Vert(0),
    Way_PhaseTwo_Vert(0),
    Way_PhaseThree_Vert(0),
    Time_PhaseOne_Vert(0),
    Time_PhaseTwo_Vert(0),
    Time_PhaseThree_Vert(0),
    Time_Vert(0),
    Time(0)

{
    getSegmentType();
    readConfigXML();
    getSegmentTimeAndWay();
    //ctor
}


void Segment::getSegmentType()
{
    if (configXML["Segment@"+num2Str(Number)+"/HorizontalDistance"] == 0 && configXML["Segment@"+num2Str(Number)+"/VerticalDistance"] == 0)
    {
        Type = "Hover";
    }
    else if (configXML["Segment@"+num2Str(Number)+"/HorizontalDistance"] != 0 && configXML["Segment@"+num2Str(Number)+"/VerticalDistance"] == 0)
    {
        Type = "Horizontal";
    }
    else if (configXML["Segment@"+num2Str(Number)+"/HorizontalDistance"] == 0 && configXML["Segment@"+num2Str(Number)+"/VerticalDistance"] != 0)
    {
        Type = "Vertical";
    }
    else
    {
        Type = "Aslope";
    }
}

void Segment::readConfigXML()
{
    if (Type == "Hover")
    {
        HoverTime = configXML["Segment@"+num2Str(Number)+"/HoverTime"];
        HorizontalDistance = 0;
        VerticalDistance = 0;
        HorizontalVelocity = 0;
        VerticalVelocity = 0;
        HorizontalAcceleration = 0;
        VerticalAcceleration = 0;
    }
    else
    {
        HorizontalDistance = configXML["Segment@"+num2Str(Number)+"/HorizontalDistance"];
        VerticalDistance = configXML["Segment@"+num2Str(Number)+"/VerticalDistance"];
        HorizontalVelocity = configXML["Segment@"+num2Str(Number)+"/HorizontalSpeed"];
        VerticalVelocity = configXML["Segment@"+num2Str(Number)+"/VerticalSpeed"];
        HorizontalAcceleration = configXML["Segment@"+num2Str(Number)+"/HorizontalAcceleration"];
        VerticalAcceleration = configXML["Segment@"+num2Str(Number)+"/VerticalAcceleration"];
    }
}

double Segment::calculateSlopeVelocity(double VelocityStart, double VelocityEnd, double SlopeAcceleration, double MissionTime, double MissionWay)
{
    /** Annahme, dass Geschwindigkeit des nächsten Segments größer ist als die zu errechnende Reisefluggeschwindigkeit des aktuellen Segments **/
    SlopeVelocity = (2*MissionWay*SlopeAcceleration + pow(VelocityStart,2) - pow(VelocityEnd,2))/(2*MissionTime*SlopeAcceleration + 2*VelocityStart - 2*VelocityEnd);

    if (VelocityEnd < SlopeVelocity) /** obige Annahme wird überprüft und ggf. wird zu errechnende Reisefluggeschwindigkeit neu berechnet  **/
    {
        SlopeVelocity1 = -0.5*(-MissionTime*SlopeAcceleration - VelocityStart - VelocityEnd) + sqrt(pow((0.5*(-MissionTime*SlopeAcceleration - VelocityStart - VelocityEnd)),2) - 0.5*pow(VelocityStart,2) - 0.5*pow(VelocityEnd,2) - MissionWay*SlopeAcceleration);
        SlopeVelocity2 = -0.5*(-MissionTime*SlopeAcceleration - VelocityStart - VelocityEnd) - sqrt(pow((0.5*(-MissionTime*SlopeAcceleration - VelocityStart - VelocityEnd)),2) - 0.5*pow(VelocityStart,2) - 0.5*pow(VelocityEnd,2) - MissionWay*SlopeAcceleration);
        /** Es soll die minimale Geschwindigkeit gewählt werden, es sei denn selbige ist negativ **/
        if (SlopeVelocity1 <= 0 && SlopeVelocity2 > 0)
        {
            SlopeVelocity = SlopeVelocity2;
        }
        else if (SlopeVelocity1 > 0 && SlopeVelocity2 <= 0)
        {
            SlopeVelocity = SlopeVelocity1;
        }
        else
        {
            SlopeVelocity = min(SlopeVelocity1,SlopeVelocity2);
        }
    }
    return SlopeVelocity;
}

void Segment::getSegmentTimeAndWay()
{
    /** IF: First Segment **/
    if (Number == 1)
    {
        if (VerticalVelocity < 0) /** Exception für Flug in negative Höhen **/
        {
            myRuntimeInfo->err << "Negative Velocity at First Segment. Please adjust the settings!" << endl;
        }
        else
        {
            if (Type == "Vertical") /** Startsegment ist ein senkrechtes Steigsegment **/
            {
                Time_PhaseOne = VerticalVelocity / VerticalAcceleration;
                VerticalWay_PhaseOne = 0.5 * VerticalAcceleration * pow(Time_PhaseOne,2);

                /** Wenn nächstes Segment ein Sinksegment ist, muss zunächst auf Geschwindigkeit 0 gebremst werden und dann wird auf negative
                Geschwindigkeit beschleunigt. Sonst würde das Fluggerät das erste Segment nie beenden können **/

                if (configXML["Segment@"+num2Str(Number+1)+"/VerticalSpeed"] < 0)
                {
                    Time_PhaseThree = VerticalVelocity / VerticalAcceleration;
                    VerticalWay_PhaseThree = VerticalVelocity * Time_PhaseThree - 0.5 * VerticalAcceleration * pow(Time_PhaseThree,2);
                }
                else
                {
                    Time_PhaseThree = abs((VerticalVelocity - configXML["Segment@"+num2Str(Number+1)+"/VerticalSpeed"]) / VerticalAcceleration);
                    if (VerticalVelocity < configXML["Segment@"+num2Str(Number+1)+"/VerticalSpeed"])
                    {
                        VerticalWay_PhaseThree = VerticalVelocity * Time_PhaseThree + 0.5 * VerticalAcceleration * pow(Time_PhaseThree,2);
                    }
                    else
                    {
                        VerticalWay_PhaseThree = VerticalVelocity * Time_PhaseThree - 0.5 * VerticalAcceleration * pow(Time_PhaseThree,2);
                    }
                }

                VerticalWay_PhaseTwo = VerticalDistance - VerticalWay_PhaseOne - VerticalWay_PhaseThree;
                Time_PhaseTwo = VerticalWay_PhaseTwo / VerticalVelocity;
                Time = Time_PhaseOne + Time_PhaseTwo + Time_PhaseThree;
            }
            else if (Type == "Aslope") /** Startsegment ist ein schräges Segment mit Steig- und Vorwärtsgeschwindigkeit **/
            {
                /** Calculate horizontal and vertical independently, then check which takes longer.
                    The shorter one will be set to new, matching acceleration **/
                Time_PhaseOne_Vert = VerticalVelocity / VerticalAcceleration;
                Time_PhaseOne_Hor = HorizontalVelocity / HorizontalAcceleration;
                VerticalWay_PhaseOne = 0.5 * VerticalAcceleration * pow(Time_PhaseOne_Vert,2);
                HorizontalWay_PhaseOne = 0.5 * HorizontalAcceleration * pow(Time_PhaseOne_Hor,2);

                if (configXML["Segment@"+num2Str(Number+1)+"/VerticalSpeed"] < 0)
                /** Nächstes Segment ist Sinksegment, auf Geschwindigkeit 0 abbremsen und
                    anschließend auf negative Geschwindigkeit beschleunigen **/
                {
                    Time_PhaseThree_Vert = abs(VerticalVelocity / VerticalAcceleration);
                    VerticalWay_PhaseThree = VerticalVelocity * Time_PhaseThree_Vert - 0.5 * VerticalAcceleration * pow(Time_PhaseThree_Vert,2);
                }
                else
                {
                    Time_PhaseThree_Vert = abs((VerticalVelocity - configXML["Segment@"+num2Str(Number+1)+"/VerticalSpeed"]) / VerticalAcceleration);
                    if (VerticalVelocity < configXML["Segment@"+num2Str(Number+1)+"/VerticalSpeed"])
                    {
                        VerticalWay_PhaseThree = VerticalVelocity * Time_PhaseThree_Vert + 0.5 * VerticalAcceleration * pow(Time_PhaseThree_Vert,2);
                    }
                    else
                    {
                        VerticalWay_PhaseThree = VerticalVelocity * Time_PhaseThree_Vert - 0.5 * VerticalAcceleration * pow(Time_PhaseThree_Vert,2);
                    }
                }

                /** Horizontale Geschwindigkeit wird nicht 0, da wir nicht "zurück" fliegen wollen **/
                Time_PhaseThree_Hor = abs((HorizontalVelocity - configXML["Segment@"+num2Str(Number+1)+"/HorizontalSpeed"]) / HorizontalAcceleration);
                if (HorizontalVelocity < configXML["Segment@"+num2Str(Number+1)+"/HorizontalSpeed"])
                {
                    HorizontalWay_PhaseThree = HorizontalVelocity * Time_PhaseThree_Hor + 0.5 * HorizontalAcceleration * pow(Time_PhaseThree_Hor,2);
                }
                else
                {
                    HorizontalWay_PhaseThree = HorizontalVelocity * Time_PhaseThree_Hor - 0.5 * HorizontalAcceleration * pow(Time_PhaseThree_Hor,2);
                }

                VerticalWay_PhaseTwo = VerticalDistance - VerticalWay_PhaseOne - VerticalWay_PhaseThree;
                HorizontalWay_PhaseTwo = HorizontalDistance - HorizontalWay_PhaseOne - HorizontalWay_PhaseThree;
                Time_PhaseTwo_Vert = VerticalWay_PhaseTwo / VerticalVelocity;
                Time_PhaseTwo_Hor = HorizontalWay_PhaseTwo / HorizontalVelocity;
                Time_Vert = Time_PhaseOne_Vert + Time_PhaseTwo_Vert + Time_PhaseThree_Vert;
                Time_Hor = Time_PhaseOne_Hor + Time_PhaseTwo_Hor + Time_PhaseThree_Hor;

                /** Check, which is longer and recalculate shorter one **/
                if (Time_Hor > Time_Vert)
                {
                    Time_Vert = Time_Hor;
                    if(configXML["Segment@"+num2Str(Number+1)+"/VerticalSpeed"] > 0)
                    {
                        configXML["Segment@"+num2Str(Number)+"/VerticalAcceleration"] = configXML["Segment@"+num2Str(Number+1)+"/VerticalSpeed"] / Time_Vert;
                    }
                    else
                    {
                        /** Reisefluggeschwindigkeit wird neu berechnet, Geschwindigkeit von 0 auf 0 **/
                        configXML["Segment@"+num2Str(Number)+"/VerticalSpeed"] = calculateSlopeVelocity(0,0,VerticalAcceleration,Time_Vert,VerticalDistance);
                        VerticalVelocity = configXML["Segment@"+num2Str(Number)+"/VerticalSpeed"];
                        Time_PhaseOne_Vert = VerticalVelocity / VerticalAcceleration;
                        Way_PhaseOne_Vert = 0.5 * VerticalAcceleration * pow(Time_PhaseOne_Vert,2);
                        Time_PhaseThree_Vert = Time_PhaseOne_Vert;
                        Way_PhaseThree_Vert = Way_PhaseOne_Vert;
                        Way_PhaseTwo_Vert = VerticalDistance - Way_PhaseOne_Vert - Way_PhaseThree_Vert;
                        Time_PhaseTwo_Vert = Way_PhaseTwo_Vert / VerticalVelocity;
                    }
                }
                else if (Time_Hor < Time_Vert)
                {
                    Time_Hor = Time_Vert;
                    if(configXML["Segment@"+num2Str(Number+1)+"/HorizontalSpeed"] > 0)
                    {
                        configXML["Segment@"+num2Str(Number)+"/HorizontalAcceleration"] = configXML["Segment@"+num2Str(Number+1)+"/HorizontalSpeed"] / Time_Hor;
                    }
                    else
                    {
                        /** Reisefluggeschwindigkeit wird neu berechnet, Geschwindigkeit von 0 auf 0 **/
                        configXML["Segment@"+num2Str(Number)+"/HorizontalSpeed"] = calculateSlopeVelocity(0,0,HorizontalAcceleration,Time_Hor,HorizontalDistance);
                        HorizontalVelocity = configXML["Segment@"+num2Str(Number)+"/HorizontalSpeed"];
                        Time_PhaseOne_Hor = HorizontalVelocity / HorizontalAcceleration;
                        Way_PhaseOne_Hor = 0.5 * HorizontalAcceleration * pow(Time_PhaseOne_Hor,2);
                        Time_PhaseThree_Hor = Time_PhaseOne_Hor;
                        Way_PhaseThree_Hor = Way_PhaseOne_Hor;
                        Way_PhaseTwo_Hor = HorizontalDistance - Way_PhaseOne_Hor - Way_PhaseThree_Hor;
                        Time_PhaseTwo_Hor = Way_PhaseTwo_Hor / HorizontalVelocity;
                    }
                }
                Time = Time_Hor;
            }
            else
            {
                myRuntimeInfo->err << "First Segment is not suitable. Please adjust the settings!" << endl;
            }

        }
    }


    /** All middle Segments **/
    if (Number != 1 && Number != configXML["NumberOfSegments"])
    {
        if(Type == "Hover")
        {
            /** Im schwebenden Segment ist die Flugzeit immer die Schwebezeit,
            egal welche Segmente vorher abgeflogen wurden und welche anschließend folgen **/

            Time = configXML["Segment@"+num2Str(Number)+"/HoverTime"];
        }
        else if(Type == "Vertical")
        {
            if ((configXML["Segment@"+num2Str(Number-1)+"/VerticalSpeed"] < 0 && configXML["Segment@"+num2Str(Number)+"/VerticalSpeed"] < 0) || (configXML["Segment@"+num2Str(Number-1)+"/VerticalSpeed"] > 0 && configXML["Segment@"+num2Str(Number)+"/VerticalSpeed"] > 0))
            {
                /** Wenn Geschwindigkeiten vorher und aktuell gleiches Vorzeichen haben, dann muss anfangs nicht beschleunigt werden **/
                if ((configXML["Segment@"+num2Str(Number)+"/VerticalSpeed"] < 0 && configXML["Segment@"+num2Str(Number+1)+"/VerticalSpeed"] < 0) || (configXML["Segment@"+num2Str(Number)+"/VerticalSpeed"] > 0 && configXML["Segment@"+num2Str(Number+1)+"/VerticalSpeed"] > 0))
                {
                    /** Wenn Geschwindigkeit im nächsten Segment gleiches Vorzeichen hat, dann muss nicht auf 0 abgebremst werden **/
                    Time_PhaseThree = abs((VerticalVelocity - configXML["Segment@"+num2Str(Number+1)+"/VerticalSpeed"]) / VerticalAcceleration);
                    if (abs(VerticalVelocity) < abs(configXML["Segment@"+num2Str(Number+1)+"/VerticalSpeed"]))
                    {
                        VerticalWay_PhaseThree = abs(VerticalVelocity * Time_PhaseThree) + 0.5 * VerticalAcceleration * pow(Time_PhaseThree,2);
                    }
                    else
                    {
                        VerticalWay_PhaseThree = abs(VerticalVelocity * Time_PhaseThree) - 0.5 * VerticalAcceleration * pow(Time_PhaseThree,2);
                    }
                    VerticalWay_PhaseTwo = VerticalDistance - VerticalWay_PhaseThree;
                    Time_PhaseTwo = abs(VerticalWay_PhaseTwo / VerticalVelocity);
                    Time = Time_PhaseThree + Time_PhaseTwo;
                }
                else
                {
                    /** Wenn Geschwindigkeit im nächsten Segment unterschiedliches Vorzeichen hat, dann muss auf 0 abgebremst werden **/
                    Time_PhaseThree = abs(VerticalVelocity / VerticalAcceleration);
                    VerticalWay_PhaseThree = abs(VerticalVelocity * Time_PhaseThree) - 0.5 * VerticalAcceleration * pow(Time_PhaseThree,2);
                    VerticalWay_PhaseTwo = VerticalDistance - VerticalWay_PhaseThree;
                    Time_PhaseTwo = abs(VerticalWay_PhaseTwo / VerticalVelocity);
                    Time = Time_PhaseThree + Time_PhaseTwo;
                }
            }
            else
            {
                if ((configXML["Segment@"+num2Str(Number)+"/VerticalSpeed"] < 0 && configXML["Segment@"+num2Str(Number+1)+"/VerticalSpeed"] < 0) || (configXML["Segment@"+num2Str(Number)+"/VerticalSpeed"] > 0 && configXML["Segment@"+num2Str(Number+1)+"/VerticalSpeed"] > 0))
                {
                    /** Wenn Geschwindigkeit im nächsten Segment gleiches Vorzeichen hat, dann muss nicht auf 0 abgebremst werden **/
                    Time_PhaseOne = abs(VerticalVelocity / VerticalAcceleration);
                    VerticalWay_PhaseOne = 0.5 * VerticalAcceleration * pow(Time_PhaseOne,2);
                    Time_PhaseThree = abs((VerticalVelocity - configXML["Segment@"+num2Str(Number+1)+"/VerticalSpeed"]) / VerticalAcceleration);
                    if (abs(VerticalVelocity) < abs(configXML["Segment@"+num2Str(Number+1)+"/VerticalSpeed"]))
                    {
                        VerticalWay_PhaseThree = abs(VerticalVelocity * Time_PhaseThree) + 0.5 * VerticalAcceleration * pow(Time_PhaseThree,2);
                    }
                    else
                    {
                        VerticalWay_PhaseThree = abs(VerticalVelocity * Time_PhaseThree) - 0.5 * VerticalAcceleration * pow(Time_PhaseThree,2);
                    }
                    VerticalWay_PhaseTwo = VerticalDistance - VerticalWay_PhaseOne - VerticalWay_PhaseThree;
                    Time_PhaseTwo = abs(VerticalWay_PhaseTwo / VerticalVelocity);
                    Time = Time_PhaseOne + Time_PhaseTwo + Time_PhaseThree;
                }
                else
                {
                    /** Wenn Geschwindigkeit im nächsten Segment unterschiedliches Vorzeichen hat, dann muss auf 0 abgebremst werden **/
                    Time_PhaseOne = abs(VerticalVelocity / VerticalAcceleration);
                    VerticalWay_PhaseOne = 0.5 * VerticalAcceleration * pow(Time_PhaseOne,2);
                    Time_PhaseThree = abs(VerticalVelocity / VerticalAcceleration);
                    VerticalWay_PhaseThree = abs(VerticalVelocity * Time_PhaseThree) - 0.5 * VerticalAcceleration * pow(Time_PhaseThree,2);
                    VerticalWay_PhaseTwo = VerticalDistance - VerticalWay_PhaseOne - VerticalWay_PhaseThree;
                    Time_PhaseTwo = abs(VerticalWay_PhaseTwo / VerticalVelocity);
                    Time = Time_PhaseOne + Time_PhaseTwo + Time_PhaseThree;
                }
            }
        }

        else if(Type == "Horizontal")
        {
            if((configXML["Segment@"+num2Str(Number-1)+"/HorizontalDistance"] == 0 && configXML["Segment@"+num2Str(Number-1)+"/VerticalDistance"] == 0) || (configXML["Segment@"+num2Str(Number-1)+"/HorizontalDistance"] == 0 && configXML["Segment@"+num2Str(Number-1)+"/VerticalDistance"] != 0))
            {
                /** Wenn vertikales Segment oder Schwebesegment vorher war, dann muss anfangs beschleunigt werden **/
                Time_PhaseOne = HorizontalVelocity / HorizontalAcceleration;
                HorizontalWay_PhaseOne = 0.5 * HorizontalAcceleration * pow(Time_PhaseOne,2);
                Time_PhaseThree = abs((HorizontalVelocity - configXML["Segment@"+num2Str(Number+1)+"/HorizontalSpeed"]) / HorizontalAcceleration);
                if (HorizontalVelocity < configXML["Segment@"+num2Str(Number+1)+"/HorizontalSpeed"])
                {
                    HorizontalWay_PhaseThree = HorizontalVelocity * Time_PhaseThree + 0.5 * HorizontalAcceleration * pow(Time_PhaseThree,2);
                }
                else
                {
                    HorizontalWay_PhaseThree = HorizontalVelocity * Time_PhaseThree - 0.5 * HorizontalAcceleration * pow(Time_PhaseThree,2);
                }
                HorizontalWay_PhaseTwo = HorizontalDistance - HorizontalWay_PhaseOne - HorizontalWay_PhaseThree;
                Time_PhaseTwo = HorizontalWay_PhaseTwo / HorizontalVelocity;
                Time = Time_PhaseOne + Time_PhaseTwo + Time_PhaseThree;
            }
            else
            {
                /** Wenn schräges Segment oder horizontales Segment vorher war, dann muss anfangs nicht beschleunigt werden **/
                Time_PhaseThree = abs((HorizontalVelocity - configXML["Segment@"+num2Str(Number+1)+"/HorizontalSpeed"]) / HorizontalAcceleration);
                if (HorizontalVelocity < configXML["Segment@"+num2Str(Number+1)+"/HorizontalSpeed"])
                {
                    HorizontalWay_PhaseThree = HorizontalVelocity * Time_PhaseThree + 0.5 * HorizontalAcceleration * pow(Time_PhaseThree,2);
                }
                else
                {
                    HorizontalWay_PhaseThree = HorizontalVelocity * Time_PhaseThree - 0.5 * HorizontalAcceleration * pow(Time_PhaseThree,2);
                }
                HorizontalWay_PhaseTwo = HorizontalDistance - HorizontalWay_PhaseThree;
                Time_PhaseTwo = HorizontalWay_PhaseTwo / HorizontalVelocity;
                Time = Time_PhaseTwo + Time_PhaseThree;
            }
        }

        else if(Type == "Aslope")
        {
            if(configXML["Segment@"+num2Str(Number-1)+"/HorizontalDistance"] == 0 && configXML["Segment@"+num2Str(Number-1)+"/VerticalDistance"] == 0)
            {
                /** Wenn Schwebesegment vorher war, muss anfangs beschleunigt werden **/

                /** Calculate horizontal and vertical independently, then check which takes longer.
                    The shorter one will be set to new, matching acceleration **/
                Time_PhaseOne_Vert = abs(VerticalVelocity / VerticalAcceleration);
                Time_PhaseOne_Hor = HorizontalVelocity / HorizontalAcceleration;
                VerticalWay_PhaseOne = 0.5 * VerticalAcceleration * pow(Time_PhaseOne_Vert,2);
                HorizontalWay_PhaseOne = 0.5 * HorizontalAcceleration * pow(Time_PhaseOne_Hor,2);

                if ((configXML["Segment@"+num2Str(Number)+"/VerticalSpeed"] > 0 && configXML["Segment@"+num2Str(Number+1)+"/VerticalSpeed"] > 0) || (configXML["Segment@"+num2Str(Number)+"/VerticalSpeed"] < 0 && configXML["Segment@"+num2Str(Number+1)+"/VerticalSpeed"] < 0))
                {
                    /** Wenn die aktuelle Vertikalgeschwindigkeit und die darauffolgende das gleiche Vorzeichen haben, muss nicht auf 0 abgebremst werden **/

                    Time_PhaseThree_Vert = abs((VerticalVelocity - configXML["Segment@"+num2Str(Number+1)+"/VerticalSpeed"]) / VerticalAcceleration);
                    Time_PhaseThree_Hor = abs((HorizontalVelocity - configXML["Segment@"+num2Str(Number+1)+"/HorizontalSpeed"]) / HorizontalAcceleration);
                    if (abs(VerticalVelocity) < abs(configXML["Segment@"+num2Str(Number+1)+"/VerticalSpeed"]))
                    {
                        VerticalWay_PhaseThree = abs(VerticalVelocity * Time_PhaseThree_Vert) + 0.5 * VerticalAcceleration * pow(Time_PhaseThree_Vert,2);
                    }
                    else
                    {
                        VerticalWay_PhaseThree = abs(VerticalVelocity * Time_PhaseThree_Vert) - 0.5 * VerticalAcceleration * pow(Time_PhaseThree_Vert,2);
                    }
                    if (HorizontalVelocity < configXML["Segment@"+num2Str(Number+1)+"/HorizontalSpeed"])
                    {
                        HorizontalWay_PhaseThree = HorizontalVelocity * Time_PhaseThree_Hor + 0.5 * HorizontalAcceleration * pow(Time_PhaseThree_Hor,2);
                    }
                    else
                    {
                        HorizontalWay_PhaseThree = HorizontalVelocity * Time_PhaseThree_Hor - 0.5 * HorizontalAcceleration * pow(Time_PhaseThree_Hor,2);
                    }
                }
                else
                {
                    /** Wenn die aktuelle Vertikalgeschwindigkeit und die darauffolgende unterschiedliche Vorzeichen haben, muss auf 0 abgebremst werden **/

                    Time_PhaseThree_Vert = abs(VerticalVelocity / VerticalAcceleration);
                    Time_PhaseThree_Hor = abs((HorizontalVelocity - configXML["Segment@"+num2Str(Number+1)+"/HorizontalSpeed"]) / HorizontalAcceleration);
                    VerticalWay_PhaseThree = abs(VerticalVelocity * Time_PhaseThree_Vert) - 0.5 * VerticalAcceleration * pow(Time_PhaseThree_Vert,2);

                    if (HorizontalVelocity < configXML["Segment@"+num2Str(Number+1)+"/HorizontalSpeed"])
                    {
                        HorizontalWay_PhaseThree = HorizontalVelocity * Time_PhaseThree_Hor + 0.5 * HorizontalAcceleration * pow(Time_PhaseThree_Hor,2);
                    }
                    else
                    {
                        HorizontalWay_PhaseThree = HorizontalVelocity * Time_PhaseThree_Hor - 0.5 * HorizontalAcceleration * pow(Time_PhaseThree_Hor,2);
                    }
                }
                VerticalWay_PhaseTwo = VerticalDistance - VerticalWay_PhaseOne - VerticalWay_PhaseThree;
                HorizontalWay_PhaseTwo = HorizontalDistance - HorizontalWay_PhaseOne - HorizontalWay_PhaseThree;
                Time_PhaseTwo_Vert = abs(VerticalWay_PhaseTwo / VerticalVelocity);
                Time_PhaseTwo_Hor = HorizontalWay_PhaseTwo / HorizontalVelocity;
                Time_Vert = Time_PhaseOne_Vert + Time_PhaseTwo_Vert + Time_PhaseThree_Vert;
                Time_Hor = Time_PhaseOne_Hor + Time_PhaseTwo_Hor + Time_PhaseThree_Hor;

                /** Check, which is longer and recalculate shorter one **/
                if (Time_Hor > Time_Vert)
                {
                    Time_Vert = Time_Hor;
                    if (configXML["Segment@"+num2Str(Number)+"/VerticalSpeed"] > 0)
                    {
                        /**Annahme: positive Geschwindigkeiten im aktuellen Segment **/

                        if(configXML["Segment@"+num2Str(Number+1)+"/VerticalSpeed"] > 0)
                        {
                            configXML["Segment@"+num2Str(Number)+"/VerticalAcceleration"] = configXML["Segment@"+num2Str(Number+1)+"/VerticalSpeed"] / Time_Vert;
                        }
                        else
                        {
                            /** Reisefluggeschwindigkeit wird neu berechnet, Geschwindigkeit von 0 auf 0 **/
                            configXML["Segment@"+num2Str(Number)+"/VerticalSpeed"] = calculateSlopeVelocity(0,0,VerticalAcceleration,Time_Vert,VerticalDistance);
                            VerticalVelocity = configXML["Segment@"+num2Str(Number)+"/VerticalSpeed"];
                            Time_PhaseOne_Vert = VerticalVelocity / VerticalAcceleration;
                            Way_PhaseOne_Vert = 0.5 * VerticalAcceleration * pow(Time_PhaseOne_Vert,2);
                            Time_PhaseThree_Vert = Time_PhaseOne_Vert;
                            Way_PhaseThree_Vert = Way_PhaseOne_Vert;
                            Way_PhaseTwo_Vert = VerticalDistance - Way_PhaseOne_Vert - Way_PhaseThree_Vert;
                            Time_PhaseTwo_Vert = Way_PhaseTwo_Vert / VerticalVelocity;
                        }
                    }
                    else
                    {
                        /**Annahme: negative Geschwindigkeiten im aktuellen Segment **/

                        if(configXML["Segment@"+num2Str(Number+1)+"/VerticalSpeed"] > 0)
                        {
                            /** Reisefluggeschwindigkeit wird neu berechnet, Geschwindigkeit von 0 auf 0 **/
                            configXML["Segment@"+num2Str(Number)+"/VerticalSpeed"] = (-1) * calculateSlopeVelocity(0,0,VerticalAcceleration,Time_Vert,VerticalDistance);
                            VerticalVelocity = configXML["Segment@"+num2Str(Number)+"/VerticalSpeed"];
                            Time_PhaseOne_Vert = abs(VerticalVelocity / VerticalAcceleration);
                            Way_PhaseOne_Vert = abs(0.5 * VerticalAcceleration * pow(Time_PhaseOne_Vert,2));
                            Time_PhaseThree_Vert = Time_PhaseOne_Vert;
                            Way_PhaseThree_Vert = Way_PhaseOne_Vert;
                            Way_PhaseTwo_Vert = VerticalDistance - Way_PhaseOne_Vert - Way_PhaseThree_Vert;
                            Time_PhaseTwo_Vert = abs(Way_PhaseTwo_Vert / VerticalVelocity);
                        }
                        else
                        {
                            configXML["Segment@"+num2Str(Number)+"/VerticalAcceleration"] = abs(configXML["Segment@"+num2Str(Number+1)+"/VerticalSpeed"] / Time_Vert);
                        }
                    }

                }
                else if (Time_Hor < Time_Vert)
                {
                    Time_Hor = Time_Vert;
                    if(configXML["Segment@"+num2Str(Number+1)+"/HorizontalSpeed"] > 0)
                    {
                        configXML["Segment@"+num2Str(Number)+"/HorizontalAcceleration"] = configXML["Segment@"+num2Str(Number+1)+"/HorizontalSpeed"] / Time_Hor;
                    }
                    else
                    {
                        /** Reisefluggeschwindigkeit wird neu berechnet, Geschwindigkeit von 0 auf 0 **/
                        configXML["Segment@"+num2Str(Number)+"/HorizontalSpeed"] = calculateSlopeVelocity(0,0,HorizontalAcceleration,Time_Hor,HorizontalDistance);
                        HorizontalVelocity = configXML["Segment@"+num2Str(Number)+"/HorizontalSpeed"];
                        Time_PhaseOne_Hor = HorizontalVelocity / HorizontalAcceleration;
                        Way_PhaseOne_Hor = 0.5 * HorizontalAcceleration * pow(Time_PhaseOne_Hor,2);
                        Time_PhaseThree_Hor = Time_PhaseOne_Hor;
                        Way_PhaseThree_Hor = Way_PhaseOne_Hor;
                        Way_PhaseTwo_Hor = HorizontalDistance - Way_PhaseOne_Hor - Way_PhaseThree_Hor;
                        Time_PhaseTwo_Hor = Way_PhaseTwo_Hor / HorizontalVelocity;
                    }
                }
                Time = Time_Hor;
            }

            else if(configXML["Segment@"+num2Str(Number-1)+"/HorizontalDistance"] != 0 && configXML["Segment@"+num2Str(Number-1)+"/VerticalDistance"] == 0)
            {
                /** Wenn horizontales Segment vorher war, muss anfangs in horizontale Richtung nicht beschleunigt werden **/

                /** Calculate horizontal and vertical independently, then check which takes longer.
                    The shorter one will be set to new, matching acceleration **/
                Time_PhaseOne_Vert = abs(VerticalVelocity / VerticalAcceleration);
                VerticalWay_PhaseOne = 0.5 * VerticalAcceleration * pow(Time_PhaseOne_Vert,2);

                if ((configXML["Segment@"+num2Str(Number)+"/VerticalSpeed"] > 0 && configXML["Segment@"+num2Str(Number+1)+"/VerticalSpeed"] > 0) || (configXML["Segment@"+num2Str(Number)+"/VerticalSpeed"] < 0 && configXML["Segment@"+num2Str(Number+1)+"/VerticalSpeed"] < 0))
                {
                    /** Wenn die aktuelle Vertikalgeschwindigkeit und die darauffolgende das gleiche Vorzeichen haben, muss nicht auf 0 abgebremst werden **/

                    Time_PhaseThree_Vert = abs((VerticalVelocity - configXML["Segment@"+num2Str(Number+1)+"/VerticalSpeed"]) / VerticalAcceleration);
                    Time_PhaseThree_Hor = abs((HorizontalVelocity - configXML["Segment@"+num2Str(Number+1)+"/HorizontalSpeed"]) / HorizontalAcceleration);
                    if (abs(VerticalVelocity) < abs(configXML["Segment@"+num2Str(Number+1)+"/VerticalSpeed"]))
                    {
                        VerticalWay_PhaseThree = abs(VerticalVelocity * Time_PhaseThree_Vert) + 0.5 * VerticalAcceleration * pow(Time_PhaseThree_Vert,2);
                    }
                    else
                    {
                        VerticalWay_PhaseThree = abs(VerticalVelocity * Time_PhaseThree_Vert) - 0.5 * VerticalAcceleration * pow(Time_PhaseThree_Vert,2);
                    }
                    if (HorizontalVelocity < configXML["Segment@"+num2Str(Number+1)+"/HorizontalSpeed"])
                    {
                        HorizontalWay_PhaseThree = HorizontalVelocity * Time_PhaseThree_Hor + 0.5 * HorizontalAcceleration * pow(Time_PhaseThree_Hor,2);
                    }
                    else
                    {
                        HorizontalWay_PhaseThree = HorizontalVelocity * Time_PhaseThree_Hor - 0.5 * HorizontalAcceleration * pow(Time_PhaseThree_Hor,2);
                    }
                }
                else
                {
                    /** Wenn die aktuelle Vertikalgeschwindigkeit und die darauffolgende unterschiedliche Vorzeichen haben, muss auf 0 abgebremst werden **/

                    Time_PhaseThree_Vert = abs(VerticalVelocity / VerticalAcceleration);
                    Time_PhaseThree_Hor = abs((HorizontalVelocity - configXML["Segment@"+num2Str(Number+1)+"/HorizontalSpeed"]) / HorizontalAcceleration);
                    VerticalWay_PhaseThree = abs(VerticalVelocity * Time_PhaseThree_Vert) - 0.5 * VerticalAcceleration * pow(Time_PhaseThree_Vert,2);

                    if (HorizontalVelocity < configXML["Segment@"+num2Str(Number+1)+"/HorizontalSpeed"])
                    {
                        HorizontalWay_PhaseThree = HorizontalVelocity * Time_PhaseThree_Hor + 0.5 * HorizontalAcceleration * pow(Time_PhaseThree_Hor,2);
                    }
                    else
                    {
                        HorizontalWay_PhaseThree = HorizontalVelocity * Time_PhaseThree_Hor - 0.5 * HorizontalAcceleration * pow(Time_PhaseThree_Hor,2);
                    }
                }
                VerticalWay_PhaseTwo = VerticalDistance - VerticalWay_PhaseOne - VerticalWay_PhaseThree;
                HorizontalWay_PhaseTwo = HorizontalDistance - HorizontalWay_PhaseThree;
                Time_PhaseTwo_Vert = abs(VerticalWay_PhaseTwo / VerticalVelocity);
                Time_PhaseTwo_Hor = HorizontalWay_PhaseTwo / HorizontalVelocity;
                Time_Vert = Time_PhaseOne_Vert + Time_PhaseTwo_Vert + Time_PhaseThree_Vert;
                Time_Hor = Time_PhaseTwo_Hor + Time_PhaseThree_Hor;

                /** Check, which is longer and recalculate shorter one **/
                if (Time_Hor > Time_Vert)
                {
                    Time_Vert = Time_Hor;
                    if (configXML["Segment@"+num2Str(Number)+"/VerticalSpeed"] > 0)
                    {
                        /**Annahme: positive Geschwindigkeiten im aktuellen Segment **/

                        if(configXML["Segment@"+num2Str(Number+1)+"/VerticalSpeed"] > 0)
                        {
                            configXML["Segment@"+num2Str(Number)+"/VerticalAcceleration"] = configXML["Segment@"+num2Str(Number+1)+"/VerticalSpeed"] / Time_Vert;
                        }
                        else
                        {
                            /** Reisefluggeschwindigkeit wird neu berechnet, Geschwindigkeit von 0 auf 0 **/
                            configXML["Segment@"+num2Str(Number)+"/VerticalSpeed"] = calculateSlopeVelocity(0,0,VerticalAcceleration,Time_Vert,VerticalDistance);
                            VerticalVelocity = configXML["Segment@"+num2Str(Number)+"/VerticalSpeed"];
                            Time_PhaseOne_Vert = VerticalVelocity / VerticalAcceleration;
                            Way_PhaseOne_Vert = 0.5 * VerticalAcceleration * pow(Time_PhaseOne_Vert,2);
                            Time_PhaseThree_Vert = Time_PhaseOne_Vert;
                            Way_PhaseThree_Vert = Way_PhaseOne_Vert;
                            Way_PhaseTwo_Vert = VerticalDistance - Way_PhaseOne_Vert - Way_PhaseThree_Vert;
                            Time_PhaseTwo_Vert = Way_PhaseTwo_Vert / VerticalVelocity;
                        }
                    }
                    else
                    {
                        /**Annahme: negative Geschwindigkeiten im aktuellen Segment **/

                        if(configXML["Segment@"+num2Str(Number+1)+"/VerticalSpeed"] > 0)
                        {
                            /** Reisefluggeschwindigkeit wird neu berechnet, Geschwindigkeit von 0 auf 0 **/
                            configXML["Segment@"+num2Str(Number)+"/VerticalSpeed"] = (-1) * calculateSlopeVelocity(0,0,VerticalAcceleration,Time_Vert,VerticalDistance);
                            VerticalVelocity = configXML["Segment@"+num2Str(Number)+"/VerticalSpeed"];
                            Time_PhaseOne_Vert = abs(VerticalVelocity / VerticalAcceleration);
                            Way_PhaseOne_Vert = 0.5 * VerticalAcceleration * pow(Time_PhaseOne_Vert,2);
                            Time_PhaseThree_Vert = Time_PhaseOne_Vert;
                            Way_PhaseThree_Vert = Way_PhaseOne_Vert;
                            Way_PhaseTwo_Vert = VerticalDistance - Way_PhaseOne_Vert - Way_PhaseThree_Vert;
                            Time_PhaseTwo_Vert = abs(Way_PhaseTwo_Vert / VerticalVelocity);
                        }
                        else
                        {
                            configXML["Segment@"+num2Str(Number)+"/VerticalAcceleration"] = abs(configXML["Segment@"+num2Str(Number+1)+"/VerticalSpeed"] / Time_Vert);
                        }
                    }

                }
                else if (Time_Hor < Time_Vert)
                {
                    Time_Hor = Time_Vert;
                    if(configXML["Segment@"+num2Str(Number+1)+"/HorizontalSpeed"] > 0)
                    {
                        configXML["Segment@"+num2Str(Number)+"/HorizontalAcceleration"] = abs(configXML["Segment@"+num2Str(Number+1)+"/HorizontalSpeed"] - configXML["Segment@"+num2Str(Number)+"/HorizontalSpeed"])/ Time_Hor;
                    }
                    else
                    {
                        /** Reisefluggeschwindigkeit wird neu berechnet, Geschwindigkeit von aktueller Geschwindigkeit auf 0 **/
                        configXML["Segment@"+num2Str(Number)+"/HorizontalSpeed"] = calculateSlopeVelocity(HorizontalVelocity,0,HorizontalAcceleration,Time_Hor,HorizontalDistance);
                        HorizontalVelocity = configXML["Segment@"+num2Str(Number)+"/HorizontalSpeed"];
                        Time_PhaseOne_Hor = HorizontalVelocity / HorizontalAcceleration;
                        Way_PhaseOne_Hor = 0.5 * HorizontalAcceleration * pow(Time_PhaseOne_Hor,2);
                        Time_PhaseThree_Hor = Time_PhaseOne_Hor;
                        Way_PhaseThree_Hor = Way_PhaseOne_Hor;
                        Way_PhaseTwo_Hor = HorizontalDistance - Way_PhaseOne_Hor - Way_PhaseThree_Hor;
                        Time_PhaseTwo_Hor = Way_PhaseTwo_Hor / HorizontalVelocity;
                    }
                }
                Time = Time_Hor;
            }
            else if(configXML["Segment@"+num2Str(Number-1)+"/HorizontalDistance"] == 0 && configXML["Segment@"+num2Str(Number-1)+"/VerticalDistance"] != 0)
            {
                if ((configXML["Segment@"+num2Str(Number-1)+"/VerticalSpeed"] > 0 && configXML["Segment@"+num2Str(Number)+"/VerticalSpeed"] < 0) || (configXML["Segment@"+num2Str(Number-1)+"/VerticalSpeed"] < 0 && configXML["Segment@"+num2Str(Number)+"/VerticalSpeed"] > 0))
                {
                    /** Wenn Steigsegment mit unterschiedlichem Vorzeichen in der vertikalen Geschwindigkeit vorher war, muss anfangs beschleunigt werden. **/

                    /** Calculate horizontal and vertical independently, then check which takes longer.
                        The shorter one will be set to new, matching acceleration **/
                    Time_PhaseOne_Vert = abs(VerticalVelocity / VerticalAcceleration);
                    Time_PhaseOne_Hor = HorizontalVelocity / HorizontalAcceleration;
                    VerticalWay_PhaseOne = 0.5 * VerticalAcceleration * pow(Time_PhaseOne_Vert,2);
                    HorizontalWay_PhaseOne = 0.5 * HorizontalAcceleration * pow(Time_PhaseOne_Hor,2);

                    if ((configXML["Segment@"+num2Str(Number)+"/VerticalSpeed"] > 0 && configXML["Segment@"+num2Str(Number+1)+"/VerticalSpeed"] > 0) || (configXML["Segment@"+num2Str(Number)+"/VerticalSpeed"] < 0 && configXML["Segment@"+num2Str(Number+1)+"/VerticalSpeed"] < 0))
                    {
                        /** Wenn die aktuelle Vertikalgeschwindigkeit und die darauffolgende das gleiche Vorzeichen haben, muss nicht auf 0 abgebremst werden **/

                        Time_PhaseThree_Vert = abs((VerticalVelocity - configXML["Segment@"+num2Str(Number+1)+"/VerticalSpeed"]) / VerticalAcceleration);
                        Time_PhaseThree_Hor = abs((HorizontalVelocity - configXML["Segment@"+num2Str(Number+1)+"/HorizontalSpeed"]) / HorizontalAcceleration);
                        if (abs(VerticalVelocity) < abs(configXML["Segment@"+num2Str(Number+1)+"/VerticalSpeed"]))
                        {
                            VerticalWay_PhaseThree = abs(VerticalVelocity * Time_PhaseThree_Vert) + 0.5 * VerticalAcceleration * pow(Time_PhaseThree_Vert,2);
                        }
                        else
                        {
                            VerticalWay_PhaseThree = abs(VerticalVelocity * Time_PhaseThree_Vert) - 0.5 * VerticalAcceleration * pow(Time_PhaseThree_Vert,2);
                        }
                        if (HorizontalVelocity < configXML["Segment@"+num2Str(Number+1)+"/HorizontalSpeed"])
                        {
                            HorizontalWay_PhaseThree = HorizontalVelocity * Time_PhaseThree_Hor + 0.5 * HorizontalAcceleration * pow(Time_PhaseThree_Hor,2);
                        }
                        else
                        {
                            HorizontalWay_PhaseThree = HorizontalVelocity * Time_PhaseThree_Hor - 0.5 * HorizontalAcceleration * pow(Time_PhaseThree_Hor,2);
                        }
                    }
                    else
                    {
                        /** Wenn die aktuelle Vertikalgeschwindigkeit und die darauffolgende unterschiedliche Vorzeichen haben, muss auf 0 abgebremst werden **/

                        Time_PhaseThree_Vert = abs(VerticalVelocity / VerticalAcceleration);
                        Time_PhaseThree_Hor = abs((HorizontalVelocity - configXML["Segment@"+num2Str(Number+1)+"/HorizontalSpeed"]) / HorizontalAcceleration);
                        VerticalWay_PhaseThree = abs(VerticalVelocity * Time_PhaseThree_Vert) - 0.5 * VerticalAcceleration * pow(Time_PhaseThree_Vert,2);

                        if (HorizontalVelocity < configXML["Segment@"+num2Str(Number+1)+"/HorizontalSpeed"])
                        {
                            HorizontalWay_PhaseThree = HorizontalVelocity * Time_PhaseThree_Hor + 0.5 * HorizontalAcceleration * pow(Time_PhaseThree_Hor,2);
                        }
                        else
                        {
                            HorizontalWay_PhaseThree = HorizontalVelocity * Time_PhaseThree_Hor - 0.5 * HorizontalAcceleration * pow(Time_PhaseThree_Hor,2);
                        }
                    }
                    VerticalWay_PhaseTwo = VerticalDistance - VerticalWay_PhaseOne - VerticalWay_PhaseThree;
                    HorizontalWay_PhaseTwo = HorizontalDistance - HorizontalWay_PhaseOne - HorizontalWay_PhaseThree;
                    Time_PhaseTwo_Vert = abs(VerticalWay_PhaseTwo / VerticalVelocity);
                    Time_PhaseTwo_Hor = HorizontalWay_PhaseTwo / HorizontalVelocity;
                    Time_Vert = Time_PhaseOne_Vert + Time_PhaseTwo_Vert + Time_PhaseThree_Vert;
                    Time_Hor = Time_PhaseOne_Hor + Time_PhaseTwo_Hor + Time_PhaseThree_Hor;

                    /** Check, which is longer and recalculate shorter one **/
                    if (Time_Hor > Time_Vert)
                    {
                        Time_Vert = Time_Hor;
                        if (configXML["Segment@"+num2Str(Number)+"/VerticalSpeed"] > 0)
                        {
                            /**Annahme: positive Geschwindigkeiten im aktuellen Segment **/

                            if(configXML["Segment@"+num2Str(Number+1)+"/VerticalSpeed"] > 0)
                            {
                                configXML["Segment@"+num2Str(Number)+"/VerticalAcceleration"] = configXML["Segment@"+num2Str(Number+1)+"/VerticalSpeed"] / Time_Vert;
                            }
                            else
                            {
                                /** Reisefluggeschwindigkeit wird neu berechnet, Geschwindigkeit von 0 auf 0 **/
                                configXML["Segment@"+num2Str(Number)+"/VerticalSpeed"] = calculateSlopeVelocity(0,0,VerticalAcceleration,Time_Vert,VerticalDistance);
                                VerticalVelocity = configXML["Segment@"+num2Str(Number)+"/VerticalSpeed"];
                                Time_PhaseOne_Vert = VerticalVelocity / VerticalAcceleration;
                                Way_PhaseOne_Vert = 0.5 * VerticalAcceleration * pow(Time_PhaseOne_Vert,2);
                                Time_PhaseThree_Vert = Time_PhaseOne_Vert;
                                Way_PhaseThree_Vert = Way_PhaseOne_Vert;
                                Way_PhaseTwo_Vert = VerticalDistance - Way_PhaseOne_Vert - Way_PhaseThree_Vert;
                                Time_PhaseTwo_Vert = Way_PhaseTwo_Vert / VerticalVelocity;
                            }
                        }
                        else
                        {
                            /**Annahme: negative Geschwindigkeiten im aktuellen Segment **/

                            if(configXML["Segment@"+num2Str(Number+1)+"/VerticalSpeed"] > 0)
                            {
                                /** Reisefluggeschwindigkeit wird neu berechnet, Geschwindigkeit von 0 auf 0 **/
                                configXML["Segment@"+num2Str(Number)+"/VerticalSpeed"] = (-1) * calculateSlopeVelocity(0,0,VerticalAcceleration,Time_Vert,VerticalDistance);
                                VerticalVelocity = configXML["Segment@"+num2Str(Number)+"/VerticalSpeed"];
                                Time_PhaseOne_Vert = abs(VerticalVelocity / VerticalAcceleration);
                                Way_PhaseOne_Vert = abs(0.5 * VerticalAcceleration * pow(Time_PhaseOne_Vert,2));
                                Time_PhaseThree_Vert = Time_PhaseOne_Vert;
                                Way_PhaseThree_Vert = Way_PhaseOne_Vert;
                                Way_PhaseTwo_Vert = VerticalDistance - Way_PhaseOne_Vert - Way_PhaseThree_Vert;
                                Time_PhaseTwo_Vert = abs(Way_PhaseTwo_Vert / VerticalVelocity);
                            }
                            else
                            {
                                configXML["Segment@"+num2Str(Number)+"/VerticalAcceleration"] = abs(configXML["Segment@"+num2Str(Number+1)+"/VerticalSpeed"] / Time_Vert);
                            }
                        }

                    }
                    else if (Time_Hor < Time_Vert)
                    {
                        Time_Hor = Time_Vert;
                        if(configXML["Segment@"+num2Str(Number+1)+"/HorizontalSpeed"] > 0)
                        {
                            configXML["Segment@"+num2Str(Number)+"/HorizontalAcceleration"] = configXML["Segment@"+num2Str(Number+1)+"/HorizontalSpeed"] / Time_Hor;
                        }
                        else
                        {
                            /** Reisefluggeschwindigkeit wird neu berechnet, Geschwindigkeit von 0 auf 0 **/
                            configXML["Segment@"+num2Str(Number)+"/HorizontalSpeed"] = calculateSlopeVelocity(0,0,HorizontalAcceleration,Time_Hor,HorizontalDistance);
                            HorizontalVelocity = configXML["Segment@"+num2Str(Number)+"/HorizontalSpeed"];
                            Time_PhaseOne_Hor = HorizontalVelocity / HorizontalAcceleration;
                            Way_PhaseOne_Hor = 0.5 * HorizontalAcceleration * pow(Time_PhaseOne_Hor,2);
                            Time_PhaseThree_Hor = Time_PhaseOne_Hor;
                            Way_PhaseThree_Hor = Way_PhaseOne_Hor;
                            Way_PhaseTwo_Hor = HorizontalDistance - Way_PhaseOne_Hor - Way_PhaseThree_Hor;
                            Time_PhaseTwo_Hor = Way_PhaseTwo_Hor / HorizontalVelocity;
                        }
                    }
                    Time = Time_Hor;
                }
                else
                {

                    /** Wenn Steigsegment mit gleichem Vorzeichen in der vertikalen Geschwindigkeit vorher war, muss anfangs nicht beschleunigt werden. **/

                    /** Calculate horizontal and vertical independently, then check which takes longer.
                        The shorter one will be set to new, matching acceleration **/
                    Time_PhaseOne_Hor = HorizontalVelocity / HorizontalAcceleration;
                    HorizontalWay_PhaseOne = 0.5 * HorizontalAcceleration * pow(Time_PhaseOne_Hor,2);

                    if ((configXML["Segment@"+num2Str(Number)+"/VerticalSpeed"] > 0 && configXML["Segment@"+num2Str(Number+1)+"/VerticalSpeed"] > 0) || (configXML["Segment@"+num2Str(Number)+"/VerticalSpeed"] < 0 && configXML["Segment@"+num2Str(Number+1)+"/VerticalSpeed"] < 0))
                    {
                        /** Wenn die aktuelle Vertikalgeschwindigkeit und die darauffolgende das gleiche Vorzeichen haben, muss nicht auf 0 abgebremst werden **/

                        Time_PhaseThree_Vert = abs((VerticalVelocity - configXML["Segment@"+num2Str(Number+1)+"/VerticalSpeed"]) / VerticalAcceleration);
                        Time_PhaseThree_Hor = abs((HorizontalVelocity - configXML["Segment@"+num2Str(Number+1)+"/HorizontalSpeed"]) / HorizontalAcceleration);
                        if (abs(VerticalVelocity) < abs(configXML["Segment@"+num2Str(Number+1)+"/VerticalSpeed"]))
                        {
                            VerticalWay_PhaseThree = abs(VerticalVelocity * Time_PhaseThree_Vert) + 0.5 * VerticalAcceleration * pow(Time_PhaseThree_Vert,2);
                        }
                        else
                        {
                            VerticalWay_PhaseThree = abs(VerticalVelocity * Time_PhaseThree_Vert) - 0.5 * VerticalAcceleration * pow(Time_PhaseThree_Vert,2);
                        }
                        if (HorizontalVelocity < configXML["Segment@"+num2Str(Number+1)+"/HorizontalSpeed"])
                        {
                            HorizontalWay_PhaseThree = HorizontalVelocity * Time_PhaseThree_Hor + 0.5 * HorizontalAcceleration * pow(Time_PhaseThree_Hor,2);
                        }
                        else
                        {
                            HorizontalWay_PhaseThree = HorizontalVelocity * Time_PhaseThree_Hor - 0.5 * HorizontalAcceleration * pow(Time_PhaseThree_Hor,2);
                        }
                    }
                    else
                    {
                        /** Wenn die aktuelle Vertikalgeschwindigkeit und die darauffolgende unterschiedliche Vorzeichen haben, muss auf 0 abgebremst werden **/

                        Time_PhaseThree_Vert = abs(VerticalVelocity / VerticalAcceleration);
                        Time_PhaseThree_Hor = abs((HorizontalVelocity - configXML["Segment@"+num2Str(Number+1)+"/HorizontalSpeed"]) / HorizontalAcceleration);
                        VerticalWay_PhaseThree = abs(VerticalVelocity * Time_PhaseThree_Vert) - 0.5 * VerticalAcceleration * pow(Time_PhaseThree_Vert,2);

                        if (HorizontalVelocity < configXML["Segment@"+num2Str(Number+1)+"/HorizontalSpeed"])
                        {
                            HorizontalWay_PhaseThree = HorizontalVelocity * Time_PhaseThree_Hor + 0.5 * HorizontalAcceleration * pow(Time_PhaseThree_Hor,2);
                        }
                        else
                        {
                            HorizontalWay_PhaseThree = HorizontalVelocity * Time_PhaseThree_Hor - 0.5 * HorizontalAcceleration * pow(Time_PhaseThree_Hor,2);
                        }
                    }
                    VerticalWay_PhaseTwo = VerticalDistance - VerticalWay_PhaseThree;
                    HorizontalWay_PhaseTwo = HorizontalDistance - HorizontalWay_PhaseOne - HorizontalWay_PhaseThree;
                    Time_PhaseTwo_Vert = abs(VerticalWay_PhaseTwo / VerticalVelocity);
                    Time_PhaseTwo_Hor = HorizontalWay_PhaseTwo / HorizontalVelocity;
                    Time_Vert = Time_PhaseTwo_Vert + Time_PhaseThree_Vert;
                    Time_Hor = Time_PhaseOne_Hor + Time_PhaseTwo_Hor + Time_PhaseThree_Hor;

                    /** Check, which is longer and recalculate shorter one **/
                    if (Time_Hor > Time_Vert)
                    {
                        Time_Vert = Time_Hor;
                        if (configXML["Segment@"+num2Str(Number)+"/VerticalSpeed"] > 0)
                        {
                            /**Annahme: positive Geschwindigkeiten im aktuellen Segment **/

                            if(configXML["Segment@"+num2Str(Number+1)+"/VerticalSpeed"] > 0)
                            {
                                configXML["Segment@"+num2Str(Number)+"/VerticalAcceleration"] = abs(configXML["Segment@"+num2Str(Number+1)+"/VerticalSpeed"] - configXML["Segment@"+num2Str(Number)+"/VerticalSpeed"]) / Time_Vert;
                            }
                            else
                            {
                                /** Reisefluggeschwindigkeit wird neu berechnet, Geschwindigkeit von aktueller Geschwindigkeit auf 0 **/
                                configXML["Segment@"+num2Str(Number)+"/VerticalSpeed"] = calculateSlopeVelocity(VerticalVelocity,0,VerticalAcceleration,Time_Vert,VerticalDistance);
                                VerticalVelocity = configXML["Segment@"+num2Str(Number)+"/VerticalSpeed"];
                                Time_PhaseOne_Vert = VerticalVelocity / VerticalAcceleration;
                                Way_PhaseOne_Vert = 0.5 * VerticalAcceleration * pow(Time_PhaseOne_Vert,2);
                                Time_PhaseThree_Vert = Time_PhaseOne_Vert;
                                Way_PhaseThree_Vert = Way_PhaseOne_Vert;
                                Way_PhaseTwo_Vert = VerticalDistance - Way_PhaseOne_Vert - Way_PhaseThree_Vert;
                                Time_PhaseTwo_Vert = Way_PhaseTwo_Vert / VerticalVelocity;
                            }
                        }
                        else
                        {
                            /**Annahme: negative Geschwindigkeiten im aktuellen Segment **/

                            if(configXML["Segment@"+num2Str(Number+1)+"/VerticalSpeed"] > 0)
                            {
                                /** Reisefluggeschwindigkeit wird neu berechnet, Geschwindigkeit von 0 auf 0 **/
                                configXML["Segment@"+num2Str(Number)+"/VerticalSpeed"] = (-1) * calculateSlopeVelocity(VerticalVelocity,0,VerticalAcceleration,Time_Vert,VerticalDistance);
                                VerticalVelocity = configXML["Segment@"+num2Str(Number)+"/VerticalSpeed"];
                                Time_PhaseOne_Vert = abs(VerticalVelocity / VerticalAcceleration);
                                Way_PhaseOne_Vert = abs(0.5 * VerticalAcceleration * pow(Time_PhaseOne_Vert,2));
                                Time_PhaseThree_Vert = Time_PhaseOne_Vert;
                                Way_PhaseThree_Vert = Way_PhaseOne_Vert;
                                Way_PhaseTwo_Vert = VerticalDistance - Way_PhaseOne_Vert - Way_PhaseThree_Vert;
                                Time_PhaseTwo_Vert = abs(Way_PhaseTwo_Vert / VerticalVelocity);
                            }
                            else
                            {
                                configXML["Segment@"+num2Str(Number)+"/VerticalAcceleration"] = abs(configXML["Segment@"+num2Str(Number+1)+"/VerticalSpeed"] - configXML["Segment@"+num2Str(Number)+"/VerticalSpeed"] / Time_Vert);
                            }
                        }

                    }
                    else if (Time_Hor < Time_Vert)
                    {
                        Time_Hor = Time_Vert;
                        if(configXML["Segment@"+num2Str(Number+1)+"/HorizontalSpeed"] > 0)
                        {
                            configXML["Segment@"+num2Str(Number)+"/HorizontalAcceleration"] = configXML["Segment@"+num2Str(Number+1)+"/HorizontalSpeed"] / Time_Hor;
                        }
                        else
                        {
                            /** Reisefluggeschwindigkeit wird neu berechnet, Geschwindigkeit von 0 auf 0 **/
                            configXML["Segment@"+num2Str(Number)+"/HorizontalSpeed"] = calculateSlopeVelocity(0,0,HorizontalAcceleration,Time_Hor,HorizontalDistance);
                            HorizontalVelocity = configXML["Segment@"+num2Str(Number)+"/HorizontalSpeed"];
                            Time_PhaseOne_Hor = HorizontalVelocity / HorizontalAcceleration;
                            Way_PhaseOne_Hor = 0.5 * HorizontalAcceleration * pow(Time_PhaseOne_Hor,2);
                            Time_PhaseThree_Hor = Time_PhaseOne_Hor;
                            Way_PhaseThree_Hor = Way_PhaseOne_Hor;
                            Way_PhaseTwo_Hor = HorizontalDistance - Way_PhaseOne_Hor - Way_PhaseThree_Hor;
                            Time_PhaseTwo_Hor = Way_PhaseTwo_Hor / HorizontalVelocity;
                        }
                    }
                    Time = Time_Hor;
                }
            }

            else
            {
                if ((configXML["Segment@"+num2Str(Number-1)+"/VerticalSpeed"] > 0 && configXML["Segment@"+num2Str(Number)+"/VerticalSpeed"] < 0) || (configXML["Segment@"+num2Str(Number-1)+"/VerticalSpeed"] < 0 && configXML["Segment@"+num2Str(Number)+"/VerticalSpeed"] > 0))
                {
                    /** Wenn schräges Segment mit gleichem Vorzeichen der Vertikalgeschwindigkeit vorher war,
                    muss anfangs in horizontale Richtung und in vertikale Richtung nicht beschleunigt werden **/

                /** Calculate horizontal and vertical independently, then check which takes longer.
                    The shorter one will be set to new, matching acceleration **/

                if ((configXML["Segment@"+num2Str(Number)+"/VerticalSpeed"] > 0 && configXML["Segment@"+num2Str(Number+1)+"/VerticalSpeed"] > 0) || (configXML["Segment@"+num2Str(Number)+"/VerticalSpeed"] < 0 && configXML["Segment@"+num2Str(Number+1)+"/VerticalSpeed"] < 0))
                {
                    /** Wenn die aktuelle Vertikalgeschwindigkeit und die darauffolgende das gleiche Vorzeichen haben, muss nicht auf 0 abgebremst werden **/

                    Time_PhaseThree_Vert = abs((VerticalVelocity - configXML["Segment@"+num2Str(Number+1)+"/VerticalSpeed"]) / VerticalAcceleration);
                    Time_PhaseThree_Hor = abs((HorizontalVelocity - configXML["Segment@"+num2Str(Number+1)+"/HorizontalSpeed"]) / HorizontalAcceleration);
                    if (abs(VerticalVelocity) < abs(configXML["Segment@"+num2Str(Number+1)+"/VerticalSpeed"]))
                    {
                        VerticalWay_PhaseThree = abs(VerticalVelocity * Time_PhaseThree_Vert) + 0.5 * VerticalAcceleration * pow(Time_PhaseThree_Vert,2);
                    }
                    else
                    {
                        VerticalWay_PhaseThree = abs(VerticalVelocity * Time_PhaseThree_Vert) - 0.5 * VerticalAcceleration * pow(Time_PhaseThree_Vert,2);
                    }
                    if (HorizontalVelocity < configXML["Segment@"+num2Str(Number+1)+"/HorizontalSpeed"])
                    {
                        HorizontalWay_PhaseThree = HorizontalVelocity * Time_PhaseThree_Hor + 0.5 * HorizontalAcceleration * pow(Time_PhaseThree_Hor,2);
                    }
                    else
                    {
                        HorizontalWay_PhaseThree = HorizontalVelocity * Time_PhaseThree_Hor - 0.5 * HorizontalAcceleration * pow(Time_PhaseThree_Hor,2);
                    }
                }
                else
                {
                    /** Wenn die aktuelle Vertikalgeschwindigkeit und die darauffolgende unterschiedliche Vorzeichen haben, muss auf 0 abgebremst werden **/

                    Time_PhaseThree_Vert = abs(VerticalVelocity / VerticalAcceleration);
                    Time_PhaseThree_Hor = abs((HorizontalVelocity - configXML["Segment@"+num2Str(Number+1)+"/HorizontalSpeed"]) / HorizontalAcceleration);
                    VerticalWay_PhaseThree = abs(VerticalVelocity * Time_PhaseThree_Vert) - 0.5 * VerticalAcceleration * pow(Time_PhaseThree_Vert,2);

                    if (HorizontalVelocity < configXML["Segment@"+num2Str(Number+1)+"/HorizontalSpeed"])
                    {
                        HorizontalWay_PhaseThree = HorizontalVelocity * Time_PhaseThree_Hor + 0.5 * HorizontalAcceleration * pow(Time_PhaseThree_Hor,2);
                    }
                    else
                    {
                        HorizontalWay_PhaseThree = HorizontalVelocity * Time_PhaseThree_Hor - 0.5 * HorizontalAcceleration * pow(Time_PhaseThree_Hor,2);
                    }
                }
                VerticalWay_PhaseTwo = VerticalDistance - VerticalWay_PhaseThree;
                HorizontalWay_PhaseTwo = HorizontalDistance - HorizontalWay_PhaseThree;
                Time_PhaseTwo_Vert = abs(VerticalWay_PhaseTwo / VerticalVelocity);
                Time_PhaseTwo_Hor = HorizontalWay_PhaseTwo / HorizontalVelocity;
                Time_Vert = Time_PhaseTwo_Vert + Time_PhaseThree_Vert;
                Time_Hor = Time_PhaseTwo_Hor + Time_PhaseThree_Hor;

                /** Check, which is longer and recalculate shorter one **/
                if (Time_Hor > Time_Vert)
                {
                    Time_Vert = Time_Hor;
                    if (configXML["Segment@"+num2Str(Number)+"/VerticalSpeed"] > 0)
                    {
                        /**Annahme: positive Geschwindigkeiten im aktuellen Segment **/

                        if(configXML["Segment@"+num2Str(Number+1)+"/VerticalSpeed"] > 0)
                        {
                            configXML["Segment@"+num2Str(Number)+"/VerticalAcceleration"] = abs(configXML["Segment@"+num2Str(Number+1)+"/VerticalSpeed"] - configXML["Segment@"+num2Str(Number)+"/VerticalSpeed"]) / Time_Vert;
                        }
                        else
                        {
                            /** Reisefluggeschwindigkeit wird neu berechnet, Geschwindigkeit von 0 auf 0 **/
                            configXML["Segment@"+num2Str(Number)+"/VerticalSpeed"] = calculateSlopeVelocity(VerticalVelocity,0,VerticalAcceleration,Time_Vert,VerticalDistance);
                            VerticalVelocity = configXML["Segment@"+num2Str(Number)+"/VerticalSpeed"];
                            Time_PhaseOne_Vert = VerticalVelocity / VerticalAcceleration;
                            Way_PhaseOne_Vert = 0.5 * VerticalAcceleration * pow(Time_PhaseOne_Vert,2);
                            Time_PhaseThree_Vert = Time_PhaseOne_Vert;
                            Way_PhaseThree_Vert = Way_PhaseOne_Vert;
                            Way_PhaseTwo_Vert = VerticalDistance - Way_PhaseOne_Vert - Way_PhaseThree_Vert;
                            Time_PhaseTwo_Vert = Way_PhaseTwo_Vert / VerticalVelocity;
                        }
                    }
                    else
                    {
                        /**Annahme: negative Geschwindigkeiten im aktuellen Segment **/

                        if(configXML["Segment@"+num2Str(Number+1)+"/VerticalSpeed"] > 0)
                        {
                            /** Reisefluggeschwindigkeit wird neu berechnet, Geschwindigkeit von 0 auf 0 **/
                            configXML["Segment@"+num2Str(Number)+"/VerticalSpeed"] = (-1) * calculateSlopeVelocity(VerticalVelocity,0,VerticalAcceleration,Time_Vert,VerticalDistance);
                            VerticalVelocity = configXML["Segment@"+num2Str(Number)+"/VerticalSpeed"];
                            Time_PhaseOne_Vert = abs(VerticalVelocity / VerticalAcceleration);
                            Way_PhaseOne_Vert = abs(0.5 * VerticalAcceleration * pow(Time_PhaseOne_Vert,2));
                            Time_PhaseThree_Vert = Time_PhaseOne_Vert;
                            Way_PhaseThree_Vert = Way_PhaseOne_Vert;
                            Way_PhaseTwo_Vert = VerticalDistance - Way_PhaseOne_Vert - Way_PhaseThree_Vert;
                            Time_PhaseTwo_Vert = abs(Way_PhaseTwo_Vert / VerticalVelocity);
                        }
                        else
                        {
                            configXML["Segment@"+num2Str(Number)+"/VerticalAcceleration"] = abs(configXML["Segment@"+num2Str(Number+1)+"/VerticalSpeed"] - configXML["Segment@"+num2Str(Number)+"/VerticalSpeed"]) / Time_Vert;
                        }
                    }

                }
                else if (Time_Hor < Time_Vert)
                {
                    Time_Hor = Time_Vert;
                    if(configXML["Segment@"+num2Str(Number+1)+"/HorizontalSpeed"] > 0)
                    {
                        configXML["Segment@"+num2Str(Number)+"/HorizontalAcceleration"] = abs(configXML["Segment@"+num2Str(Number+1)+"/HorizontalSpeed"] - configXML["Segment@"+num2Str(Number)+"/HorizontalSpeed"])/ Time_Hor;
                    }
                    else
                    {
                        /** Reisefluggeschwindigkeit wird neu berechnet, Geschwindigkeit von aktueller Geschwindigkeit auf 0 **/
                        configXML["Segment@"+num2Str(Number)+"/HorizontalSpeed"] = calculateSlopeVelocity(HorizontalVelocity,0,HorizontalAcceleration,Time_Hor,HorizontalDistance);
                        HorizontalVelocity = configXML["Segment@"+num2Str(Number)+"/HorizontalSpeed"];
                        Time_PhaseOne_Hor = HorizontalVelocity / HorizontalAcceleration;
                        Way_PhaseOne_Hor = 0.5 * HorizontalAcceleration * pow(Time_PhaseOne_Hor,2);
                        Time_PhaseThree_Hor = Time_PhaseOne_Hor;
                        Way_PhaseThree_Hor = Way_PhaseOne_Hor;
                        Way_PhaseTwo_Hor = HorizontalDistance - Way_PhaseOne_Hor - Way_PhaseThree_Hor;
                        Time_PhaseTwo_Hor = Way_PhaseTwo_Hor / HorizontalVelocity;
                    }
                }
                Time = Time_Hor;
                }
                else
                {
                    /** Wenn schräges Segment mit unterschiedlichem Vorzeichen in vertikaler Geschwindigkeit vorher war,
                    muss anfangs in horizontale Richtung nicht beschleunigt werden, aber in vertikale Richtung. **/

                    /** Calculate horizontal and vertical independently, then check which takes longer.
                        The shorter one will be set to new, matching acceleration **/
                    Time_PhaseOne_Vert = abs(VerticalVelocity / VerticalAcceleration);
                    VerticalWay_PhaseOne = 0.5 * VerticalAcceleration * pow(Time_PhaseOne_Vert,2);

                    if ((configXML["Segment@"+num2Str(Number)+"/VerticalSpeed"] > 0 && configXML["Segment@"+num2Str(Number+1)+"/VerticalSpeed"] > 0) || (configXML["Segment@"+num2Str(Number)+"/VerticalSpeed"] < 0 && configXML["Segment@"+num2Str(Number+1)+"/VerticalSpeed"] < 0))
                    {
                        /** Wenn die aktuelle Vertikalgeschwindigkeit und die darauffolgende das gleiche Vorzeichen haben, muss nicht auf 0 abgebremst werden **/

                        Time_PhaseThree_Vert = abs((VerticalVelocity - configXML["Segment@"+num2Str(Number+1)+"/VerticalSpeed"]) / VerticalAcceleration);
                        Time_PhaseThree_Hor = abs((HorizontalVelocity - configXML["Segment@"+num2Str(Number+1)+"/HorizontalSpeed"]) / HorizontalAcceleration);
                        if (abs(VerticalVelocity) < abs(configXML["Segment@"+num2Str(Number+1)+"/VerticalSpeed"]))
                        {
                            VerticalWay_PhaseThree = abs(VerticalVelocity * Time_PhaseThree_Vert) + 0.5 * VerticalAcceleration * pow(Time_PhaseThree_Vert,2);
                        }
                        else
                        {
                            VerticalWay_PhaseThree = abs(VerticalVelocity * Time_PhaseThree_Vert) - 0.5 * VerticalAcceleration * pow(Time_PhaseThree_Vert,2);
                        }
                        if (HorizontalVelocity < configXML["Segment@"+num2Str(Number+1)+"/HorizontalSpeed"])
                        {
                            HorizontalWay_PhaseThree = HorizontalVelocity * Time_PhaseThree_Hor + 0.5 * HorizontalAcceleration * pow(Time_PhaseThree_Hor,2);
                        }
                        else
                        {
                            HorizontalWay_PhaseThree = HorizontalVelocity * Time_PhaseThree_Hor - 0.5 * HorizontalAcceleration * pow(Time_PhaseThree_Hor,2);
                        }
                    }
                    else
                    {
                        /** Wenn die aktuelle Vertikalgeschwindigkeit und die darauffolgende unterschiedliche Vorzeichen haben, muss auf 0 abgebremst werden **/

                        Time_PhaseThree_Vert = abs(VerticalVelocity / VerticalAcceleration);
                        Time_PhaseThree_Hor = abs((HorizontalVelocity - configXML["Segment@"+num2Str(Number+1)+"/HorizontalSpeed"]) / HorizontalAcceleration);
                        VerticalWay_PhaseThree = abs(VerticalVelocity * Time_PhaseThree_Vert) - 0.5 * VerticalAcceleration * pow(Time_PhaseThree_Vert,2);

                        if (HorizontalVelocity < configXML["Segment@"+num2Str(Number+1)+"/HorizontalSpeed"])
                        {
                            HorizontalWay_PhaseThree = HorizontalVelocity * Time_PhaseThree_Hor + 0.5 * HorizontalAcceleration * pow(Time_PhaseThree_Hor,2);
                        }
                        else
                        {
                            HorizontalWay_PhaseThree = HorizontalVelocity * Time_PhaseThree_Hor - 0.5 * HorizontalAcceleration * pow(Time_PhaseThree_Hor,2);
                        }
                    }
                    VerticalWay_PhaseTwo = VerticalDistance - VerticalWay_PhaseOne - VerticalWay_PhaseThree;
                    HorizontalWay_PhaseTwo = HorizontalDistance - HorizontalWay_PhaseThree;
                    Time_PhaseTwo_Vert = abs(VerticalWay_PhaseTwo / VerticalVelocity);
                    Time_PhaseTwo_Hor = HorizontalWay_PhaseTwo / HorizontalVelocity;
                    Time_Vert = Time_PhaseOne_Vert + Time_PhaseTwo_Vert + Time_PhaseThree_Vert;
                    Time_Hor = Time_PhaseTwo_Hor + Time_PhaseThree_Hor;

                    /** Check, which is longer and recalculate shorter one **/
                    if (Time_Hor > Time_Vert)
                    {
                        Time_Vert = Time_Hor;
                        if (configXML["Segment@"+num2Str(Number)+"/VerticalSpeed"] > 0)
                        {
                            /**Annahme: positive Geschwindigkeiten im aktuellen Segment **/

                            if(configXML["Segment@"+num2Str(Number+1)+"/VerticalSpeed"] > 0)
                            {
                                configXML["Segment@"+num2Str(Number)+"/VerticalAcceleration"] = configXML["Segment@"+num2Str(Number+1)+"/VerticalSpeed"] / Time_Vert;
                            }
                            else
                            {
                                /** Reisefluggeschwindigkeit wird neu berechnet, Geschwindigkeit von 0 auf 0 **/
                                configXML["Segment@"+num2Str(Number)+"/VerticalSpeed"] = calculateSlopeVelocity(0,0,VerticalAcceleration,Time_Vert,VerticalDistance);
                                VerticalVelocity = configXML["Segment@"+num2Str(Number)+"/VerticalSpeed"];
                                Time_PhaseOne_Vert = VerticalVelocity / VerticalAcceleration;
                                Way_PhaseOne_Vert = 0.5 * VerticalAcceleration * pow(Time_PhaseOne_Vert,2);
                                Time_PhaseThree_Vert = Time_PhaseOne_Vert;
                                Way_PhaseThree_Vert = Way_PhaseOne_Vert;
                                Way_PhaseTwo_Vert = VerticalDistance - Way_PhaseOne_Vert - Way_PhaseThree_Vert;
                                Time_PhaseTwo_Vert = Way_PhaseTwo_Vert / VerticalVelocity;
                            }
                        }
                        else
                        {
                            /**Annahme: negative Geschwindigkeiten im aktuellen Segment **/

                            if(configXML["Segment@"+num2Str(Number+1)+"/VerticalSpeed"] > 0)
                            {
                                /** Reisefluggeschwindigkeit wird neu berechnet, Geschwindigkeit von 0 auf 0 **/
                                configXML["Segment@"+num2Str(Number)+"/VerticalSpeed"] = (-1) * calculateSlopeVelocity(0,0,VerticalAcceleration,Time_Vert,VerticalDistance);
                                VerticalVelocity = configXML["Segment@"+num2Str(Number)+"/VerticalSpeed"];
                                Time_PhaseOne_Vert = abs(VerticalVelocity / VerticalAcceleration);
                                Way_PhaseOne_Vert = abs(0.5 * VerticalAcceleration * pow(Time_PhaseOne_Vert,2));
                                Time_PhaseThree_Vert = Time_PhaseOne_Vert;
                                Way_PhaseThree_Vert = Way_PhaseOne_Vert;
                                Way_PhaseTwo_Vert = VerticalDistance - Way_PhaseOne_Vert - Way_PhaseThree_Vert;
                                Time_PhaseTwo_Vert = abs(Way_PhaseTwo_Vert / VerticalVelocity);
                            }
                            else
                            {
                                configXML["Segment@"+num2Str(Number)+"/VerticalAcceleration"] = abs(configXML["Segment@"+num2Str(Number+1)+"/VerticalSpeed"] / Time_Vert);
                            }
                        }

                    }
                    else if (Time_Hor < Time_Vert)
                    {
                        Time_Hor = Time_Vert;
                        if(configXML["Segment@"+num2Str(Number+1)+"/HorizontalSpeed"] > 0)
                        {
                            configXML["Segment@"+num2Str(Number)+"/HorizontalAcceleration"] = abs(configXML["Segment@"+num2Str(Number+1)+"/HorizontalSpeed"] - configXML["Segment@"+num2Str(Number)+"/HorizontalSpeed"])/ Time_Hor;
                        }
                        else
                        {
                            /** Reisefluggeschwindigkeit wird neu berechnet, Geschwindigkeit von aktueller Geschwindigkeit auf 0 **/
                            configXML["Segment@"+num2Str(Number)+"/HorizontalSpeed"] = calculateSlopeVelocity(HorizontalVelocity,0,HorizontalAcceleration,Time_Hor,HorizontalDistance);
                            HorizontalVelocity = configXML["Segment@"+num2Str(Number)+"/HorizontalSpeed"];
                            Time_PhaseOne_Hor = HorizontalVelocity / HorizontalAcceleration;
                            Way_PhaseOne_Hor = 0.5 * HorizontalAcceleration * pow(Time_PhaseOne_Hor,2);
                            Time_PhaseThree_Hor = Time_PhaseOne_Hor;
                            Way_PhaseThree_Hor = Way_PhaseOne_Hor;
                            Way_PhaseTwo_Hor = HorizontalDistance - Way_PhaseOne_Hor - Way_PhaseThree_Hor;
                            Time_PhaseTwo_Hor = Way_PhaseTwo_Hor / HorizontalVelocity;
                        }
                    }
                    Time = Time_Hor;
                }
            }
        }
    }

    /** IF: Last Segment **/
    if (Number == configXML["NumberOfSegments"])
    {
        if (Type == "Vertical")
        {
            if ((configXML["Segment@"+num2Str(Number-1)+"/HorizontalDistance"] == 0 && configXML["Segment@"+num2Str(Number-1)+"/VerticalDistance"] == 0) || (configXML["Segment@"+num2Str(Number-1)+"/HorizontalDistance"] != 0 && configXML["Segment@"+num2Str(Number-1)+"/VerticalDistance"] == 0) || (configXML["Segment@"+num2Str(Number-1)+"/HorizontalDistance"] == 0 && configXML["Segment@"+num2Str(Number-1)+"/VerticalDistance"] != 0 && configXML["Segment@"+num2Str(Number-1)+"/VerticalSpeed"] > 0) || (configXML["Segment@"+num2Str(Number-1)+"/HorizontalDistance"] != 0 && configXML["Segment@"+num2Str(Number-1)+"/VerticalDistance"] != 0 && configXML["Segment@"+num2Str(Number-1)+"/VerticalSpeed"] > 0))
            {
                /** Wenn vor dem vertikalen Sinksegment ein Schwebesegment, ein horizontales Segment oder ein vertikales bzw. schräges Steigsegment war, dann muss zunächst beschleunigt werden **/
                Time_PhaseThree = abs(VerticalVelocity / VerticalAcceleration);
                VerticalWay_PhaseThree = abs(VerticalVelocity * Time_PhaseThree) - 0.5 * VerticalAcceleration * pow(Time_PhaseThree,2);
                Time_PhaseOne = abs(VerticalVelocity / VerticalAcceleration);
                VerticalWay_PhaseOne = 0.5 * VerticalAcceleration * pow(Time_PhaseOne,2);
                VerticalWay_PhaseTwo = VerticalDistance - VerticalWay_PhaseOne - VerticalWay_PhaseThree;
                Time_PhaseTwo = abs(VerticalWay_PhaseTwo / VerticalVelocity);
                Time = Time_PhaseOne + Time_PhaseTwo + Time_PhaseThree;
            }
            else
            {
                    /** Wenn vor dem vertikalen Sinksegment ein vertikales Sinksegment oder ein schräges Sinksegment war, dann muss nicht mehr beschleunigt werden **/
                    Time_PhaseThree = abs(VerticalVelocity / VerticalAcceleration);
                    VerticalWay_PhaseThree = abs(VerticalVelocity * Time_PhaseThree) - 0.5 * VerticalAcceleration * pow(Time_PhaseThree,2);
                    VerticalWay_PhaseTwo = VerticalDistance - VerticalWay_PhaseThree;
                    Time_PhaseTwo = abs(VerticalWay_PhaseTwo / VerticalVelocity);
                    Time = Time_PhaseTwo + Time_PhaseThree;
            }
        }
        else if (Type == "Aslope")
        {
            if ((configXML["Segment@"+num2Str(Number-1)+"/HorizontalDistance"] == 0 && configXML["Segment@"+num2Str(Number-1)+"/VerticalDistance"] == 0) || (configXML["Segment@"+num2Str(Number-1)+"/HorizontalDistance"] == 0 && configXML["Segment@"+num2Str(Number-1)+"/VerticalDistance"] != 0 && configXML["Segment@"+num2Str(Number-1)+"/VerticalSpeed"] > 0))
            {
                /** Wenn vor dem schrägen Segment ein Schwebesegment oder ein vertikales Steigsegment war, dann muss zunächst beschleunigt werden **/

                /** Calculate horizontal and vertical independently, then check which takes longer.
                The shorter one will be set to new, matching acceleration **/
                Time_PhaseThree_Vert = abs(VerticalVelocity / VerticalAcceleration);
                Time_PhaseThree_Hor = HorizontalVelocity / HorizontalAcceleration;
                VerticalWay_PhaseThree = abs(VerticalVelocity * Time_PhaseThree_Vert) - 0.5 * VerticalAcceleration * pow(Time_PhaseThree_Vert,2);
                HorizontalWay_PhaseThree = HorizontalVelocity * Time_PhaseThree_Hor - 0.5 * HorizontalAcceleration * pow(Time_PhaseThree_Hor,2);

                Time_PhaseOne_Vert = abs(VerticalVelocity / VerticalAcceleration);
                Time_PhaseOne_Hor = HorizontalVelocity / HorizontalAcceleration;
                VerticalWay_PhaseOne = 0.5 * VerticalAcceleration * pow(Time_PhaseOne_Vert,2);
                HorizontalWay_PhaseOne = 0.5 * HorizontalAcceleration * pow(Time_PhaseOne_Hor,2);

                VerticalWay_PhaseTwo = VerticalDistance - VerticalWay_PhaseOne - VerticalWay_PhaseThree;
                HorizontalWay_PhaseTwo = HorizontalDistance - HorizontalWay_PhaseOne - HorizontalWay_PhaseThree;
                Time_PhaseTwo_Vert = abs(VerticalWay_PhaseTwo / VerticalVelocity);
                Time_PhaseTwo_Hor = HorizontalWay_PhaseTwo / HorizontalVelocity;
                Time_Vert = Time_PhaseOne_Vert + Time_PhaseTwo_Vert + Time_PhaseThree_Vert;
                Time_Hor = Time_PhaseOne_Hor + Time_PhaseTwo_Hor + Time_PhaseThree_Hor;

                /** Check, which is longer and recalculate shorter one **/
                if (Time_Hor > Time_Vert)
                {
                    Time_Vert = Time_Hor;
                    /** War Geschwindigkeit vorher positiv oder null, dann muss zu Anfang des Segments beschleunigt werden **/
                    if(configXML["Segment@"+num2Str(Number-1)+"/VerticalSpeed"] >= 0)
                    {
                        /** Reisefluggeschwindigkeit wird neu berechnet, Geschwindigkeit von 0 auf 0 **/
                        configXML["Segment@"+num2Str(Number)+"/VerticalSpeed"] = (-1) * calculateSlopeVelocity(0,0,VerticalAcceleration,Time_Vert,VerticalDistance);
                        VerticalVelocity = configXML["Segment@"+num2Str(Number)+"/VerticalSpeed"];
                        Time_PhaseOne_Vert = abs(VerticalVelocity / VerticalAcceleration);
                        Way_PhaseOne_Vert = abs(0.5 * VerticalAcceleration * pow(Time_PhaseOne_Vert,2));
                        Time_PhaseThree_Vert = Time_PhaseOne_Vert;
                        Way_PhaseThree_Vert = Way_PhaseOne_Vert;
                        Way_PhaseTwo_Vert = VerticalDistance - Way_PhaseOne_Vert - Way_PhaseThree_Vert;
                        Time_PhaseTwo_Vert = abs(Way_PhaseTwo_Vert / VerticalVelocity);
                    }
                    else
                    {
                        /** War Geschwindigkeit vorher negativ, dann kann konstant auf 0 gebremst werden **/
                        configXML["Segment@"+num2Str(Number)+"/VerticalAcceleration"] = abs(configXML["Segment@"+num2Str(Number)+"/VerticalSpeed"] / Time_Vert);
                    }
                }
                else if (Time_Hor < Time_Vert)
                {
                    Time_Hor = Time_Vert;
                    if(configXML["Segment@"+num2Str(Number-1)+"/HorizontalSpeed"] > 0)
                    {
                        /** War Geschwindigkeit vorher positiv, dann kann konstant auf 0 gebremst werden **/
                        configXML["Segment@"+num2Str(Number)+"/HorizontalAcceleration"] = configXML["Segment@"+num2Str(Number)+"/HorizontalSpeed"] / Time_Hor;
                    }
                    else
                    {
                        /** War Geschwindigkeit vorher 0 (negativ nicht sinnvoll), dann muss zunächst beschleunigt werden und anschließend auf 0 abgebremst werden **/
                        configXML["Segment@"+num2Str(Number)+"/HorizontalSpeed"] = calculateSlopeVelocity(0,0,HorizontalAcceleration,Time_Hor,HorizontalDistance);
                        HorizontalVelocity = configXML["Segment@"+num2Str(Number)+"/HorizontalSpeed"];
                        Time_PhaseOne_Hor = HorizontalVelocity / HorizontalAcceleration;
                        Way_PhaseOne_Hor = 0.5 * HorizontalAcceleration * pow(Time_PhaseOne_Hor,2);
                        Time_PhaseThree_Hor = Time_PhaseOne_Hor;
                        Way_PhaseThree_Hor = Way_PhaseOne_Hor;
                        Way_PhaseTwo_Hor = HorizontalDistance - Way_PhaseOne_Hor - Way_PhaseThree_Hor;
                        Time_PhaseTwo_Hor = Way_PhaseTwo_Hor / HorizontalVelocity;
                    }
                }
                Time = Time_Hor;
            }
            else if ((configXML["Segment@"+num2Str(Number-1)+"/HorizontalDistance"] != 0 && configXML["Segment@"+num2Str(Number-1)+"/VerticalDistance"] == 0) || (configXML["Segment@"+num2Str(Number-1)+"/HorizontalDistance"] != 0 && configXML["Segment@"+num2Str(Number-1)+"/VerticalDistance"] != 0 && configXML["Segment@"+num2Str(Number-1)+"/VerticalSpeed"] > 0))
            {
                /** Wenn vor dem schrägen Segment ein horizontales Segment oder ein schräges Steigsegment war, dann muss horizontal nicht beschleunigt werden **/

                /** Calculate horizontal and vertical independently, then check which takes longer.
                The shorter one will be set to new, matching acceleration **/
                Time_PhaseThree_Vert = abs(VerticalVelocity / VerticalAcceleration);
                Time_PhaseThree_Hor = HorizontalVelocity / HorizontalAcceleration;
                VerticalWay_PhaseThree = abs(VerticalVelocity * Time_PhaseThree_Vert) - 0.5 * VerticalAcceleration * pow(Time_PhaseThree_Vert,2);
                HorizontalWay_PhaseThree = HorizontalVelocity * Time_PhaseThree_Hor - 0.5 * HorizontalAcceleration * pow(Time_PhaseThree_Hor,2);

                Time_PhaseOne_Vert = abs(VerticalVelocity / VerticalAcceleration);
                VerticalWay_PhaseOne = 0.5 * VerticalAcceleration * pow(Time_PhaseOne_Vert,2);

                VerticalWay_PhaseTwo = VerticalDistance - VerticalWay_PhaseOne - VerticalWay_PhaseThree;
                HorizontalWay_PhaseTwo = HorizontalDistance - HorizontalWay_PhaseThree;
                Time_PhaseTwo_Vert = abs(VerticalWay_PhaseTwo / VerticalVelocity);
                Time_PhaseTwo_Hor = HorizontalWay_PhaseTwo / HorizontalVelocity;
                Time_Vert = Time_PhaseOne_Vert + Time_PhaseTwo_Vert + Time_PhaseThree_Vert;
                Time_Hor = Time_PhaseTwo_Hor + Time_PhaseThree_Hor;

                /** Check, which is longer and recalculate shorter one **/
                if (Time_Hor > Time_Vert)
                {
                    Time_Vert = Time_Hor;
                    /** War Geschwindigkeit vorher positiv oder null, dann muss zu Anfang des Segments beschleunigt werden **/
                    if(configXML["Segment@"+num2Str(Number-1)+"/VerticalSpeed"] >= 0)
                    {
                        /** Reisefluggeschwindigkeit wird neu berechnet, Geschwindigkeit von 0 auf 0 **/
                        configXML["Segment@"+num2Str(Number)+"/VerticalSpeed"] = (-1) * calculateSlopeVelocity(0,0,VerticalAcceleration,Time_Vert,VerticalDistance);
                        VerticalVelocity = configXML["Segment@"+num2Str(Number)+"/VerticalSpeed"];
                        Time_PhaseOne_Vert = abs(VerticalVelocity / VerticalAcceleration);
                        Way_PhaseOne_Vert = abs(0.5 * VerticalAcceleration * pow(Time_PhaseOne_Vert,2));
                        Time_PhaseThree_Vert = Time_PhaseOne_Vert;
                        Way_PhaseThree_Vert = Way_PhaseOne_Vert;
                        Way_PhaseTwo_Vert = VerticalDistance - Way_PhaseOne_Vert - Way_PhaseThree_Vert;
                        Time_PhaseTwo_Vert = abs(Way_PhaseTwo_Vert / VerticalVelocity);
                    }
                    else
                    {
                        /** War Geschwindigkeit vorher negativ, dann kann konstant auf 0 gebremst werden **/
                        configXML["Segment@"+num2Str(Number)+"/VerticalAcceleration"] = abs(configXML["Segment@"+num2Str(Number)+"/VerticalSpeed"] / Time_Vert);
                    }
                }
                else if (Time_Hor < Time_Vert)
                {
                    Time_Hor = Time_Vert;
                    if(configXML["Segment@"+num2Str(Number-1)+"/HorizontalSpeed"] > 0)
                    {
                        /** War Geschwindigkeit vorher positiv, dann kann konstant auf 0 gebremst werden **/
                        configXML["Segment@"+num2Str(Number)+"/HorizontalAcceleration"] = configXML["Segment@"+num2Str(Number)+"/HorizontalSpeed"] / Time_Hor;
                    }
                    else
                    {
                        /** War Geschwindigkeit vorher 0 (negativ nicht sinnvoll), dann muss zunächst beschleunigt werden und anschließend auf 0 abgebremst werden **/
                        configXML["Segment@"+num2Str(Number)+"/HorizontalSpeed"] = calculateSlopeVelocity(0,0,HorizontalAcceleration,Time_Hor,HorizontalDistance);
                        HorizontalVelocity = configXML["Segment@"+num2Str(Number)+"/HorizontalSpeed"];
                        Time_PhaseOne_Hor = HorizontalVelocity / HorizontalAcceleration;
                        Way_PhaseOne_Hor = 0.5 * HorizontalAcceleration * pow(Time_PhaseOne_Hor,2);
                        Time_PhaseThree_Hor = Time_PhaseOne_Hor;
                        Way_PhaseThree_Hor = Way_PhaseOne_Hor;
                        Way_PhaseTwo_Hor = HorizontalDistance - Way_PhaseOne_Hor - Way_PhaseThree_Hor;
                        Time_PhaseTwo_Hor = Way_PhaseTwo_Hor / HorizontalVelocity;
                    }
                }
                Time = Time_Hor;
            }
            else if (configXML["Segment@"+num2Str(Number-1)+"/HorizontalDistance"] == 0 && configXML["Segment@"+num2Str(Number-1)+"/VerticalDistance"] != 0 && configXML["Segment@"+num2Str(Number-1)+"/VerticalSpeed"] < 0)
            {
                /** Wenn vor dem schrägen Segment ein vertikales Sinksegment war, dann muss vertikal nicht beschleunigt werden **/

                /** Calculate horizontal and vertical independently, then check which takes longer.
                The shorter one will be set to new, matching acceleration **/
                Time_PhaseThree_Vert = abs(VerticalVelocity / VerticalAcceleration);
                Time_PhaseThree_Hor = HorizontalVelocity / HorizontalAcceleration;
                VerticalWay_PhaseThree = abs(VerticalVelocity * Time_PhaseThree_Vert) - 0.5 * VerticalAcceleration * pow(Time_PhaseThree_Vert,2);
                HorizontalWay_PhaseThree = HorizontalVelocity * Time_PhaseThree_Hor - 0.5 * HorizontalAcceleration * pow(Time_PhaseThree_Hor,2);

                Time_PhaseOne_Hor = HorizontalVelocity / HorizontalAcceleration;
                HorizontalWay_PhaseOne = 0.5 * HorizontalAcceleration * pow(Time_PhaseOne_Hor,2);

                VerticalWay_PhaseTwo = VerticalDistance - VerticalWay_PhaseThree;
                HorizontalWay_PhaseTwo = HorizontalDistance - HorizontalWay_PhaseOne - HorizontalWay_PhaseThree;
                Time_PhaseTwo_Vert = abs(VerticalWay_PhaseTwo / VerticalVelocity);
                Time_PhaseTwo_Hor = HorizontalWay_PhaseTwo / HorizontalVelocity;
                Time_Vert = Time_PhaseTwo_Vert + Time_PhaseThree_Vert;
                Time_Hor = Time_PhaseOne_Hor + Time_PhaseTwo_Hor + Time_PhaseThree_Hor;

                /** Check, which is longer and recalculate shorter one **/
                if (Time_Hor > Time_Vert)
                {
                    Time_Vert = Time_Hor;
                    /** War Geschwindigkeit vorher positiv oder null, dann muss zu Anfang des Segments beschleunigt werden **/
                    if(configXML["Segment@"+num2Str(Number-1)+"/VerticalSpeed"] >= 0)
                    {
                        /** Reisefluggeschwindigkeit wird neu berechnet, Geschwindigkeit von 0 auf 0 **/
                        configXML["Segment@"+num2Str(Number)+"/VerticalSpeed"] = (-1) * calculateSlopeVelocity(0,0,VerticalAcceleration,Time_Vert,VerticalDistance);
                        VerticalVelocity = configXML["Segment@"+num2Str(Number)+"/VerticalSpeed"];
                        Time_PhaseOne_Vert = abs(VerticalVelocity / VerticalAcceleration);
                        Way_PhaseOne_Vert = abs(0.5 * VerticalAcceleration * pow(Time_PhaseOne_Vert,2));
                        Time_PhaseThree_Vert = Time_PhaseOne_Vert;
                        Way_PhaseThree_Vert = Way_PhaseOne_Vert;
                        Way_PhaseTwo_Vert = VerticalDistance - Way_PhaseOne_Vert - Way_PhaseThree_Vert;
                        Time_PhaseTwo_Vert = abs(Way_PhaseTwo_Vert / VerticalVelocity);
                    }
                    else
                    {
                        /** War Geschwindigkeit vorher negativ, dann kann konstant auf 0 gebremst werden **/
                        configXML["Segment@"+num2Str(Number)+"/VerticalAcceleration"] = abs(configXML["Segment@"+num2Str(Number)+"/VerticalSpeed"] / Time_Vert);
                    }
                }
                else if (Time_Hor < Time_Vert)
                {
                    Time_Hor = Time_Vert;
                    if(configXML["Segment@"+num2Str(Number-1)+"/HorizontalSpeed"] > 0)
                    {
                        /** War Geschwindigkeit vorher positiv, dann kann konstant auf 0 gebremst werden **/
                        configXML["Segment@"+num2Str(Number)+"/HorizontalAcceleration"] = configXML["Segment@"+num2Str(Number)+"/HorizontalSpeed"] / Time_Hor;
                    }
                    else
                    {
                        /** War Geschwindigkeit vorher 0 (negativ nicht sinnvoll), dann muss zunächst beschleunigt werden und anschließend auf 0 abgebremst werden **/
                        configXML["Segment@"+num2Str(Number)+"/HorizontalSpeed"] = calculateSlopeVelocity(0,0,HorizontalAcceleration,Time_Hor,HorizontalDistance);
                        HorizontalVelocity = configXML["Segment@"+num2Str(Number)+"/HorizontalSpeed"];
                        Time_PhaseOne_Hor = HorizontalVelocity / HorizontalAcceleration;
                        Way_PhaseOne_Hor = 0.5 * HorizontalAcceleration * pow(Time_PhaseOne_Hor,2);
                        Time_PhaseThree_Hor = Time_PhaseOne_Hor;
                        Way_PhaseThree_Hor = Way_PhaseOne_Hor;
                        Way_PhaseTwo_Hor = HorizontalDistance - Way_PhaseOne_Hor - Way_PhaseThree_Hor;
                        Time_PhaseTwo_Hor = Way_PhaseTwo_Hor / HorizontalVelocity;
                    }
                }
                Time = Time_Hor;
            }
            else
            {
                /** Wenn vor dem schrägen Segment ein schräges Sinksegment war, dann muss nicht mehr beschleunigt werden **/

                /** Calculate horizontal and vertical independently, then check which takes longer.
                The shorter one will be set to new, matching acceleration **/
                Time_PhaseThree_Vert = abs(VerticalVelocity / VerticalAcceleration);
                Time_PhaseThree_Hor = HorizontalVelocity / HorizontalAcceleration;
                VerticalWay_PhaseThree = abs(VerticalVelocity * Time_PhaseThree_Vert) - 0.5 * VerticalAcceleration * pow(Time_PhaseThree_Vert,2);
                HorizontalWay_PhaseThree = HorizontalVelocity * Time_PhaseThree_Hor - 0.5 * HorizontalAcceleration * pow(Time_PhaseThree_Hor,2);

                VerticalWay_PhaseTwo = VerticalDistance - VerticalWay_PhaseThree;
                HorizontalWay_PhaseTwo = HorizontalDistance - HorizontalWay_PhaseThree;
                Time_PhaseTwo_Vert = abs(VerticalWay_PhaseTwo / VerticalVelocity);
                Time_PhaseTwo_Hor = HorizontalWay_PhaseTwo / HorizontalVelocity;
                Time_Vert = Time_PhaseTwo_Vert + Time_PhaseThree_Vert;
                Time_Hor = Time_PhaseTwo_Hor + Time_PhaseThree_Hor;

                /** Check, which is longer and recalculate shorter one **/
                if (Time_Hor > Time_Vert)
                {
                    Time_Vert = Time_Hor;
                    /** War Geschwindigkeit vorher positiv oder null, dann muss zu Anfang des Segments beschleunigt werden **/
                    if(configXML["Segment@"+num2Str(Number-1)+"/VerticalSpeed"] >= 0)
                    {
                        /** Reisefluggeschwindigkeit wird neu berechnet, Geschwindigkeit von 0 auf 0 **/
                        configXML["Segment@"+num2Str(Number)+"/VerticalSpeed"] = (-1) * calculateSlopeVelocity(0,0,VerticalAcceleration,Time_Vert,VerticalDistance);
                        VerticalVelocity = configXML["Segment@"+num2Str(Number)+"/VerticalSpeed"];
                        Time_PhaseOne_Vert = abs(VerticalVelocity / VerticalAcceleration);
                        Way_PhaseOne_Vert = abs(0.5 * VerticalAcceleration * pow(Time_PhaseOne_Vert,2));
                        Time_PhaseThree_Vert = Time_PhaseOne_Vert;
                        Way_PhaseThree_Vert = Way_PhaseOne_Vert;
                        Way_PhaseTwo_Vert = VerticalDistance - Way_PhaseOne_Vert - Way_PhaseThree_Vert;
                        Time_PhaseTwo_Vert = abs(Way_PhaseTwo_Vert / VerticalVelocity);
                    }
                    else
                    {
                        /** War Geschwindigkeit vorher negativ, dann kann konstant auf 0 gebremst werden **/
                        configXML["Segment@"+num2Str(Number)+"/VerticalAcceleration"] = abs(configXML["Segment@"+num2Str(Number)+"/VerticalSpeed"] / Time_Vert);
                    }
                }
                else if (Time_Hor < Time_Vert)
                {
                    Time_Hor = Time_Vert;
                    if(configXML["Segment@"+num2Str(Number-1)+"/HorizontalSpeed"] > 0)
                    {
                        /** War Geschwindigkeit vorher positiv, dann kann konstant auf 0 gebremst werden **/
                        configXML["Segment@"+num2Str(Number)+"/HorizontalAcceleration"] = configXML["Segment@"+num2Str(Number)+"/HorizontalSpeed"] / Time_Hor;
                    }
                    else
                    {
                        /** War Geschwindigkeit vorher 0 (negativ nicht sinnvoll), dann muss zunächst beschleunigt werden und anschließend auf 0 abgebremst werden **/
                        configXML["Segment@"+num2Str(Number)+"/HorizontalSpeed"] = calculateSlopeVelocity(0,0,HorizontalAcceleration,Time_Hor,HorizontalDistance);
                        HorizontalVelocity = configXML["Segment@"+num2Str(Number)+"/HorizontalSpeed"];
                        Time_PhaseOne_Hor = HorizontalVelocity / HorizontalAcceleration;
                        Way_PhaseOne_Hor = 0.5 * HorizontalAcceleration * pow(Time_PhaseOne_Hor,2);
                        Time_PhaseThree_Hor = Time_PhaseOne_Hor;
                        Way_PhaseThree_Hor = Way_PhaseOne_Hor;
                        Way_PhaseTwo_Hor = HorizontalDistance - Way_PhaseOne_Hor - Way_PhaseThree_Hor;
                        Time_PhaseTwo_Hor = Way_PhaseTwo_Hor / HorizontalVelocity;
                    }
                }
                Time = Time_Hor;
            }
        }
        else
        {
            myRuntimeInfo->err << "Last Segment is not suitable. Please adjust the settings!" << endl;
        }
    }
}

Segment::~Segment()
{
    //dtor
}
