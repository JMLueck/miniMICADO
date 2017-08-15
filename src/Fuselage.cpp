#include "Fuselage.h"

Fuselage::Fuselage(node& configXML)
    :
    configXML(configXML)
{
    FuselageLength = sqrt(pow(configXML["PayloadLength"],2) + 18 * pow(configXML["PayloadWidth"],2)) / 2;
    FuselageWidth = sqrt(pow(configXML["PayloadLength"],2) + 18 * pow(configXML["PayloadWidth"],2)) / 6;
    FuselageWettedArea = 2 * PI * FuselageWidth * (FuselageWidth + pow(FuselageLength,2)/sqrt(pow(FuselageLength,2) - pow(FuselageWidth,2)) * asin(sqrt(pow(FuselageLength,2) - pow(FuselageWidth,2))/FuselageLength));
    //ctor
}

Fuselage::~Fuselage()
{
    //dtor
}
