/**
  Name: atmosphaere.h
  Author: Stephan Zajac
  Date: 25.04.2007
  Description: Headerdatei zur Einbindung der International Standard Atmosphere (ISA)

  Quelle: Wikipedia, Skript Flugzeugbau I ILR RWTH Aachen

  Angepasst: 22.11.2007, Eckhard Anton
  Angepasst: 25.04.2013, Kristof Risse, Florian Schültke: Version 2.0

  Gültig bis 20km
**/

#ifndef ATMOSPHERE_H  // Verhindert die Mehrfach-Deklaration
#define ATMOSPHERE_H

// Allgemein:
#include <iostream>
#include <string>
#include <math.h>



// Eigenes Projekt:
//#include "xmlsupport.h"

using namespace std;

class atmosphere
{
    public:
        /** ISA-Standardwerte auf Meereshöhe: **/
        static const double rho_0_isa;
        static const double T_0_isa;//[K]
        static const double T_0_celsius;//[K]
        static const double p_0_isa;//[Pa]
        static const double T_trop;//Temperatur der Tropospause [K]

        /** Berechnete Werte **/
        double h_trop;//Hoehe der Tropospause [m]
        double delta_h;//Höhenkorrektur [m] für lokale Höhenmessereinstellung (in Deutschland unter 5000 ft sonst gilt ISA Standard und FL)
        double rho_0;//Dichte NN [kg/m^3]
        double T_0;//Temperatur NN [K]
        double p_0;//Druck NN [Pa]

        /** Anpassen der Standardatmosphäre mit lokalen Daten
        double T_ref [K]
        double p_ref [Pa]
        double h_ref [m]
        **/
        void setAtmosphere(double h_ref, double T_ref, double p_ref);

        /** Gibt die Temperatur in Abh. von der Referenztemperatur und der Höhe wieder: **/
        inline double getTemperature(const double& h);
        inline double getTemperatureISA(const double& h);//ISA Temperaturprofil für Instrumentenkorrektur
        inline double getDensity(const double& h);
        inline double getPressure(const double& h);
        inline double getPressureISA(const double& h);
        inline double getSpeedOfSound(const double& h);
        inline double getSpeedOfSoundISA(const double& h);//Schallgeschwindigkeit bei ISA-Bedingungen für Machzahl über TA
        inline double getViscosity(const double& h);//Dynamische Viskosität [kg/(m*s)]

        inline double temperatureRatio(const double& h);
        inline double densityRatio(const double& h);
        inline double pressureRatio(const double& h);

        double getFLatDensity(const double& rho);//Höhe in Metern aus Dichte bestimmen
        double getFLatPressureRatio(const double& pressureRatio);//Höhe in Metern aus Dichte bestimmen

        atmosphere();               //Default constructor
        //atmosphere(string name);    //Constructor: Dateiname: "name".xml
        virtual ~atmosphere();

    private:

        /** Allgemeine Konstanten: **/
        static const double GaskonstanteLuft;
        static const double kappa;
        static const double TemperaturGradient1;
        //static const double TemperaturGradient2;
        //static const double TemperaturGradient3;
        static const double my_0;//[kg/(m*s)] Referenzviskosität der Luft (dynamisch) für 273 K aus Fluent Doku
        static const double SutherlandConst;//[K]

        /**
            h1(km)      h2(km)     dT/dh (K/km)  Quelle: http://www.pdas.com/coesa.html
                0          11         -6.5
                11         20          0.0
                20         32          1.0
                32         47          2.8
                47         51          0.0
                51         71         -2.8
                71         84.852     -2.0
        **/
        static const double g_Erde;


};
/** Implementierung der Inline-Funktionen **/
inline double atmosphere::getTemperature(const double& h)
{
    return (h < this->h_trop) ? this->T_0+this->TemperaturGradient1*h : this->T_trop;
}
inline double atmosphere::getTemperatureISA(const double& h)
{
    return (h < this->h_trop) ? this->T_0_isa+this->TemperaturGradient1*h : this->T_trop;
}
inline double atmosphere::getDensity(const double& h)
{
    return (h < this->h_trop) ?
        this->rho_0*pow(1+(this->TemperaturGradient1*h/this->T_0),(-this->g_Erde/this->GaskonstanteLuft/this->TemperaturGradient1-1)) :
        this->rho_0*pow(this->T_trop/this->T_0,(-this->g_Erde/this->GaskonstanteLuft/this->TemperaturGradient1-1))*exp(-this->g_Erde/this->GaskonstanteLuft/this->T_trop*(h-this->h_trop));
}
inline double atmosphere::getPressure(const double& h)
{
    return (h < this->h_trop) ?
        this->p_0*pow(1+(this->TemperaturGradient1*h/this->T_0),(-this->g_Erde/this->GaskonstanteLuft/this->TemperaturGradient1)) :
        this->p_0*pow(this->T_trop/this->T_0,(-this->g_Erde/this->GaskonstanteLuft/this->TemperaturGradient1))*exp(-this->g_Erde/this->GaskonstanteLuft/this->T_trop*(h-this->h_trop));
}
inline double atmosphere::getPressureISA(const double& h)
{
    return (h < this->h_trop) ?
        this->p_0_isa*pow(1+(this->TemperaturGradient1*h/this->T_0_isa),(-this->g_Erde/this->GaskonstanteLuft/this->TemperaturGradient1)) :
        this->p_0_isa*pow(this->T_trop/this->T_0_isa,(-this->g_Erde/this->GaskonstanteLuft/this->TemperaturGradient1))*exp(-this->g_Erde/this->GaskonstanteLuft/this->T_trop*(h-this->h_trop));
}
inline double atmosphere::getSpeedOfSound(const double& h)
{
    return (h < this->h_trop) ?
        sqrt((this->T_0+(TemperaturGradient1*h))*this->kappa*this->GaskonstanteLuft) :
        sqrt(this->T_trop*this->kappa*this->GaskonstanteLuft);
}
inline double atmosphere::getSpeedOfSoundISA(const double& h)
{
    return (h < 11000.) ?
        sqrt((this->T_0_isa+(TemperaturGradient1*h))*this->kappa*this->GaskonstanteLuft) :
        sqrt(this->T_trop*this->kappa*this->GaskonstanteLuft);
}
inline double atmosphere::getViscosity(const double& h)
{
    double beta(this->my_0*(this->T_0_celsius+this->SutherlandConst)/pow(this->T_0_celsius,1.5));
    double T(this->T_0+this->TemperaturGradient1*h);

    return (h < this->h_trop) ?
        beta * pow(T,1.5)/(T+this->SutherlandConst) :
        beta * pow(T_trop,1.5)/(T_trop+this->SutherlandConst);
}
inline double atmosphere::temperatureRatio(const double& h)
{
    return (h < this->h_trop) ? 1+(TemperaturGradient1*h/this->T_0) : this->T_trop/this->T_0;
}
inline double atmosphere::densityRatio(const double& h)
{
    return (h < this->h_trop) ?
        pow(1+(this->TemperaturGradient1*h/this->T_0),(-this->g_Erde/this->GaskonstanteLuft/this->TemperaturGradient1-1)) :
        pow(this->T_trop/this->T_0,(-this->g_Erde/this->GaskonstanteLuft/this->TemperaturGradient1-1))*exp(-this->g_Erde/this->GaskonstanteLuft/this->T_trop*(h-this->h_trop));
}
inline double atmosphere::pressureRatio(const double& h)
{
    return (h < this->h_trop) ?
        pow(1+(this->TemperaturGradient1*h/this->T_0),(-this->g_Erde/this->GaskonstanteLuft/this->TemperaturGradient1)) :
        pow(this->T_trop/this->T_0,(-this->g_Erde/this->GaskonstanteLuft/this->TemperaturGradient1))*exp(-this->g_Erde/this->GaskonstanteLuft/this->T_trop*(h-this->h_trop));
}

#endif // ATMOSPHERE_H
