#ifndef OUTPUT_H
#define OUTPUT_H

#include <string>
#include <fstream>
#include <iostream>
#include <vector>
#include <iomanip>
#include <windows.h>

#include "functions.h"
#include "runtimeInfo.h"
#include "node.h"

using namespace std;

/**
*  Abstrakte Klasse zur Erstellung der Programmausgaben und Verwaltung der Ausgabedaten
**/

class output
{

    protected:
        string mProgramName;
        vector<string> Output;/**< Enthält alle Einträge, die in die .log files geschreiben werden */

        node& acXML;
        node& configXML;

        double columnWidth;/**< Breite der Bilder im HTML-Report in Pixeln **/
        double columnSpace;/**< Spaltenabstand im HTML-Report in Pixeln **/

        void generateReportHeader(ofstream &report, string acftName); /** Header für den Report **/
        void generateReportFooter(ofstream &report); /** Footer für den Report */

        void generateInfoFiles();/**< Methode zum erzeugen der Info-Datei **/
        void generateGuiSettingsFile();/**< Methode zum Erzeugen der GUI-Settings-Datei **/
        void generatePlotScriptHeader(ofstream &script, string format, string delimiter, bool xtraLineStyles, bool doubleSize);/**<Schreibt standardisierten Header fuer plt-Skript **/
        void convertPlotEPS2png(string filename, bool isLandscape,int newSize=0,string plotDir="__DEFAULT_DIR");/**< Methode um Postscript-Plots in png-Grafiken umzuwandeln **/
        void convertPlotSvg2png(string filename, int outputSizeX,string plotDir="__DEFAULT_DIR");/**< Methode um SVG-Dateien in png-Grafigen umzuwandeln **/
        void convertPlotEPS2PDF(string filename,string plotDir="__DEFAULT_DIR");/**< Methode um EPS-Dateien in PDF-Grafiken umzuwandeln **/


    public:
        virtual void writeOutput()=0;/**< Buffer für die Outputs **/
        output(string programName, node& configXML, node& doc);
        virtual ~output();


};

extern vector<string> Input;/**< Enthält alle Einträge für die guiSettings.xml */

#endif // OUTPUT_H
