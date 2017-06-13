#ifndef FUNCTIONS_H
#define FUNCTIONS_H

#include <iostream>
#include <sstream>
#include <stdio.h>
#include <conio.h>
#include <algorithm>
#include <windows.h>
#include <string>
#include <math.h>
#include <sys/stat.h>//fuer "stat"-funktion in Methode "fileExists"
#include <tchar.h>
#include <ctype.h>
#include "runtimeInfo.h"

using namespace std;

/** Funktionen **/
int handleChildProcess(string processName, string relExecDir);/**< Ausfuehren von Unterprozessen mit Rückgabe des exitValues **/
int handleChildProcessOtherDirectory(string processName, string relExecDir, string workingDir);
void deleteObsoleteFiles(const string &filename);/**< Löschen nicht benoetigter Dateien **/
void wildCardDeleteFiles(const string &filesPath, const string &filesDir);/**< Löschen mehrerer Dateien gleicher Dateiendung oder gleichen Namens **/
vector <string> listDirFiles(const string &filesDir, const string &fileName);/**< Listet alle Dateien mit vorgegebenem Namen aus Ordner in Vektor **/
void renameFiles(const string &oldname, const string &newname, const string &newfiletype, const bool &backup);/**< Umbenennen oder Verschieben von Dateien **/
void copyFiles(const string &from, const string &to,const bool renameOnCollision=false);/**< Zum Kopieren fuer Ordner und Dateien, Wildcards sind erlaubt, mehrere Dateien/Ordner müssen mit \0 getrennt werden,es kann mit übergebenem "true" eine automatische Umbenennung aktiviert werden**/
bool fileExists(const string &filename);/**< Existenz von Dateien oder Ordnern pruefen **/
string getFullPathString(const string &filename);/**< absoluten Pfad einer Datei / eines Ordners zurueckgeben **/
bool copyFolder(char r_szSrcPath[1024], char r_szDesPath[1024]);

vector<string> readFile(const string &filename);/**< Zum Einlesen einer Textdatei (z.B. csv, txt) **/
int findInFile(const string &filename, const string &searchWord); /**< gibt Zeile zurueck, in der der gesuchte String steht **/
bool compareStrings(const string& str1, const string& str2, bool exact);/**< 2 Strings auf (teilweise) Uebereinstimmung ueberpruefen **/
string replaceAll(string Wort, const string &altesZeichen, const string &neuesZeichen);/**< Methode um bestimmte Zeichen in strings zu ersetzen **/
string stringForTex(string theData);/**<Methode zum ersetzen der Sonderzeichen fuer Tex-Output **/
string win2lin(string Pfad); /**<Methode zum Umwandeln eines relativen DOS-Pfades in einen Linux Pfad **/
string relativePath(string Path, string basePath); /**<Umwandeln eines Pfades in einen relativen Pfad von basePath ausgehend **/
bool deleteDirectory(string dir, bool recycleBin = true); /**< Löscht Ordner und enthaltene Dateien rekursiv **/
double Rounding(double Zahl, int Stellen);/**< Funktion zum Runden von Double-Werten **/
double RoundUp(double Zahl, int Stellen);/**< Funktion zum Aufrunden von Double-Werten **/
double RoundDown(double Zahl, int Stellen);/**< Funktion zum Abrunden von Double-Werten **/
bool timeCheck(int jahrBegrenzung,int monatBegrenzung);/**<Testet ob Lizenz abgelaufen, Angabe für timeCheck mit zwei Stellen(jj,mm)**/
bool accuracyCheck(double value,double targetValue,double accuracy);/**<testet ob Wert in Intervall um Zielwert liegt**/
double linearInterp(double pos, double posA, double valA, double posB, double valB);


template <typename T>
string num2Str(T aNum)/**< Funktion zum Casten einer Zahl in einen String **/
{
    stringstream ss_tmp;
    ss_tmp << aNum;
    return ss_tmp.str();
};

template <typename T>
T str2Num(string aString)
{
    stringstream ss_tmp;
    ss_tmp << aString;
    T result = 0;
    ss_tmp >> result;
    return result;
}

__int64 getDirectorySize(string path); /**< Gibt Größe eines Ordners in Bytes zurück - rekursiv **/
bool deleteDirectory(string dir, bool recycleBin); /**< Löscht Ordner und enthaltene Dateien rekursiv **/
string size2Str(__int64 size); /**< Wandelt eine Dateigröße (in Bytes) in einen String mit Einheit (kB,MB,etc.) um **/
bool isMasterProcess(); /** Gibt "true" zurück wenn Master-Prozess im MPI Modus, ist immer true wenn nicht im MPI Modus **/
#ifdef USE_MPI
// Die folgenden Funktionen geben den Erfolg der Operation zurück
bool readValueFromFile( char const * const file, const string &key, string &value ); /** Liest einen Wert aus einer Key=Value formatierten Datei **/
bool writeValueToFile( char const * const file, const string &key, const string &value ); /** Schreibt ein Key=Value Paar in eine Datei (überschreibt evtl. vorhandenen Wert) **/
bool incrementValueInFile( char const * const file, const string &key, int &value, const int &inc ); /** Erhöht Value eines Key=Value Paares um inc, der zuvor gespeicherte Wert wird in value abgelegt **/
void tokenize(const string &str, vector<string>& tokens, const string &delimiters); /** Teilt str in mehrere Zeichenketten an einem der delimiters Zeichen **/
#endif

#endif // FUNCTIONS_H
