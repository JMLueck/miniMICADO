#ifndef RUNTIMEINFO_H
#define RUNTIMEINFO_H

#include <fstream>
#include <string>
#include <iomanip>
#include "node.h"

#include "time.h"

using namespace std;

class outStream
{
public:
    outStream(ostream& os1, ostream& os2, bool comments_on, bool logfile_on, bool cleanFirstLine = true);
    virtual ~outStream();

    template<class T>
    outStream& operator<<(const T& x)
    {
        if (newLine)
        {
            string timeNow;
            if(comments_on||logfile_on)
            {
                timeNow=getTimeString();
            }
          if (comments_on) os1 << timeNow;
          if (logfile_on) os2 << timeNow;
          newLine = false;
        }
        if (comments_on) os1 << x;
        if (logfile_on) os2 << x;
        return *this;
    }

    typedef outStream& (*outStreamManipulator)(outStream&);// function that takes a custom stream, and returns it
    outStream& operator<<(outStreamManipulator manip)// take in a function with the custom signature
    {
        return manip(*this);// call the function, and return it's value
    }
    typedef std::basic_ostream<char, std::char_traits<char> > CoutType;// this is the type of std::cout
    typedef CoutType& (*StandardEndLine)(CoutType&);// this is the function signature of std::endl
    outStream& operator<<(StandardEndLine manip)// define an operator<< to take in std::endl
    {
        // call the function, but we cannot return it's value
        if (comments_on) manip(os1);
        if (logfile_on) manip(os2);
        newLine = true;
        //*this << getTimeString();
        return *this;
    }
    string getTimeString();
protected:
    ostream& os1;
    ostream& os2;
    bool comments_on;
    bool logfile_on;
    bool newLine;
};

class errStream : public outStream
{
public:
    errStream(ostream& os1, ostream& os2, bool comments_on, bool logfile_on, bool cleanFirstLine = true)
    : outStream(os1,os2, comments_on , logfile_on, cleanFirstLine) {};
    template<class T>
    outStream& operator<<(const T& x)
    {
        if (newLine)
        {
          if (comments_on) os1 << getTimeString() << "ERROR: ";
          if (logfile_on) os2 << getTimeString() << "ERROR: ";
          newLine = false;
        }
        if (comments_on) os1 << x;
        if (logfile_on) os2 << x;
        return *this;
    }
};

class warnStream : public outStream
{
public:
    warnStream(ostream& os1, ostream& os2, bool comments_on, bool logfile_on, bool cleanFirstLine = true)
    : outStream(os1,os2,comments_on,logfile_on,cleanFirstLine) {};
    template<class T>
    outStream& operator<<(const T& x)
    {
        if (newLine)
        {
          if (comments_on) os1 << getTimeString() << "WARNING: ";
          if (logfile_on) os2 << getTimeString() << "WARNING: ";
          newLine = false;
        }
        if (comments_on) os1 << x;
        if (logfile_on) os2 << x;
        return *this;
    }
};

class runtimeInfo
{
protected:
    bool comments_on;
    bool logfile_on;
    string logFilename;
    string _programName;
public:
    ofstream logFile;

    runtimeInfo(node& configXML, const string _programName);
    virtual ~runtimeInfo();

    outStream out;
    errStream err;
    warnStream warn;
};

extern runtimeInfo *myRuntimeInfo;
#endif // RUNTIMEINFO_H
