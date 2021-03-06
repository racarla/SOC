
#ifndef DATALOGGER_HXX_
#define DATALOGGER_HXX_

#include "global-defs.hxx"

#include <stdio.h>
#include <fcntl.h>
#include <unistd.h>
#include <termios.h>
#include <stdint.h>
#include <fstream>
#include <iostream>
#include <vector>
#include <exception>
#include <stdexcept>

class Datalogger {
  public:
    Datalogger();
    void LogFmuData(FmuData FmuDataRef);
  private:
    FILE *LogFile_;
    bool FileExists(const std::string &FileName);
};

#endif
