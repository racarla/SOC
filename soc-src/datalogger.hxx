
#ifndef DATALOGGER_HXX_
#define DATALOGGER_HXX_

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

#include "global-defs.hxx"
#include "navigation.hxx"
#include "airdata.hxx"
#include "structs.hxx"
#include "missionMgr.hxx"
#include "exciteMgr.hxx"
#include "cntrlMgr.hxx"

class Datalogger {
  public:
    Datalogger();
    void LogData(FmuData fmuData, AirdataLog airdataLog, NavLog navLog, MissMgrLog missMgrLog, ExciteMgrLog exciteMgrLog, CntrlMgrLog cntrlMgrLog);
  private:
    FILE *LogFile_;
    bool FileExists(const std::string &FileName);
};

#endif
