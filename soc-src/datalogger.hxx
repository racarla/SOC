
#ifndef DATALOGGER_HXX_
#define DATALOGGER_HXX_

#include "global-defs.hxx"
#include "airdata.hxx"
#include "structs.hxx"
#include "missionMgr.hxx"
#include "exciteMgr.hxx"
#include "cntrlMgr.hxx"
#include "cntrlAllocMgr.hxx"

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
    void LogData(FmuData fmuData, AirdataStruct airdataData, NavigationData NavData, MissMgrStruct missMgrData, ExciteMgrStruct exciteMgrData, CntrlMgrStruct cntrlMgrData, CntrlAllocStruct cntrlAllocData);
  private:
    FILE *LogFile_;
    bool FileExists(const std::string &FileName);
};

#endif
