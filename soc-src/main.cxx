/*
main.cxx

See: LICENSE.md for Copyright and License Agreement

*/

#include <iostream>
#include <iomanip>      // std::setw
#include <time.h>

#define kTIC2SEC 1/CLOCKS_PER_SEC; // Convert CPU tics to seconds
#define kTIC2MS 1000/CLOCKS_PER_SEC; // Convert CPU tics to milli-seconds

#define kMaxCntrlCmd 4 // Controller Dimension
#define kMaxCntrlEff 7 // Effectors
#define kMaxAllocObj 3 // Allocator Objectives
#define kMaxAllocEff 6 // Allocator Effectors
#define kMaxExciteChan kMaxCntrlCmd // Excitation Channels
#define kMaxExciteElem 46 // Excitation Elements (Multisine components)

#define kVerboseConfig 1

#include "datalogger.hxx"
#include "config.hxx"
#include "fmu.hxx"
#include "hardware-defs.hxx"

#include "MissionMgr.hxx"
#include "CntrlMgr.hxx"

#define EIGEN_INITIALIZE_MATRICES_BY_ZERO 1
//#define EIGEN_INITIALIZE_MATRICES_BY_NAN 1

//#define EIGEN_MAX_ALIGN_BYTES 0
//#define EIGEN_MAX_STATIC_ALIGN_BYTES 0


int main(int argc, char* argv[]) {
  if (argc!=2) {
    std::cerr << "ERROR: Incorrect number of input arguments." << std::endl;
    return -1;
  }

  // Seutp FMU
  Fmu fmu; // Class
  FmuData fmuData; // Struct
//std::cout << "Fmu" << "\t";

  // Setup Datalogger
  Datalogger log;

  // Load configuration file
  AircraftConfig configData; // Structure
  LoadConfigFile(argv[1], fmu, &configData, &fmuData);

  // SETUP MISSION
  // Define Mission Manager
  MissMgr missMgr; // Create System Class
  missMgr.Config(&configData); // Configure System
  MissMgrOut missMgrOut; // Create the Data Structure

    // Waveform Generation and Excitation Injection Systems are Components
    WaveSys waveSys; // Create System Class
    waveSys.Config(&configData); // Configure System
    WaveSysOut waveSysOut; // Create the Data Structure

    // Define Excitation System and Excitations
    ExciteSys exciteSys; // Create System Class
    exciteSys.Config(&configData); // Configure System
    ExciteSysOut exciteSysOut; // Create the Data Structure

  // Define Controller Manager
  CtrlMgr ctrlMgr; // Create System Class
  ctrlMgr.Config(&configData); // Configure System
  CtrlMgrOut ctrlMgrOut; // Create the Data Structure

    // Setup Input Processing
    InputProc inputProc; // Create System Class
    inputProc.Config(&configData); // Configure System
    InputProcOut inputProcOut; // Create the Data Structure

    // Setup Airdata
    Airdata airdata; // Create System Class
    airdata.Config(&configData); // Configure System
    AirdataOut airdataOut; // Create the Data Structure

    // Navigation Estimation Filter
    NavEst navEst; // Create System Class
    navEst.Config(&configData); // Configure System
    NavEstOut navEstOut; // Create the Data Structure

    // Define Guidance Systems
    GuidSys guidSys; // Create System Class
    guidSys.Config(&configData); // Configure System
    GuidSysOut guidSysOut; // Create the Data Structure

    // Define Scas Systems
    ScasSys scasSys; // Create System Class
    scasSys.Config(&configData); // Configure System
    ScasSysOut scasSysOut; // Create the Data Structure

    // Define Control Allocation Systems
    AllocSys allocSys; // Create System Class
    allocSys.Config(&configData); // Configure System
    AllocSysOut allocSysOut; // Create the Data Structure

  // Major Frame Loop
  while (1) {

    // Attempt to Read the FMU, return 1 on success
    if (fmu.GetSensorData(fmuData)) { // Run the major frame

      // Input Processing (Input, Nav, Airdata), Baseline Systems Only
      ctrlOut = ctrlMgr.BaseInput(missMgrOut, fmuData);

      // Misison Manager Mode Switching - Check safety triggers
      missMgrOut = missMgr.ModeMgr(ctrlOut);

      // Run Control Systems (Guidance, SCAS, Allocation), Baseline Systems Only
      ctrlOut = ctrlMgr.BaseCtrl(missMgrOut, fmuData);

      // Run Waveform Generation
      missMgrOut = missMgr.WaveGen();

      // Run Excitation System
      missMgrOut = missMgr.Excite(ctrlOut);

      // Input Processing (Input, Nav, Airdata), Experimental Systems
      ctrlOut = ctrlMgr.ExpInput(missMgrOut, fmuData);

      // Run Control Systems (Guidance, SCAS, Allocation), Experimental Systems
      ctrlOut = ctrlMgr.ExpCtrl(missMgrOut, fmuData);



      // OUTPUT PROCESSING
      // send control surface commands
      cntrlMgrOut.cmdEff.resize(7);
      std::vector<uint8_t> cmdEffSerial;
      cmdEffSerial.resize(cntrlMgrOut.cmdEff.size()*sizeof(float));

      cntrlMgrOut.cmdEff[0] = cntrlMgrOut.cmdCntrl[3]; // Throttle
      cntrlMgrOut.cmdEff[1] = cntrlMgrOut.cmdAlloc[0] + exciteMgrOut.cmdExcite[1]; // Elevator
      cntrlMgrOut.cmdEff[2] = cntrlMgrOut.cmdAlloc[1] + exciteMgrOut.cmdExcite[2]; // Rudder
      cntrlMgrOut.cmdEff[3] = cntrlMgrOut.cmdAlloc[2] - exciteMgrOut.cmdExcite[0]; // Ail R
      cntrlMgrOut.cmdEff[4] = cntrlMgrOut.cmdAlloc[3] + fmuData.SbusRx[0].Inceptors[3]; // Flap R
      cntrlMgrOut.cmdEff[5] = cntrlMgrOut.cmdAlloc[4] + fmuData.SbusRx[0].Inceptors[3]; // Flap L
      cntrlMgrOut.cmdEff[6] = cntrlMgrOut.cmdAlloc[5] + exciteMgrOut.cmdExcite[0]; // Ail L


      memcpy(cmdEffSerial.data(), cntrlMgrOut.cmdEff.data(), cmdEffSerial.size());

      // Write the Effector Commands to the FMU
      fmu.WriteMessage(kEffectorAngleCmd, cmdEffSerial.size(), cmdEffSerial.data());

      // Log Data
      MissMgrLog missMgrLog = missMgr.Log(missMgrOut);
      AirdataLog airdataLog = airdata.Log(airdataOut);
      NavLog navLog = NavFilter.Log(navOut);
      CntrlMgrLog cntrlMgrLog = cntrlMgr.Log(cntrlMgrOut);
      ExciteMgrLog exciteMgrLog = exciteMgr.Log(exciteMgrOut);

      log.LogData(fmuData, airdataLog, navLog, missMgrLog, exciteMgrLog, cntrlMgrLog);

      // Send Telemetry

    } // FMU Read, Major frame
  } // While 1

	return 0;
} // main
