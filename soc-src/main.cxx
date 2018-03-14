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

#include "navigation.hxx"
#include "structs.hxx"
#include "airdata.hxx"
#include "datalogger.hxx"
#include "config.hxx"
#include "fmu.hxx"
#include "hardware-defs.hxx"
#include "global-defs.hxx"
#include "missionMgr.hxx"
#include "cntrlMgr.hxx"
#include "exciteMgr.hxx"

#define EIGEN_INITIALIZE_MATRICES_BY_ZERO 1
//#define EIGEN_INITIALIZE_MATRICES_BY_NAN 1

//#define EIGEN_MAX_ALIGN_BYTES 0
//#define EIGEN_MAX_STATIC_ALIGN_BYTES 0


int main(int argc, char* argv[]) {
  if (argc!=2) {
    std::cerr << "ERROR: Incorrect number of input arguments." << std::endl;
    return -1;
  }

  // FMU Sensors
  Fmu fmu; // Class
  FmuData fmuData; // Struct
//std::cout << "Fmu" << "\t";

  // Data Logger
  Datalogger log;

  // load configuration file
  AircraftConfig configData; // Structure
  LoadConfigFile(argv[1], fmu, &configData, &fmuData);

  // Airdata
  Airdata airdata; // Class
  airdata.Init(); // Init
  AirdataOut airdataOut; // Struct
//std::cout << "Airdata" << "\t";

  // Navigation Filter
  Navigation NavFilter; // Class
  NavOut navOut; // Struct
//std::cout << "Navigation" << "\t";

  // SETUP MISSION
  // Define Mission Manager
  MissMgr missMgr; // Create the Mission Manager
  missMgr.Init(); // Initialize mission manager
  MissMgrOut missMgrOut; // Create the Mission Manage Structure
//std::cout << "MissMgr" << "\t";

  // Define Controller Manager and Controllers
  CntrlMgr cntrlMgr; // Create the Controller Manager

  // Define Excitation Manager and Excitations
  ExciteMgr exciteMgr; // Create the Excitation Manager
  exciteMgr.Init();    // Initialize the Excitation Manager
  ExciteMgrOut exciteMgrOut; // Struct
//std::cout << "ExciteMgr" << "\t";

  // Define Control Allocation
  CntrlAllocDef cntrlAllocDef; // Structure for control allocation definition

  uint8_t numObj = 3;
  uint8_t numEff = 6;

  cntrlAllocDef.cntrlEff.conservativeResize(numObj, numEff); // Control effectiveness matrix
  cntrlAllocDef.uMin.conservativeResize(numEff); // Min effector commands
  cntrlAllocDef.uMax.conservativeResize(numEff); // Max effector commands
  cntrlAllocDef.uPref.conservativeResize(numEff); // Prefered effector commands
  cntrlAllocDef.wtObj.conservativeResize(numObj, numObj); // Objective weighting matrix
  cntrlAllocDef.wtEff.conservativeResize(numEff, numEff); // Effector weighting matrix

  // Control Allocation Definition
  // Surface Order - Elev, Rud, AilR, FlapR, FlapL, AilL
  // Objectives Order - Roll Rate, Pitch Rate, Yaw Rate
//  cntrlAllocDef.cntrlEff <<  0.0000, -5.0084, -78.235, -33.013,  33.013,  78.235,
//                            -133.69,  0.0047,  3.0002,  2.6238,  2.6238,  3.0002,
//                             0.0000, -82.041,  5.7521,  1.7941, -1.7941, -5.7521; // rad/s per deg

  // Objectives Order - Roll Rate, Pitch Rate, Yaw Rate
//  cntrlAllocDef.cntrlEff <<  0.0000, -5.0084, -78.235, -33.013,  33.013,  78.235,
//                            -133.69,  0.0000,  0.0000,  0.0000,  0.0000,  0.0000,
//                             0.0000, -82.041,  5.7521,  1.7941, -1.7941, -5.7521; // rad/s per deg

  // Objectives Order - Roll Rate, Pitch Rate, Yaw Rate
  cntrlAllocDef.cntrlEff <<  0.0000,  0.0000, -78.235,  0.0000,  0.0000,  78.235,
                            -133.69,  0.0000,  0.0000,  0.0000,  0.0000,  0.0000,
                             0.0000, -82.041,  0.0000,  0.0000,  0.0000,  0.0000; // rad/s per deg

  cntrlAllocDef.cntrlEff *= kD2R; // Convert to rad/s per rad

  cntrlAllocDef.uMin << -25.0*kD2R, -15.0*kD2R, -25.0*kD2R, -25.0*kD2R, -25.0*kD2R, -25.0*kD2R ;
  cntrlAllocDef.uMax <<  25.0*kD2R,  15.0*kD2R,  25.0*kD2R,  25.0*kD2R,  25.0*kD2R,  25.0*kD2R ;
  cntrlAllocDef.uPref << 0.0,        0.0,        0.0,        0.0,        0.0,        0.0;

  cntrlAllocDef.wtObj.setIdentity();
  cntrlAllocDef.wtEff.setIdentity();

  cntrlMgr.Init(cntrlAllocDef);   // Define the Baseline and Research Controller, and Allocator
  CntrlMgrOut cntrlMgrOut; // Struct
//std::cout << "CntrlMgr" << "\t";
//std::cout << "CntrlAllocMgr" << "\t";

  clock_t frameStart_tic; // Start the in-frame timer
  clock_t frameStartNav_tic, frameStartExcite_tic, frameStartCntrl; // Intermidiate in-frame timers

  /* main loop */
  while (1) {

    frameStart_tic = clock();

    // Attempt to Read the FMU, return 1 on success
    bool fmuReadSuccess_bool = fmu.GetSensorData(&fmuData);

    if (fmuReadSuccess_bool) { // Run the major frame
      missMgrOut.tDurSens_ms = ((float) (clock() - frameStart_tic)) * kTIC2MS; //

  //
  // std::cout << "Time: " << timeTemp_s
  //  << "\tSuccess: " << fmu.errStatus_.cntSuccess
  //  << "\tReadUnavail: " << fmu.errStatus_.cntUnavailErr
  //  << "\tReadErr: " << fmu.errStatus_.cntReadErr
  //  << "\tPayloadErr: " << fmu.errStatus_.cntPayloadErr
  //  << "\tMessageErr: " << fmu.errStatus_.cntMessageErr
  //  << "\tSizeErr: " << fmu.errStatus_.cntSizeErr
  //  << "\tParseErr: " << fmu.errStatus_.cntParseErr
  //  << "\tHeaderErr: " << fmu.errStatus_.cntHeaderErr
  //  << "\tChecksumErrErr: " << fmu.errStatus_.cntChecksumErr << std::endl;


      // Reset the error counters
      // fmu.errStatus_.cntSuccess = 0;
      // fmu.errStatus_.cntUnavailErr = 0;
      // fmu.errStatus_.cntReadErr = 0;
      // fmu.errStatus_.cntPayloadErr = 0;
      // fmu.errStatus_.cntMessageErr = 0;
      // fmu.errStatus_.cntSizeErr = 0;
      // fmu.errStatus_.cntParseErr = 0;
      // fmu.errStatus_.cntHeaderErr = 0;
      // fmu.errStatus_.cntChecksumErr = 0;

      // INPUT PROCESSING

      // MISSION MANAGER
      // Mode Switching
      missMgrOut = missMgr.ModeMgr(fmuData);
// std::cout << "missMgrOut" << "\t";

      // SENSOR PROCESSING
      // Airdata
      airdataOut = airdata.Compute(fmuData.Pitot[0]);
// std::cout << "Airdata" << "\t";

      // Compute the Airdata Biases during startup, 10 seconds @ 50 Hz
      if (missMgrOut.frame_cnt < (500)) {
        airdata.BiasEst();

        // Check that the Pressure Sensors are in a sensible range
        if ((fmuData.Pitot[0].Static.Pressure_Pa < 70000) || (fmuData.Pitot[0].Static.Pressure_Pa > 120000)) {
          std::cout << "Pitot Static Sensor out of range: " << fmuData.Pitot[0].Static.Pressure_Pa << std::endl;
        }

        // Check that the Pressure Sensors are in a sensible range
//        if ((fmuData.PressureTransducer[0].Pressure_Pa < 70000) || (fmuData.PressureTransducer[0].Pressure_Pa > 120000)) {
//          std::cout << "5-Hole Static Sensor out of range: " << fmuData.PressureTransducer[0].Pressure_Pa << std::endl;
//        }

      }

      // navigation filter
      frameStartNav_tic = clock();
      if (fmuData.Gps.size() > 0) {
        if (!NavFilter.Initialized) {
          NavFilter.InitializeNavigation(fmuData);
        } else {
          NavFilter.RunNavigation(fmuData,&navOut);
        }
      }
      missMgrOut.tDurNav_ms = ((float) (clock() - frameStartNav_tic)) * kTIC2MS;
// std::cout << "Stick: " << fmuData.SbusRx[0].Inceptors[0] << "\t";
// std::cout << "Rate: " << fmuData.Mpu9250.Gyro_rads[0] << "\t";
// std::cout << "Euler: " << navOut.Euler_rad[0] << "\t";

      // CONTROL LAWS
      // Execute inner-loop control law
      cntrlMgr.Mode(missMgrOut.cntrlMode); // Transfer the Mission Control mode to the Controller Manager

// std::cout << missMgrOut.time_s << "\t";
// std::cout << missMgrOut.frame_cnt << "\t";
// std::cout << missMgrOut.autoEngage << "  ";
// std::cout << missMgrOut.cntrlMode << "  ";
// std::cout << missMgrOut.testArm << "  ";
// std::cout << missMgrOut.testEngage << "  ";
// std::cout << (int) missMgrOut.indxTest << "\t";

// std::cout << std::setw(10);

// std::cout << fmuData.Pitot[0].Static.Pressure_Pa << "\t";
// std::cout << fmuData.Pitot[0].Diff.Pressure_Pa << "\t\t";

// std::cout << airdataOut.alt_m << "\t";
// std::cout << airdataOut.altFilt_m << "\t\t";
// std::cout << airdataOut.vIas_mps << "\t";
// std::cout << airdataOut.vIasFilt_mps << "\t\t";

// std::cout << fmuData.PressureTransducer[0].Pressure_Pa << "\t";
// std::cout << fmuData.PressureTransducer[1].Pressure_Pa << "\t";
// std::cout << fmuData.PressureTransducer[2].Pressure_Pa << "\t";
// std::cout << fmuData.PressureTransducer[3].Pressure_Pa << "\t";

      // Generate command excitations
      frameStartExcite_tic = clock();
      exciteMgrOut = exciteMgr.Compute(missMgrOut.testEngage, missMgrOut.indxTest, missMgrOut.time_s);
      missMgrOut.tDurExcite_ms = ((float) (clock() - frameStartExcite_tic)) * kTIC2MS;
//std::cout << exciteMgrOut.cmdExcite.transpose()/kD2R << "\t";

      // Run the controllers
      frameStartCntrl = clock();
      cntrlMgr.CmdCntrlBase(missMgrOut.time_s, fmuData, navOut, airdataOut);
      cntrlMgr.CmdCntrlRes(missMgrOut.time_s, fmuData, navOut, airdataOut);
      cntrlMgrOut = cntrlMgr.CmdCntrl();
// std::cout << cntrlMgrOut.cmdCntrlRes.transpose() << "\t";
//std::cout << cntrlMgrOut.cmdCntrl[0] << "\t\t";

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
//std::cout << cntrlMgrOut.cmdEff.transpose() << "\t\t";

      missMgrOut.tDurCntrl_ms = ((float) (clock() - frameStartCntrl)) * kTIC2MS;

      memcpy(cmdEffSerial.data(), cntrlMgrOut.cmdEff.data(), cmdEffSerial.size());

      // Write the Effector Commands to the FMU
      fmu.WriteMessage(kEffectorAngleCmd, cmdEffSerial.size(), cmdEffSerial.data());

      missMgrOut.tCmd_ms = ((float) (clock() - frameStart_tic)) * kTIC2MS;

      // DATA LOG
      MissMgrLog missMgrLog = missMgr.Log(missMgrOut);
      AirdataLog airdataLog = airdata.Log(airdataOut);
      NavLog navLog = NavFilter.Log(navOut);
      CntrlMgrLog cntrlMgrLog = cntrlMgr.Log(cntrlMgrOut);
      ExciteMgrLog exciteMgrLog = exciteMgr.Log(exciteMgrOut);

      log.LogData(fmuData, airdataLog, navLog, missMgrLog, exciteMgrLog, cntrlMgrLog);

      missMgrOut.tFrame_ms = ((float) (clock() - frameStart_tic)) * kTIC2MS;

// std::cout << "Time: " << "\t";
// std::cout << missMgrOut.tDurSens_ms << "\t";
// std::cout << missMgrOut.tDurNav_ms << "\t";
// std::cout << missMgrOut.tDurExcite_ms << "\t";
// std::cout << missMgrOut.tDurCntrl_ms << "\t";
// std::cout << missMgrOut.tCmd_ms << "\t";
// std::cout << missMgrOut.tFrame_ms << "\t";
      // telemetry

// std::cout << std::endl;
    } // FMU Read, Major frame
  } // While 1

	return 0;
} // main
