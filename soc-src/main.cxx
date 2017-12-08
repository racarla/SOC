/*
main.cxx
Brian R Taylor
brian.taylor@bolderflight.com
2017-04-18
Copyright (c) 2017 Bolder Flight Systems
Permission is hereby granted, free of charge, to any person obtaining a copy of this software 
and associated documentation files (the "Software"), to deal in the Software without restriction, 
including without limitation the rights to use, copy, modify, merge, publish, distribute, 
sublicense, and/or sell copies of the Software, and to permit persons to whom the Software is 
furnished to do so, subject to the following conditions:
The above copyright notice and this permission notice shall be included in all copies or 
substantial portions of the Software.
THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING 
BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND 
NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, 
DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, 
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
*/



#include <iostream>
#include <iomanip>      // std::setw

#define kMaxAllocObj 3 // Allocator Objectives
#define kMaxAllocEff 6 // Allocator Effectors
#define kMaxCntrlCmd 4 // Controller Dimension
#define kMaxExciteChan 4 // Excitation Channels
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
#include "cntrlAllocMgr.hxx"
#include "exciteMgr.hxx"

#define EIGEN_INITIALIZE_MATRICES_BY_ZERO 1
//#define EIGEN_INITIALIZE_MATRICES_BY_NAN 1

#define EIGEN_MAX_ALIGN_BYTES 0
#define EIGEN_MAX_STATIC_ALIGN_BYTES 0


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
  cntrlMgr.Init();   // Define the Baseline and Research Controller
  CntrlMgrOut cntrlMgrOut; // Struct
//std::cout << "CntrlMgr" << "\t";

  // Define Excitation Manager and Excitations
  ExciteMgr exciteMgr; // Create the Excitation Manager
  exciteMgr.Init();    // Initialize the Excitation Manager
  ExciteMgrOut exciteMgrOut; // Struct
//std::cout << "ExciteMgr" << "\t";

  // Define Control Allocation
  CntrlAllocMgr cntrlAllocMgr; // Create the Control Allocator
  CntrlAllocDef cntrlAllocDef; // Structure for control allocation definition
  CntrlAllocOut cntrlAllocOut; // Structure for control allocation data

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
//  cntrlAllocDef.cntrlEff <<  0.0000,  5.0084, -78.235, -33.013,  33.013,  78.235,
//                            -133.69,  0.0047,  3.0002,  2.6238,  2.6238,  3.0002,
//                             0.0000,  82.041,  5.7521,  1.7941, -1.7941, -5.7521; // rad/s per deg

  // Objectives Order - Roll Rate, Pitch Rate, Yaw Rate
  cntrlAllocDef.cntrlEff <<  0.0000,  5.0084, -78.235, -33.013,  33.013,  78.235,
                            -133.69,  0.0000,  0.0000,  0.0000,  0.0000,  0.0000,
                             0.0000,  82.041,  5.7521,  1.7941, -1.7941, -5.7521; // rad/s per deg

  cntrlAllocDef.cntrlEff *= kD2R; // Convert to rad/s per rad

  cntrlAllocDef.uMin << -25.0*kD2R, -15.0*kD2R, -25.0*kD2R, -25.0*kD2R, -25.0*kD2R, -25.0*kD2R ;
  cntrlAllocDef.uMax <<  25.0*kD2R,  15.0*kD2R,  25.0*kD2R,  25.0*kD2R,  25.0*kD2R,  25.0*kD2R ;
  cntrlAllocDef.uPref << 0.0,        0.0,        0.0,        0.0,        0.0,        0.0;

  cntrlAllocDef.wtObj.setIdentity();
  cntrlAllocDef.wtEff.setIdentity();

  cntrlAllocMgr.Init(cntrlAllocDef);     // Initialize Control Allocator
//std::cout << "CntrlAllocMgr" << "\t";

  /* main loop */
  while (1) {
    if (fmu.GetSensorData(&fmuData)) {
      // INPUT PROCESSING

      // MISSION MANAGER
      // Mode Switching
      missMgrOut = missMgr.ModeMgr(fmuData);
//std::cout << "missMgrOut" << "\t";

      // SENSOR PROCESSING
      // Airdata
      airdataOut = airdata.Compute(fmuData.Pitot[0]);
//std::cout << "Airdata" << "\t";

      // Compute the Airdata Biases during startup, 10 seconds @ 50 Hz
      if (missMgrOut.frame_cnt < (500)) {
        airdata.BiasEst();
      }

      // navigation filter
      if (fmuData.Gps.size() > 0) {
        if (!NavFilter.Initialized) {
          NavFilter.InitializeNavigation(fmuData);
        } else {
          NavFilter.RunNavigation(fmuData,&navOut);
        }
      }

      // CONTROL LAWS
      // Execute inner-loop control law
      cntrlMgr.Mode(missMgrOut.cntrlMode); // Transfer the Mission Control mode to the Controller Manager

std::cout << missMgrOut.time_s << "\t";
std::cout << missMgrOut.frame_cnt << "\t";
std::cout << missMgrOut.autoEngage << "  ";
std::cout << missMgrOut.cntrlMode << "  ";
std::cout << missMgrOut.testArm << "  ";
std::cout << missMgrOut.testEngage << "  ";
std::cout << (int) missMgrOut.indxTest << "\t";

std::cout << std::setw(10);

std::cout << airdataOut.alt_m << "\t";
std::cout << airdataOut.altFilt_m << "\t\t";
std::cout << airdataOut.vIas_mps << "\t";
std::cout << airdataOut.vIasFilt_mps << "\t\t";

//std::cout << fmuData.PressureTransducer[0].Pressure_Pa << "\t";
//std::cout << fmuData.PressureTransducer[1].Pressure_Pa << "\t";
//std::cout << fmuData.PressureTransducer[2].Pressure_Pa << "\t";
//std::cout << fmuData.PressureTransducer[3].Pressure_Pa << "\t";

      VecCmd refVecBase(kMaxCntrlCmd);
      refVecBase[0] = fmuData.SbusRx[0].Inceptors[0];
      refVecBase[1] = fmuData.SbusRx[0].Inceptors[1];
      refVecBase[2] = fmuData.SbusRx[0].Inceptors[2];
      refVecBase[3] = fmuData.SbusRx[0].Inceptors[4];
//std::cout << refVecBase.transpose() << "\t\t";
//std::cout << refVecBase[0] << "\t\t";

      VecCmd refVecRes(kMaxCntrlCmd);
      refVecRes[0] = fmuData.SbusRx[0].Inceptors[0];
      refVecRes[1] = fmuData.SbusRx[0].Inceptors[1];
      refVecRes[2] = fmuData.SbusRx[0].Inceptors[2];
      refVecRes[3] = 17; // Command speed
//std::cout << refVecRes.transpose() << "\t\t";

      VecCmd measVec(kMaxCntrlCmd);
      measVec[0] = navOut.Euler_rad[0];
      measVec[1] = navOut.Euler_rad[1];
      measVec[2] = navOut.Euler_rad[2];
      measVec[3] = airdataOut.vIasFilt_mps;
//std::cout << measVec.transpose() << "\t";

      VecCmd dMeasVec(kMaxCntrlCmd);
      dMeasVec[0] = fmuData.Mpu9250.Gyro_rads[0];
      dMeasVec[1] = fmuData.Mpu9250.Gyro_rads[1];
      dMeasVec[2] = fmuData.Mpu9250.Gyro_rads[2];
      dMeasVec[3] = 0.0;
//std::cout << dMeasVec.transpose() << "\t";

      // Run the controllers
      cntrlMgr.CmdBase(refVecBase, missMgrOut.time_s);
      cntrlMgr.CmdRes(refVecRes, measVec, dMeasVec, missMgrOut.time_s);
      cntrlMgrOut = cntrlMgr.Cmd();
//std::cout << cntrlMgrOut.cmd.transpose() << "\t";
//std::cout << cntrlMgrOut.cmd[0] << "\t\t";

      // Allocate control surfaces
      VecObj objAlloc = cntrlMgrOut.cmd.head(kMaxAllocObj);
      cntrlAllocOut = cntrlAllocMgr.Compute(objAlloc);
//std::cout << cntrlAllocOut.cmdAlloc.transpose()/kD2R << "\t";

      // Apply command excitations
      exciteMgrOut = exciteMgr.Compute(missMgrOut.testEngage, missMgrOut.indxTest, missMgrOut.time_s);
//std::cout << exciteMgrOut.cmdExcite.transpose()/kD2R << "\t";

      // OUTPUT PROCESSING
      // send control surface commands
      std::vector<float> cmdEff;
      cmdEff.resize(configData.NumberEffectors);
      std::vector<uint8_t> cmdEffSerial;
      cmdEffSerial.resize(cmdEff.size()*sizeof(float));

      cmdEff[0] = cntrlMgrOut.cmd[3]; // Throttle 
      cmdEff[1] = cntrlAllocOut.cmdAlloc[0] + exciteMgrOut.cmdExcite[1]; // Elevator
      cmdEff[2] = cntrlAllocOut.cmdAlloc[1] + exciteMgrOut.cmdExcite[2]; // Rudder
      cmdEff[3] = cntrlAllocOut.cmdAlloc[2] - exciteMgrOut.cmdExcite[0]; // Ail R
      cmdEff[4] = cntrlAllocOut.cmdAlloc[3] + fmuData.SbusRx[0].Inceptors[3]; // Flap R
      cmdEff[5] = cntrlAllocOut.cmdAlloc[4] + fmuData.SbusRx[0].Inceptors[3]; // Flap L
      cmdEff[6] = cntrlAllocOut.cmdAlloc[5] + exciteMgrOut.cmdExcite[0]; // Ail L

      memcpy(cmdEffSerial.data(), cmdEff.data(), cmdEffSerial.size());
      fmu.WriteMessage(kEffectorAngleCmd, cmdEffSerial.size(), cmdEffSerial.data());

      // DATA LOG
      CntrlMgrLog cntrlMgrLog = cntrlMgr.Log();
      CntrlAllocLog cntrlAllocLog = cntrlAllocMgr.Log();
      ExciteMgrLog exciteMgrLog = exciteMgr.Log();

      log.LogData(fmuData, airdataOut, navOut, missMgrOut, exciteMgrLog, cntrlMgrLog, cntrlAllocLog);

      // telemetry

      std::cout << std::endl;
    }
  }

	return 0;
}
