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

#include "navigation.hxx"
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

#include <iostream>
#include <iomanip>      // std::setw


#ifndef kMaxAllocObj // Allocator Objectives
#define kMaxAllocObj 3
#endif

#ifndef kMaxAllocEff // Allocator Effectors
#define kMaxAllocEff 6
#endif

#ifndef kMaxCntrlCmd // Controller Dimension
#define kMaxCntrlCmd 4
#endif

#ifndef kMaxExciteChan // Excitation Channels
#define kMaxExciteChan 4
#endif

#ifndef kMaxExciteElem // Excitation Elements (Multisine components)
#define kMaxExciteElem 46
#endif

#define EIGEN_INITIALIZE_MATRICES_BY_ZERO 1
//#define EIGEN_INITIALIZE_MATRICES_BY_NAN 1


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
  AirdataStruct airdataData; // Struct
//std::cout << "Airdata" << "\t";

  // Navigation Filter
  Navigation NavFilter; // Class
  NavigationData navData; // Struct
//std::cout << "Navigation" << "\t";

  // SETUP MISSION
  // Define Mission Manager
  MissMgr missMgr; // Create the Mission Manager
  missMgr.Init(); // Initialize mission manager
  MissMgrStruct missMgrData; // Create the Mission Manage Structure
//std::cout << "MissMgr" << "\t";

  // Define Controller Manager and Controllers
  CntrlMgr cntrlMgr; // Create the Controller Manager
  cntrlMgr.Init();   // Define the Baseline and Research Controller
  CntrlMgrStruct cntrlMgrData; // Struct
//std::cout << "CntrlMgr" << "\t";

  // Define Excitation Manager and Excitations
  ExciteMgr exciteMgr; // Create the Excitation Manager
  exciteMgr.Init();    // Initialize the Excitation Manager
  ExciteMgrStruct exciteMgrData; // Struct
//std::cout << "ExciteMgr" << "\t";

  // Define Control Allocation
  CntrlAllocMgr cntrlAllocMgr; // Create the Control Allocator
  CntrlAllocDef cntrlAllocDef; // Structure for control allocation definition
  CntrlAllocStruct cntrlAllocData; // Structure for control allocation data

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
      missMgrData = missMgr.ModeMgr(fmuData);
//std::cout << "missMgrData" << "\t";

      // SENSOR PROCESSING
      // Airdata
      airdataData = airdata.Compute(fmuData.Pitot[0]);
//std::cout << "Airdata" << "\t";

      // Compute the Airdata Biases during startup, 10 seconds @ 50 Hz
      if (missMgrData.frame_cnt < (500)) {
        airdata.BiasEst();
      }

      // navigation filter
      if (fmuData.Gps.size() > 0) {
        if (!NavFilter.Initialized) {
          NavFilter.InitializeNavigation(fmuData);
        } else {
          NavFilter.RunNavigation(fmuData,&navData);
        }
      }

      // CONTROL LAWS
      // Execute inner-loop control law
      cntrlMgr.Mode(missMgrData.cntrlMode); // Transfer the Mission Control mode to the Controller Manager

std::cout << missMgrData.time_s << "\t";
std::cout << missMgrData.frame_cnt << "\t";
std::cout << missMgrData.autoEngage << "  ";
std::cout << missMgrData.cntrlMode << "  ";
std::cout << missMgrData.testArm << "  ";
std::cout << missMgrData.testEngage << "  ";
std::cout << (int) missMgrData.indxTest << "\t";

std::cout << std::setw(10);

std::cout << airdataData.alt_m << "\t";
std::cout << airdataData.altFilt_m << "\t\t";
std::cout << airdataData.vIas_mps << "\t";
std::cout << airdataData.vIasFilt_mps << "\t\t";

std::cout << fmuData.PressureTransducer[0].Pressure_Pa << "\t";
std::cout << fmuData.PressureTransducer[1].Pressure_Pa << "\t";
std::cout << fmuData.PressureTransducer[2].Pressure_Pa << "\t";
std::cout << fmuData.PressureTransducer[3].Pressure_Pa << "\t";

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
      measVec[0] = navData.Euler_rad[0];
      measVec[1] = navData.Euler_rad[1];
      measVec[2] = navData.Euler_rad[2];
      measVec[3] = airdataData.vIasFilt_mps;
//std::cout << measVec.transpose() << "\t";

      VecCmd dMeasVec(kMaxCntrlCmd);
      dMeasVec[0] = fmuData.Mpu9250.Gyro_rads[0];
      dMeasVec[1] = fmuData.Mpu9250.Gyro_rads[1];
      dMeasVec[2] = fmuData.Mpu9250.Gyro_rads[2];
      dMeasVec[3] = 0.0;
//std::cout << dMeasVec.transpose() << "\t";

      // Run the controllers
      cntrlMgr.CmdBase(refVecBase, missMgrData.time_s);
      cntrlMgr.CmdRes(refVecRes, measVec, dMeasVec, missMgrData.time_s);
      cntrlMgrData = cntrlMgr.Cmd();
//std::cout << cntrlMgrData.cmd.transpose() << "\t";
//std::cout << cntrlMgrData.cmd[0] << "\t\t";

      // Allocate control surfaces
      VecCmd objAlloc = cntrlMgrData.cmd.head(3);
      cntrlAllocData = cntrlAllocMgr.Compute(objAlloc);
//std::cout << cntrlAllocData.cmdAlloc.transpose()/kD2R << "\t";

      // Apply command excitations
      exciteMgrData = exciteMgr.Compute(missMgrData.testEngage, missMgrData.indxTest, missMgrData.time_s);
//std::cout << exciteMgrData.cmdExcite.transpose()/kD2R << "\t";

      // OUTPUT PROCESSING
      // send control surface commands
      std::vector<float> cmdEff;
      cmdEff.resize(configData.NumberEffectors);
      std::vector<uint8_t> cmdEffSerial;
      cmdEffSerial.resize(cmdEff.size()*sizeof(float));

      cmdEff[0] = cntrlMgrData.cmd[3]; // Throttle 
      cmdEff[1] = cntrlAllocData.cmdAlloc[0] + exciteMgrData.cmdExcite[1]; // Elevator
      cmdEff[2] = cntrlAllocData.cmdAlloc[1] + exciteMgrData.cmdExcite[2]; // Rudder
      cmdEff[3] = cntrlAllocData.cmdAlloc[2] - exciteMgrData.cmdExcite[0]; // Ail R
      cmdEff[4] = cntrlAllocData.cmdAlloc[3] + fmuData.SbusRx[0].Inceptors[3]; // Flap R
      cmdEff[5] = cntrlAllocData.cmdAlloc[4] + fmuData.SbusRx[0].Inceptors[3]; // Flap L
      cmdEff[6] = cntrlAllocData.cmdAlloc[5] + exciteMgrData.cmdExcite[0]; // Ail L

      memcpy(cmdEffSerial.data(), cmdEff.data(), cmdEffSerial.size());
      fmu.WriteMessage(kEffectorAngleCmd, cmdEffSerial.size(), cmdEffSerial.data());

      // DATA LOG
      log.LogData(fmuData, airdataData, navData, missMgrData, exciteMgrData, cntrlMgrData, cntrlAllocData);

      // telemetry

      std::cout << std::endl;
    }
  }

	return 0;
}
