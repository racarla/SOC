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
#include "datalogger.hxx"
#include "config.hxx"
#include "fmu.hxx"
#include "hardware-defs.hxx"
#include "global-defs.hxx"
#include "missionMgr.hxx"
//#include "cntrlMgr.hxx"
//#include "cntrlAllocMgr.hxx"
//#include "exciteMgr.hxx"

#include <iostream>

int main(int argc, char* argv[]) {
  if (argc!=2) {
    std::cerr << "ERROR: Incorrect number of input arguments." << std::endl;
    return -1;
  }

  /* initialize classes */
  Fmu Sensors;
  Datalogger Log;
  Navigation NavFilter;

  /* initialize structures */
  AircraftConfig Config;
  FmuData Data;
  NavigationData NavData;

  /* load configuration file */
  LoadConfigFile(argv[1], Sensors, &Config,&Data);

  // SETUP MISSION
  // Define Mission Manager
  MissionMgr missionMgr; // Create the Mission Manager
  missionMgr.Init(); // Initialize mission manager

  // Define Controller Manager and Controllers
//  CntrlMgr cntrlMgr;       // Create the Controller Manager
//  cntrlMgr.CntrlBaseDef(); // Define the Baseline Controller
//  cntrlMgr.CntrlResDef();  // Define the Baseline Controller

  // Define Excitation Manager and Excitations
//  ExciteMgr exciteMgr; // Create the Excitation Manager
//  exciteMgr.Init();    // Initialize the Excitation Manager

  // Define Control Allocation
//  CntrlAllocMgr cntrlAllocMgr; // Create the Control Allocator

//  numObj = 5;
//  numEff = 7;

//  MatCntrlEff cntrlEff(numObj, numEff); // effectiveness matrix
//  VecEff uMin(numEff), uMax(numEff), uPref(numEff); // cmd, min, max, prefered effector commands
//  MatObj wtObj(numObj, numObj); // Objective weighting matrix
//  MatEff wtEff(numEff, numEff); // Effector weighting matrix

  // Control Allocation Definition
  // Surface Order - Elev, Rud, AilL, FlapL, FlapR, AilR
  // Objectives Order - Roll Rate, Pitch Rate, Yaw Rate
  // cntrlEff << 0.0000, -5.0084, 78.235,  33.013, -33.013, -78.235,
  //            -133.69, -0.0047, 3.0002,  2.6238,  2.6238,  3.0002,
  //             0.0000, -82.041, 5.7521, -1.7941,  1.7941,  5.7521; // rad/s per rad

  // Objectives Order - Roll Rate, Pitch Rate, Yaw Rate
//  cntrlEff << 0.0000,  0.0000, 78.235,  33.013, -33.013, -78.235,
//             -133.69,  0.0000, 0.0000,  0.0000,  0.0000,  0.0000,
//              0.0000, -82.041, 0.0000,  0.0000,  0.0000,  0.0000; // rad/s per rad

//  uMin << -25.0*kD2R, -25.0*kD2R, -25.0*kD2R, -25.0*kD2R, -25.0*kD2R ;
//  uMax <<  25.0*kD2R,  25.0*kD2R,  25.0*kD2R,  25.0*kD2R,  25.0*kD2R ;
//  uPref << 0.0,        0.0,        0.0,        0.0,        0.0 ;

//  MatObj.setIdenty();
//  MatEff.setIdenty();

//  cntrlAllocMgr.Init(cntrlEff, wtObj, wtEff, uMin, uMax, uPref);     // Initialize Control Allocator

//  VecObj cmdObj(3)
//  VecEff cmdAlloc(6);


  /* main loop */
  while (1) {
    if (Sensors.GetSensorData(&Data)) {
      // INPUT PROCESSING

      // MISSION MANAGER
        // Mode Switching
      MissionMode modeMgr = missionMgr.ModeMgr(Data);

      std::cout << modeMgr.time_s << "\t" << modeMgr.autoEngage << "\t" << modeMgr.cntrlMode << "\t" << modeMgr.testEngage << "\t" << modeMgr.indxTest << "\t" << std::endl;


      // SENSOR PROCESSING
      // navigation filter
      if (Data.Gps.size() > 0) {
        if (!NavFilter.Initialized) {
          NavFilter.InitializeNavigation(Data);
        } else {
          NavFilter.RunNavigation(Data,&NavData);
        }
      }
      // smoothing filters

      // CONTROL LAWS
      // execute inner-loop control law
//      cmdBase = cntrlMgr.ComputeBase(refVec, timeCurr_s)
//      cmdRes = cntrlMgr.ComputeRes(refVec, measVec, dMeasVec, timeCurr_s);
//      cmdCntrl = cntrlMgr.Compute();

      // Apply command excitations
//      excitationFlag = ExciteMgr.Run(missionMgr.exciteMode_, missionMgr.indxTest_, missionMgr.timeCurr_s_);
//      cmdExcite = ExciteMgr.GetSignal();

/*      // Allocate control surfaces / mixer
      cmdObj[0] = cmdCntrl[0] + cmdExcite[0];
      cmdObj[1] = cmdCntrl[1] + cmdExcite[1];
      cmdObj[2] = cmdCntrl[2] + cmdExcite[2];
      cmdAlloc = cntrlAllocMgr.Run(cmdObj);

      cmdElev = cmdAlloc[0];
      cmdRud = cmdAlloc[1];
      cmdAilL = cmdAlloc[2];
      cmdFlapL = cmdAlloc[3];
      cmdFlapR = cmdAlloc[4];
      cmdAilR = cmdAlloc[5];

      cmdThrot = cmdCntrl[3] + cmdExcite[3];
*/
      // OUTPUT PROCESSING
      // send control surface commands
      std::vector<float> EffectorCmd;
      EffectorCmd.resize(Config.NumberEffectors);
      std::vector<uint8_t> EffectorBuffer;
      EffectorBuffer.resize(EffectorCmd.size()*sizeof(float));
/*
      EffectorCmd[0] = cmdThrot;
      EffectorCmd[1] = cmdElev;
      EffectorCmd[2] = cmdRud;
      EffectorCmd[3] = cmdAilL;
      EffectorCmd[4] = cmdFlapL;
      EffectorCmd[5] = cmdFlapR;
      EffectorCmd[6] = cmdAilR;
*/
      memcpy(EffectorBuffer.data(),EffectorCmd.data(),EffectorBuffer.size());
      Sensors.WriteMessage(kEffectorAngleCmd,EffectorBuffer.size(),EffectorBuffer.data());

      // DATA LOG
      Log.LogFmuData(Data);

      // telemetry
    }
  }

	return 0;
}
