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
#include "cntrlMgr.hxx"
#include "cntrlAllocMgr.hxx"
#include "exciteMgr.hxx"

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
  LoadConfigFile(argv[1], Sensors, &Config, &Data);

  // SETUP MISSION
  // Define Mission Manager
  MissionMgr missionMgr; // Create the Mission Manager
  missionMgr.Init(); // Initialize mission manager

  // Define Controller Manager and Controllers
  CntrlMgr cntrlMgr;       // Create the Controller Manager
  cntrlMgr.Init(); // Define the Baseline and Research Controller

  // Define Excitation Manager and Excitations
  ExciteMgr exciteMgr; // Create the Excitation Manager
  exciteMgr.Init();    // Initialize the Excitation Manager

  // Define Control Allocation
  CntrlAllocMgr cntrlAllocMgr; // Create the Control Allocator

  uint8_t numObj = 3;
  uint8_t numEff = 6;

  MatCntrlEff cntrlEff(numObj, numEff); // effectiveness matrix
  VecEff uMin(numEff), uMax(numEff), uPref(numEff); // min, max, prefered effector commands
  MatObj wtObj(numObj, numObj); // Objective weighting matrix
  MatEff wtEff(numEff, numEff); // Effector weighting matrix
  VecEff cmdAlloc(numEff); // effector allocator commands

  // Control Allocation Definition
  // Surface Order - Elev, Rud, AilR, FlapR, FlapL, AilL
  // Objectives Order - Roll Rate, Pitch Rate, Yaw Rate
//  cntrlEff <<  0.0000,  5.0084,  78.235,  33.013, -33.013, -78.235,
//              -133.69,  0.0047,  3.0002,  2.6238,  2.6238,  3.0002,
//               0.0000,  82.041, -5.7521, -1.7941,  1.7941,  5.7521; // rad/s per deg

  // Objectives Order - Roll Rate, Pitch Rate, Yaw Rate
  cntrlEff <<  0.0000,  5.0084,  78.235,  33.013, -33.013, -78.235,
              -133.69,  0.0000,  0.0000,  0.0000,  0.0000,  0.0000,
               0.0000,  82.041, -5.7521, -1.7941,  1.7941,  5.7521; // rad/s per deg

  cntrlEff *= kD2R; // Convert to rad/s per rad

  uMin << -25.0*kD2R, -15.0*kD2R, -25.0*kD2R, -25.0*kD2R, -25.0*kD2R, -25.0*kD2R ;
  uMax <<  25.0*kD2R,  15.0*kD2R,  25.0*kD2R,  25.0*kD2R,  25.0*kD2R,  25.0*kD2R ;
  uPref << 0.0,        0.0,        0.0,        0.0,        0.0,        0.0;

  wtObj.setIdentity();
  wtEff.setIdentity();

  cntrlAllocMgr.Init(cntrlEff, wtObj, wtEff, uMin, uMax, uPref);     // Initialize Control Allocator

  /* main loop */
  while (1) {
    if (Sensors.GetSensorData(&Data)) {
      // INPUT PROCESSING

      // MISSION MANAGER
        // Mode Switching
      MissionMode modeMgr = missionMgr.ModeMgr(Data);

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

      cntrlMgr.Mode(modeMgr.cntrlMode); // Transfer the Mission Control mode to the Controller Manager

std::cout << modeMgr.time_s << "\t" << modeMgr.autoEngage << "  " << modeMgr.cntrlMode << "  " << modeMgr.testArm << "  " << modeMgr.testEngage << "  " << (int) modeMgr.indxTest << "\t";

      VecCmd refVec(4);
      refVec[0] = Data.SbusRx[0].Inceptors[0];
      refVec[1] = Data.SbusRx[0].Inceptors[1];
      refVec[2] = Data.SbusRx[0].Inceptors[2];
      refVec[3] = Data.SbusRx[0].Inceptors[4];

      VecCmd measVecAngles(4);
      measVecAngles[0] = NavData.Euler_rad[0];
      measVecAngles[1] = NavData.Euler_rad[1];
      measVecAngles[2] = NavData.Euler_rad[2];
      measVecAngles[3] = 15; // FIXIT - Need filtered airspeed

      VecCmd measVecRates(4);
      measVecRates[0] = Data.Mpu9250.Gyro_rads[0];
      measVecRates[1] = Data.Mpu9250.Gyro_rads[1];
      measVecRates[2] = Data.Mpu9250.Gyro_rads[2];
      measVecRates[3] = 0.0;

      VecCmd cmdBase = cntrlMgr.CmdBase(refVec, modeMgr.time_s);
      VecCmd cmdRes = cntrlMgr.CmdRes(refVec, measVecAngles, measVecRates, modeMgr.time_s);
      VecCmd cmdCntrl = cntrlMgr.Cmd();

std::cout << refVec.transpose() << "\t\t";
std::cout << measVecAngles[2]/kD2R << "\t";
std::cout << measVecRates[2]/kD2R << "\t";
std::cout << cmdCntrl[2]/kD2R << "\t";

// FIXIT - the failsafe mode leaves the controller engaged, at least need to disengage the excitation
// FIXIT - Throttle controller??
// FIXIT - Need airspeed source + filtered
// FIXIT - Research Yaw controller??
// FIXIT - check reference command limits, manual mode pegs the cmd with really small stick deflections


      // Apply command excitations
      VecChan cmdExcite(4);
      cmdExcite = exciteMgr.Compute(modeMgr.testEngage, modeMgr.indxTest, modeMgr.time_s);
//std::cout << cmdExcite.transpose()/kD2R << "\t";

      // Allocate control surfaces / mixer
      VecCmd objAlloc = cmdCntrl.head(3);
      cmdAlloc = cntrlAllocMgr.Compute(objAlloc);

//std::cout << cmdAlloc.transpose()/kD2R << "\t";

      float cmdElev = cmdAlloc[0] + cmdExcite[1];
      float cmdRud = cmdAlloc[1] + cmdExcite[2];
      float cmdAilL = cmdAlloc[2] + cmdExcite[0];
      float cmdFlapL = cmdAlloc[3] + Data.SbusRx[0].Inceptors[3];
      float cmdFlapR = cmdAlloc[4] + Data.SbusRx[0].Inceptors[3];
      float cmdAilR = cmdAlloc[5] - cmdExcite[0];

      float cmdThrot = cmdCntrl[3];

VecEff cmdTemp(7);
cmdTemp[0] = cmdThrot;
cmdTemp[1] = cmdElev;
cmdTemp[2] = cmdRud;
cmdTemp[3] = cmdAilL;
cmdTemp[4] = cmdFlapL;
cmdTemp[5] = cmdFlapR;
cmdTemp[6] = cmdAilR;

//std::cout << cmdTemp.transpose()/kD2R << "\t";

      // OUTPUT PROCESSING
Config.NumberEffectors = 7; // FIXIT - Shouldn't need this!!
      // send control surface commands
      std::vector<float> EffectorCmd;
      EffectorCmd.resize(Config.NumberEffectors);
      std::vector<uint8_t> EffectorBuffer;
      EffectorBuffer.resize(EffectorCmd.size()*sizeof(float));

      EffectorCmd[0] = cmdThrot;
      EffectorCmd[1] = cmdElev;
      EffectorCmd[2] = cmdRud;
      EffectorCmd[3] = cmdAilR;
      EffectorCmd[4] = cmdFlapR;
      EffectorCmd[5] = cmdFlapL;
      EffectorCmd[6] = cmdAilL;

      // Saturate(uMin, uMax, EffectorCmd);

      memcpy(EffectorBuffer.data(), EffectorCmd.data(), EffectorBuffer.size());
      Sensors.WriteMessage(kEffectorAngleCmd, EffectorBuffer.size(), EffectorBuffer.data());

      // DATA LOG
      Log.LogFmuData(Data);

      // telemetry

      std::cout << std::endl;
    }
  }

	return 0;
}
