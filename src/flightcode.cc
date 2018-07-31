/*
flightcode.cxx
Brian R Taylor
brian.taylor@bolderflight.com

Copyright (c) 2018 Bolder Flight Systems
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

#include "hardware-defs.hxx"
#include "definition-tree.hxx"
#include "configuration.hxx"
#include "fmu.hxx"
#include "sensor-processing.hxx"
#include "mission.hxx"
#include "control.hxx"
#include "excitation.hxx"
#include "effector.hxx"
#include "telemetry.hxx"
#include "datalog.hxx"
#include "rapidjson/document.h"
#include "rapidjson/stringbuffer.h"
#include "rapidjson/writer.h"
#include <iostream>
#include <iomanip>
#include <stdint.h>

#include <time.h>
#define kTIC2MS 1000/CLOCKS_PER_SEC; // Convert CPU tics to milli-seconds

int main(int argc, char* argv[]) {
  if (argc!=2) {
      std::cerr << "ERROR: Incorrect number of input arguments." << std::endl;
      std::cerr << "Configuration file name needed." << std::endl;
      return -1;
  }

  /* displaying software version information */
  std::cout << "Bolder Flight Systems" << std::endl;
  std::cout << "Flight Software Version " << SoftwareVersion << std::endl << std::endl;

  /* declare classes */
  DefinitionTree GlobalData;
  Configuration Config;
  FlightManagementUnit Fmu;
  SensorProcessing SenProc;
  MissionManager Mission;
  ControlLaws Control;
  ExcitationSystem Excitation;
  AircraftEffectors Effectors;
  DatalogClient Datalog;
  TelemetryClient Telemetry;

  /* initialize classes */
  std::cout << "Initializing software modules." << std::endl;
  std::cout << "\tInitializing FMU..." << std::flush;
  Fmu.Begin();
  std::cout << "done!" << std::endl;

  /* configure classes and register with global defs */
  std::cout << "Configuring aircraft." << std::endl;
  rapidjson::Document AircraftConfiguration;
  std::cout << "\tLoading configuration..." << std::flush;
  Config.LoadConfiguration(argv[1], &AircraftConfiguration);
  std::cout << "done!" << std::endl;

  std::cout << "\tConfiguring flight management unit..." << std::endl;
  Fmu.Configure(AircraftConfiguration,&GlobalData);
  std::cout << "\tdone!" << std::endl;

  if (AircraftConfiguration.HasMember("Sensor-Processing")) {
    std::cout << "\tConfiguring sensor processing..." << std::flush;
    SenProc.Configure(AircraftConfiguration["Sensor-Processing"],&GlobalData);
    std::cout << "done!" << std::endl;
    if (AircraftConfiguration.HasMember("Control")&&AircraftConfiguration.HasMember("Mission-Manager")&&AircraftConfiguration.HasMember("Effectors")) {
      std::cout << "\tConfiguring mission manager..." << std::flush;
      Mission.Configure(AircraftConfiguration["Mission-Manager"],&GlobalData);
      std::cout << "done!" << std::endl;
      std::cout << "\tConfiguring control laws..." << std::flush;
      Control.Configure(AircraftConfiguration["Control"],&GlobalData);
      std::cout << "done!" << std::endl;
      std::cout << "\tConfiguring effectors..." << std::flush;
      Effectors.Configure(AircraftConfiguration["Effectors"],&GlobalData);
      std::cout << "done!" << std::endl;
      if (AircraftConfiguration.HasMember("Excitation")) {
        std::cout << "\tConfiguring excitations..." << std::flush;
        Excitation.Configure(AircraftConfiguration["Excitation"],&GlobalData);
        std::cout << "done!" << std::endl;
      }
    }
  }

  // Telemetry
  if (AircraftConfiguration.HasMember("Telemetry")) {
    std::cout << "\tConfiguring telemetry..." << std::flush;
    Telemetry.Configure(AircraftConfiguration["Telemetry"],&GlobalData);
    std::cout << "done!" << std::endl;
  }

  // Initialize in-frame timers
  clock_t tFrameStart_tic, tFrame_tic, tMission_tic, tSenProc_tic, tControl_tic, tSendEffector_tic, tFrameLast_tic;
  float tFrame_ms, tMission_ms, tSenProc_ms, tControl_ms, tSendEffector_ms, tFrameLast_ms;

  GlobalData.InitMember("/Timer/tMission_ms", &tMission_ms, "Mission manager duration timer, ms", true, false);
  GlobalData.InitMember("/Timer/tSenProc_ms", &tSenProc_ms, "Sensor Processing duration timer, ms", true, false);
  GlobalData.InitMember("/Timer/tControl_ms", &tControl_ms, "Control and Excitation duration timer, ms", true, false);
  GlobalData.InitMember("/Timer/tSendEffector_ms", &tSendEffector_ms, "Effector message out self-timer, start-to-output, ms", true, false);
  GlobalData.InitMember("/Timer/tFrame_ms", &tFrame_ms, "Major frame SOC self-timer, start-to-end, ms", true, false);
  GlobalData.InitMember("/Timer/tFrameLast_ms", &tFrameLast_ms, "Major frame SOC self-timer, start-to-start of previous frame, ms", true, false);

  // Datalog
  std::cout << "\tConfiguring datalog..." << std::flush;
  Datalog.RegisterGlobalData(GlobalData);
  std::cout << "done!" << std::endl;
  std::cout << "Entering main loop." << std::endl;

  // Pretty print the Global Data Structure
  GlobalData.PrettyPrint("/");

  /* main loop */
  while(1) {
    if (Fmu.ReceiveSensorData()) {
      // Start frame timer
      tFrameLast_tic = clock() - tFrameStart_tic;
      tFrameStart_tic = clock();

      if (SenProc.Configured()&&SenProc.Initialized()) {
        // run mission
        tMission_tic = clock();
        Mission.Run();
        tMission_tic = clock() - tMission_tic;

        // get and set engaged sensor processing
        SenProc.SetEngagedSensorProcessing(Mission.GetEngagedSensorProcessing());

        // run sensor processing
        tSenProc_tic = clock();
        SenProc.Run();
        tSenProc_tic = clock() - tSenProc_tic;

        // get and set engaged and armed controllers
        Control.SetEngagedController(Mission.GetEngagedController());
        Control.SetArmedController(Mission.GetArmedController());

        // get and set engaged excitation
        Excitation.SetEngagedExcitation(Mission.GetEngagedExcitation());

        tControl_tic = clock(); // Control and Excitation Timer
        if (Mission.GetEngagedController()!="Fmu") {

          // loop through control levels running excitations and control laws
          for (size_t i=0; i < Control.ActiveControlLevels(); i++) {
            // run excitation
            Excitation.RunEngaged(Control.GetActiveLevel(i));

            // run control
            Control.RunEngaged(i);
          }

          // send effector commands to FMU
          Fmu.SendEffectorCommands(Effectors.Run());
        }
        tSendEffector_tic = clock() - tFrameStart_tic; // Total time start-to-cmdOut

        // run armed excitations
        Excitation.RunArmed();

        // run armed control laws
        Control.RunArmed();

        tControl_tic = clock() - tControl_tic; // Control and Excitation Timer
      }

      // run telemetry
      Telemetry.Send();

      // run datalog
      Datalog.LogBinaryData();

      // Total Time start-to-end
      tFrame_tic = clock() - tFrameStart_tic;



tMission_ms = ((float) tMission_tic) * kTIC2MS;
tSenProc_ms = ((float) tSenProc_tic) * kTIC2MS;
tControl_ms = ((float) tControl_tic) * kTIC2MS;
tSendEffector_ms = ((float) tSendEffector_tic) * kTIC2MS;
tFrame_ms = ((float) tFrame_tic) * kTIC2MS;
tFrameLast_ms = ((float) tFrameLast_tic) * kTIC2MS;

std::cout <<  tMission_ms << "\t" << tSenProc_ms << "\t" << tControl_ms << "\t" << tSendEffector_ms << "\t" << tFrame_ms << "\t" << tFrameLast_ms << std::endl;
    }
  }
	return 0;
}
