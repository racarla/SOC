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
#include "datalog.hxx"
#include "rapidjson/document.h"
#include "rapidjson/stringbuffer.h"
#include "rapidjson/writer.h"
#include <iostream>
#include <iomanip>
#include <stdint.h>

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
  /* initialize classes */
  std::cout << "Initializing software modules." << std::endl;
  std::cout << "\tInitializing FMU..." << std::flush;
  Fmu.Begin();
  std::cout << "\tdone!" << std::endl;
  /* configure classes and register with global defs */
  std::cout << "Configuring aircraft." << std::endl;
  rapidjson::Document AircraftConfiguration;
  std::cout << "\tLoading configuration..." << std::flush;
  Config.LoadConfiguration(argv[1], &AircraftConfiguration);
  std::cout << "\tdone!" << std::endl;
  std::cout << "\tConfiguring flight management unit..." << std::flush;
  Fmu.Configure(AircraftConfiguration,&GlobalData);
  std::cout << "\tdone!" << std::endl;
  if (AircraftConfiguration.HasMember("Sensor-Processing")) {
    std::cout << "\tConfiguring sensor processing..." << std::flush;
    SenProc.Configure(AircraftConfiguration["Sensor-Processing"],&GlobalData);
    std::cout << "\tdone!" << std::endl;
    if (AircraftConfiguration.HasMember("Control")&&AircraftConfiguration.HasMember("Mission-Manager")&&AircraftConfiguration.HasMember("Effectors")) {
      std::cout << "\tConfiguring mission manager..." << std::flush;
      Mission.Configure(AircraftConfiguration["Mission-Manager"],&GlobalData);
      std::cout << "\tdone!" << std::endl;
      std::cout << "\tConfiguring control laws..." << std::flush;
      Control.Configure(AircraftConfiguration["Control"],&GlobalData);
      std::cout << "\tdone!" << std::endl;
      std::cout << "\tConfiguring effectors..." << std::flush;
      Effectors.Configure(AircraftConfiguration["Effectors"],&GlobalData);
      std::cout << "\tdone!" << std::endl;
      if (AircraftConfiguration.HasMember("Excitation")) {
        std::cout << "\tConfiguring excitations..." << std::flush;
        Excitation.Configure(AircraftConfiguration["Excitation"],&GlobalData);
        std::cout << "\tdone!" << std::endl;
      }
    }
  }
  Datalog.RegisterGlobalData(GlobalData);
  std::cout << "done!" << std::endl;
  std::cout << "Entering main loop." << std::endl;
  /* main loop */
  while(1) {
    if (Fmu.ReceiveSensorData()) {
      if (SenProc.Configured()&&SenProc.Initialized()) {
        if (Mission.Configured()) {
          // run mission
          Mission.Run();
          // get and set engaged sensor processing
          SenProc.SetEngagedSensorProcessing(Mission.GetEnagagedSensorProcessing());
        }
        // run sensor processing
        SenProc.Run();
        if (Control.Configured()&&Mission.Configured()&&Effectors.Configured()) {
          // get and set engaged and armed controllers
          Control.SetEngagedController(Mission.GetEnagagedController());
          Control.SetArmedController(Mission.GetArmedController());
          // get and set engaged excitation
          Excitation.SetEngagedExcitation(Mission.GetEnagagedExcitation());
          // loop through control levels running excitations and control laws
          for (size_t i=0; i < Control.ActiveControlLevels(); i++) {
            if (Excitation.Configured()) {
              // run excitation
              Excitation.Run(Control.GetActiveLevel(i));
            }
            // run control
            Control.Run(i);
          }
          // run effectors
          Effectors.Run();
        }
        // run telemetry
      }
      // run datalog
      Datalog.LogBinaryData();
    }
  }
	return 0;
}
