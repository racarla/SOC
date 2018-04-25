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
#include "datalog.hxx"
#include "fmu.hxx"
#include "sensor-processing.hxx"
#include "control.hxx"
#include "mission.hxx"
#include "rapidjson/document.h"
#include "rapidjson/stringbuffer.h"
#include "rapidjson/writer.h"
#include <iostream>
#include <iomanip>
#include <stdint.h>
#include <variant>

int main(int argc, char* argv[]) {
  if (argc!=2) {
      std::cerr << "ERROR: Incorrect number of input arguments." << std::endl;
      std::cerr << "Configuration file name needed." << std::endl;
      return -1;
  }

  std::cout << "Bolder Flight Systems" << std::endl;
  std::cout << "Flight Software Version " << SoftwareVersion << std::endl << std::endl;

  /* declare classes */
  Configuration Config;
  DefinitionTree GlobalData;
  FlightManagementUnit Fmu;
  SensorProcessing SenProc;
  ControlLaws Control;
  MissionManager Mission;
  DatalogClient Datalog;

  /* initialize classes */
  std::cout << "Initializing software modules..." << std:: endl;
  std::cout << "\tInitializing FMU..." << std::endl;
  Fmu.Begin();

  /* configure classes and register with global defs */
  rapidjson::Document AircraftConfiguration;
  Config.LoadConfiguration(argv[1], &AircraftConfiguration);
  Fmu.Configure(AircraftConfiguration,&GlobalData);
  if (AircraftConfiguration.HasMember("Sensor-Processing")) {
    SenProc.Configure(AircraftConfiguration["Sensor-Processing"],&GlobalData);
  }
  if (AircraftConfiguration.HasMember("Control")) {
    Control.Configure(AircraftConfiguration["Control"],&GlobalData);
  }
  if (AircraftConfiguration.HasMember("Mission")) {
    Mission.Configure(AircraftConfiguration["Mission"],&GlobalData);
  }
  Datalog.RegisterGlobalData(GlobalData);

  /* main loop */
  while(1) {
    if (Fmu.ReceiveSensorData()) {
      if (SenProc.Initialized()) {
        SenProc.Run();
        // run mission
        // mission get excitation arm and engage
        // excitation set arm and engage
        // mission get control arm and engage
        // select control arm and engage
        // mission get allocator arm and engage
        // select allocator arm and engage
        // for each control level in group
        //    run excitation
        //    run control
        // run allocator
      }
      // run telemetry
      Datalog.LogBinaryData();
    }
  }

	return 0;
}
