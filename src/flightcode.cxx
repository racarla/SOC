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
  DatalogClient Datalog;

  /* initialize classes */
  std::cout << "Initializing software modules..." << std:: endl;
  std::cout << "\tInitializing FMU..." << std::endl;
  Fmu.Begin();

  /* configure classes */
  Config.LoadConfiguration(argv[1]);
  Fmu.UpdateConfiguration(Config.GetSensorConfiguration());

  /* Register classes with GlobalData */
  std::cout << "Registering classes with global definition tree..." << std:: endl;
  Fmu.RegisterGlobalData(&GlobalData);
  Datalog.RegisterGlobalData(GlobalData);

  while(1) {
    if (Fmu.ReceiveSensorData()) {
      Datalog.LogBinaryData();
    }
  }

	return 0;
}
