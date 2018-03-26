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

#include "hardware-defs.hxx"
#include "global-defs.hxx"
#include "datalogger.hxx"
#include "fmu.hxx"
#include <iostream>
#include <iomanip>
#include <stdint.h>
#include <variant>

int main(int argc, char* argv[]) {

  /* declare classes */
  DefinitionTree GlobalData;
  FlightManagementUnit Fmu;
  Datalogger Datalog;

  /* initialize classes */
  Fmu.Begin();

  /* Register classes with GlobalData */
  Fmu.RegisterGlobalData(&GlobalData);
  Datalog.RegisterGlobalData(GlobalData);

  size_t i=0;

  while(i < 100) {
    if (Fmu.ReceiveSensorData()) {

      Datalog.LogBinaryData();
      i++;
    }
  }

  Datalog.End();
  Datalog.CreateHdfLog();

	return 0;
}
