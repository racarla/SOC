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

  std::cout << sizeof(NavigationData) << std::endl;

  /* load configuration file */
  LoadConfigFile(argv[1],Sensors,&Config,&Data);

  /* main loop */
  while (1) {
    if (Sensors.GetSensorData(&Data)) {

      // run navigation filter
      if (Data.Gps.size() > 0) {
        if (!NavFilter.Initialized) {
          NavFilter.InitializeNavigation(Data);
        } else {
          NavFilter.RunNavigation(Data,&NavData);
        }
      }

      // control laws

      // // send control surface commands
      // std::vector<float> EffectorCmd;
      // EffectorCmd.resize(Config.NumberEffectors);
      // std::vector<uint8_t> EffectorBuffer;
      // EffectorBuffer.resize(EffectorCmd.size()*sizeof(float));
      // EffectorCmd[0] = -0.3;
      // memcpy(EffectorBuffer.data(),EffectorCmd.data(),EffectorBuffer.size());
      // Sensors.WriteMessage(kEffectorAngleCmd,EffectorBuffer.size(),EffectorBuffer.data());

      // data logging
      Log.LogFmuData(Data);
    }
  }

	return 0;
}
