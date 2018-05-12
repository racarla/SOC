/*
control.hxx
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

#ifndef CONTROL_HXX_
#define CONTROL_HXX_

#include "hardware-defs.hxx"
#include "definition-tree.hxx"
#include "rapidjson/document.h"
#include "rapidjson/stringbuffer.h"
#include "rapidjson/writer.h"
#include "generic-function.hxx"
#include "general-functions.hxx"
#include "control-functions.hxx"
#include "allocation-functions.hxx"
#include "filter-functions.hxx"
#include <stdio.h>
#include <fcntl.h>
#include <unistd.h>
#include <termios.h>
#include <stdint.h>
#include <iostream>
#include <exception>
#include <stdexcept>
#include <vector>
#include <cstring>
#include <Eigen/Dense>
#include <memory>

/* Class to manage control laws
Example JSON configuration:
{ 
  "Control": {
    "Baseline": "BaselineGroup1",
    "Research": ["ResearchGroup1","ResearchGroup2",...],
    "BaselineGroup1": [
      { "Level": "1",
        "Components": [
          { "Type": "Gain",
            "Input": 
            "Output":
            "Gain":
          }
        ]
      }
    ]
    "ResearchGroup1": [
      { "Level": "1",
        "Components": [
          { "Type": "Gain",
            "Input": 
            "Output":
            "Gain":
          }
        ]
      }
    ]
  }
}

Where:
   * Baseline names the control law group for the baseline control law to use. The baseline
     control law is run on the FMU anytime the research control law is not engaged.
   * Research is a vector of control law group names. Research control laws are run on the
     SOC anytime they are engaged.
   * Control law goups are named as arrays. Each array contains an object for each control
     law level starting with the outermost level and working to the innermost. Each level
     is named for easy reference by the excitation system. Components for each control
     level are objects configuring the specific control law.

*/

class ControlLaws {
  public:
    void Configure(const rapidjson::Value& Config, DefinitionTree *DefinitionTreePtr);
    bool Configured();
    void SetEngagedController(std::string ControlGroupName);
    void SetArmedController(std::string ControlGroupName);
    size_t ActiveControlLevels();
    std::string GetActiveLevel(size_t ControlLevel);
    void RunEngaged(size_t ControlLevel);
    void RunArmed();
  private:
    std::string RootPath_ = "/Control";
    bool Configured_ = false;
    std::string EngagedGroup_;
    std::string ArmedGroup_;
    std::vector<std::string> ResearchGroupKeys_;
    std::map<std::string,std::string> OutputKeys_;
    std::map<std::string,std::vector<std::string>> ResearchLevelNames_;
    std::map<std::string,std::vector<std::vector<std::shared_ptr<GenericFunction>>>> ResearchControlGroups_;
    std::vector<std::variant<uint64_t,uint32_t,uint16_t,uint8_t,int64_t,int32_t,int16_t,int8_t,float, double>> OutputData_;
    std::map<std::string,std::vector<std::variant<uint64_t*,uint32_t*,uint16_t*,uint8_t*,int64_t*,int32_t*,int16_t*,int8_t*,float*,double*>>> ResearchDataPtr_;
};

#endif
