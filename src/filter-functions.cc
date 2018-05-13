/*
filter-functions.cc
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

#include "filter-functions.hxx"

// /* washout filter class methods */

// /* method for configuring the washout filter block */
// /* example JSON configuration:
// {
//   "Output": "OutputName",
//   "Input": "InputName",
//   "Sample-Time": "SampleTimeName",
//   "Time-Constant": X
// }
// Where OutputName gives a convenient name for the block (i.e. SpeedControl).
// Input is the full path name of the input signal
// Sample-Time is the full path name of the sample time signal
// Time-Constant is the time constant of the filter
// */
// void WashoutFilterClass::Configure(const rapidjson::Value& Config,std::string RootPath,DefinitionTree *DefinitionTreePtr) {
//   std::string OutputName;
//   if (Config.HasMember("Output")) {
//     config_.OutputName = Config["Output"].GetString();
//     OutputName = RootPath + "/" + Config["Output"].GetString();
//   } else {
//     throw std::runtime_error(std::string("ERROR")+RootPath+std::string(": Output not specified in configuration."));
//   }
//   if (Config.HasMember("Input")) {
//     if (DefinitionTreePtr->GetValuePtr<float*>(Config["Input"].GetString())) {
//       config_.Input = DefinitionTreePtr->GetValuePtr<float*>(Config["Input"].GetString());
//     } else {
//       throw std::runtime_error(std::string("ERROR")+OutputName+std::string(": Input ")+Config["Input"].GetString()+std::string(" not found in global data."));
//     }
//   } else {
//     throw std::runtime_error(std::string("ERROR")+OutputName+std::string(": Input not specified in configuration."));
//   }
//   if (Config.HasMember("Sample-Time")) {
//     if (DefinitionTreePtr->GetValuePtr<float*>(Config["Sample-Time"].GetString())) {
//       config_.dt = DefinitionTreePtr->GetValuePtr<float*>(Config["Sample-Time"].GetString());
//     } else {
//       throw std::runtime_error(std::string("ERROR")+OutputName+std::string(": Sample time ")+Config["Sample-Time"].GetString()+std::string(" not found in global data."));
//     }
//   } else {
//     throw std::runtime_error(std::string("ERROR")+OutputName+std::string(": Sample time not specified in configuration."));
//   }
//   if (Config.HasMember("Time-Constant")) {
//     config_.Tf = Config["Time-Constant"].GetFloat();
//   } else {
//     throw std::runtime_error(std::string("ERROR")+OutputName+std::string(": Time constant not specified in configuration."));
//   }
//   // pointer to log run mode data
//   DefinitionTreePtr->InitMember(OutputName+"/Mode",&data_.Mode,"Control law mode",true,false);
//   // pointer to log command data
//   DefinitionTreePtr->InitMember(OutputName+"/Output",&data_.Command,"Control law output",true,false);
// }

// /* sets a pointer to the previous output in order to initialize to a transient free state */
// void WashoutFilterClass::SetPreviousOutput(std::string RootPath,DefinitionTree *DefinitionTreePtr) {
//   if (DefinitionTreePtr->GetValuePtr<float*>(RootPath+ "/" + config_.OutputName)) {
//     config_.PreviousCommand = DefinitionTreePtr->GetValuePtr<float*>(RootPath+ "/" + config_.OutputName);
//   } else {
//     throw std::runtime_error(std::string("ERROR")+std::string(": Command ")+RootPath+ "/" + config_.OutputName+std::string(" not found in global data."));
//   }
// }

// /* washout filter run method, outputs the mode and value */
// void WashoutFilterClass::Run(Mode mode) {
//   data_.Mode = (uint8_t) mode;
//   switch (mode) {
//     // Zero the State and Command
//     case kReset: {
//       data_.Command = 0;
//       break;
//     }
//     // Do Nothing, State and Command are unchanged
//     case kStandby: {
//       break;
//     }
//     // Run Commands
//     case kHold: {
//       CalculateCommand();
//       break;
//     }
//     // Initialize State then Run Commands
//     case kInitialize: {
//       InitializeState();
//       CalculateCommand();
//       break;
//     }
//     // Update the State and Run Commands
//     case kEngage: {
//       CalculateCommand();
//       UpdateState();
//       break;
//     }
//   }
// }

// /* initialize the state */
// void WashoutFilterClass::InitializeState() {
//   states_.Filter = *config_.Input - *config_.PreviousCommand;
// }

// /* update the state */
// void WashoutFilterClass::UpdateState() {
//   if (config_.Tf > *config_.dt) {
//     states_.Filter = (1.0f-(*config_.dt/config_.Tf))*states_.Filter + (*config_.dt/config_.Tf)*(*config_.Input);
//   }
// }

// /* calculate the command */
// void WashoutFilterClass::CalculateCommand() {
//   if (config_.Tf > *config_.dt) {
//     data_.Command = *config_.Input - states_.Filter;
//   } else {
//     data_.Command = *config_.Input;
//   }
// }
