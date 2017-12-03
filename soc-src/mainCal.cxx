/*
Calibration routine

See: LICENSE.md for Copyright and License Agreement

History:
2017-11-12 - Chris Regan - Created
*/

#include "configData.hxx"
#include "fmu.hxx"
#include "hardware-defs.hxx"
#include "global-defs.hxx"
#include "inclinometer.hxx"
#include <iostream>
#include <Eigen/Core>

#define MaxCmdDim 11

typedef Eigen::Matrix<float, -1, 1, 0, MaxCmdDim, 1> VecCmd;


int main(int argc, char* argv[]) {
  if (argc!=2) {
    std::cerr << "ERROR: Incorrect number of input arguments." << std::endl;
    return -1;
  }

  // FMU fmu
  Fmu fmu; // Class
  FmuData fmuData; // Struct

  // Data Logger
  Datalogger log;

  // load configuration file
  AircraftConfig configData; // Structure
  LoadConfigFile(argv[1], fmu, &configData, &fmuData);

  // Inclinomenter Setup
  Incline incline;
  InclineData incData;

  
  /* Define the Control Effector Vector */
  std::vector<float> EffectorCmd;
  //configData.NumberEffectors = 7; // FIXIT - Hardcoded
  EffectorCmd.resize(configData.NumberEffectors);
  
  std::vector<uint8_t> EffectorBuffer;
  EffectorBuffer.resize(EffectorCmd.size()*sizeof(float));
  
  /* Setup Inclinometer */
  incline.SetDamping();
  
  // Define Delays
  int DelayMove = 2000000; // delay for 2 second after servo move
  int DelayRead = 100000; // delay for 50 ms between reads

  int NumRead = 30; // Number of iteration to sample
  

  int ServoIndx = -1;
  int AnalogIndx = -1;
  
  // Servo Command Definition
  float CmdStart_;
  float CmdEnd_;
  int NumCmds = MaxCmdDim;

  VecCmd CmdList_;
  CmdList_.setZero(NumCmds);

  std::cout << "The control surface calibration routine requires the Inclinometer connected to the BBB_UART1 " << std::endl;
  std::cout << "The commands sent to the control surface are based on the type of servo." << std::endl;
  std::cout << "PWM range is typically 1000 to 2000, SBUS range is XXX to XXX." << std::endl;
  std::cout << "Best results are obtained by tranversing nearly the entire surface range; starting with the surface down and incrementally increasing." << std::endl;

  //Get the SBUS or PWM channel number
  std::cout << "Enter the Servo Index Number: ";
  std::cin >> ServoIndx;
  
  // Surface Commands to free-float
  EffectorCmd[ServoIndx] = 0.0;
  memcpy(EffectorBuffer.data(), EffectorCmd.data(), EffectorBuffer.size()); // copy
  fmu.WriteMessage(kEffectorDirectCmd, EffectorBuffer.size(), EffectorBuffer.data()); // write commands

  //Get the Pot channel number
  std::cout << "Enter the Analog Index Number for Surface Pot (-1 to skip): ";
  std::cin >> AnalogIndx;

  //Get Starting Command
  std::cout << "Enter the Starting Command: ";
  std::cin >> CmdStart_;

  //Get Ending Command
  std::cout << "Enter the Ending Command: ";
  std::cin >> CmdEnd_;

  // Generate the list of servo commands
  CmdList_.setLinSpaced(NumCmds, CmdStart_, CmdEnd_);

  // Begin the test
  std::cout << "Beginning Test, Surface: " << ServoIndx << std::endl; 

  std::cout << "Turn OFF Servo Power, Hold Surface at Zero Position, Press Enter to continue . . . ";
  std::cin.ignore();
  std::cout << "Proceeding " << std::endl;
  
  usleep(DelayRead); // delay
  
  /* main loop */
  while (1) {
    // Establish Zero Angle
    float IncAngleSum = 0.0;
    for (int iRead = 0; iRead < NumRead; ++iRead) {

      // Delay
      usleep (DelayRead);

      // Read inclinometer
      incline.GetAngle(&incData);
      
      // Accumulate Values
      IncAngleSum += incData.Angle_deg;
      }

    // Average values, define as zero angle
    float incAngleZero = IncAngleSum / NumRead;

    std::cout << "Zero Angle: " << incAngleZero << std::endl;

    std::cout << "Beginning Automated Test..." << std::endl;
    std::cout << "Turn on Servo Power, Press Enter to continue . . . ";
    std::cin.ignore();
    std::cout << "Proceeding " << std::endl;
    
    usleep(DelayMove); // delay
    
    std::cout << "Starting... " << std::endl;

    // Loop Command Steps
    VecCmd incAngleMeas;
    incAngleMeas.setZero(NumCmds);
  
    VecCmd potValMeas;
    potValMeas.setZero(NumCmds);
    
    for (int iCmd = 0; iCmd < NumCmds; ++iCmd) {
      // Command Servo to command
      float cmdCurr = CmdList_[iCmd];
      
      std::cout << "Servo Command: " << cmdCurr << std::endl;
      
      // Send the Servo Command
      EffectorCmd[ServoIndx] = cmdCurr;
      memcpy(EffectorBuffer.data(),EffectorCmd.data(),EffectorBuffer.size());
      fmu.WriteMessage(kEffectorDirectCmd, EffectorBuffer.size(), EffectorBuffer.data());
      
      usleep(DelayMove); // delay
      
      IncAngleSum = 0.0;
      float potValTemp = 0.0;
      float potValSum = 0.0;
      // Read the Inclinometer and pot data
      for (int iRead = 0; iRead < NumRead; ++iRead) {
        // Keep Sending the same servo command
        memcpy(EffectorBuffer.data(),EffectorCmd.data(),EffectorBuffer.size());
        fmu.WriteMessage(kEffectorDirectCmd,EffectorBuffer.size(),EffectorBuffer.data());
      
        // Delay
        usleep (DelayRead);

        // Read inclinometer
        incline.GetAngle(&incData);
        
        // Accumulate Values
        IncAngleSum += incData.Angle_deg;
        
        // Read Pot fmu
        if(AnalogIndx >= 0){
          fmu.GetSensorData(&fmuData);
        
          potValTemp = fmuData.Analog[AnalogIndx].Voltage_V;
          potValSum += potValTemp;
        }
      }

      // Average values
      incAngleMeas[iCmd] = (IncAngleSum / NumRead) - incAngleZero; // Average minus the 'Zero'
      if(AnalogIndx >= 0){potValMeas[iCmd] = potValSum / NumRead;}

    }

    // Surface Commands to free-float
    EffectorCmd[ServoIndx] = 0.0;
    memcpy(EffectorBuffer.data(), EffectorCmd.data(), EffectorBuffer.size()); // copy
    fmu.WriteMessage(kEffectorDirectCmd, EffectorBuffer.size(), EffectorBuffer.data()); // write commands
        
    
    std::cout << "Calibration Complete" << std::endl;
    std::cout << "%% " << "Surface: " << ServoIndx << std::endl;
    std::cout << "CmdList_ = [" << CmdList_.transpose() << "];" << std::endl;
    std::cout << "incAngleMeas_rad = [" << incAngleMeas.transpose() << "] * d2r;" << std::endl;
    
    if(AnalogIndx >= 0){
      std::cout << "potValMeas_rad = [" << potValMeas.transpose() << "] * d2r;" << std::endl;
    }
    
    std::cout << "fitOrder = 1; % Start with Linear, alter as required" << std::endl;
    std::cout << "[polyCoefServo, cmdZero] = SurfCal(CmdList_, incAngleMeas_rad, fitOrder);" << std::endl;
    if(AnalogIndx >= 0){std::cout << "[polyCoefPot, potZero] = SurfCal(potValMeas, incAngleMeas_rad, fitOrder);" << std::endl;}
    std::cout << "%%" << std::endl;
    
	return 0;
	
  }

}

