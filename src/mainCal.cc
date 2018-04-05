/*
Calibration routine

See: LICENSE.md for Copyright and License Agreement

History:
2017-11-12 - Chris Regan - Created
*/

#include "config.hxx"
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

  // load configuration file
  AircraftConfig configData; // Structure
  LoadConfigFile(argv[1], fmu, &configData, &fmuData);

  // Inclinomenter Setup
  Incline incline;
  InclineData incData;
  
  /* Define the Control Effector Vector */
  std::vector<float> cmdEff;
  configData.NumberEffectors = 7; // FIXIT - Hardcoded
  cmdEff.resize(configData.NumberEffectors);
  
  std::vector<uint8_t> cmdEffSerial;
  cmdEffSerial.resize(cmdEff.size()*sizeof(float));
  
  /* Setup Inclinometer */
  incline.SetDamping();
  
  // Define Delays
  int DelayMove = 2000000; // delay for 2 second after servo move
  int DelayRead = 100000; // delay for 100 ms between reads

  int NumRead = 30; // Number of iteration to sample
  
  int ServoIndx = -1;
  int AnalogIndx = -1;
  
  // Servo Command Definition
  float CmdStart;
  float CmdEnd;
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
  cmdEff[ServoIndx] = 0.0;
  memcpy(cmdEffSerial.data(), cmdEff.data(), cmdEffSerial.size()); // copy
  fmu.WriteMessage(kEffectorDirectCmd, cmdEffSerial.size(), cmdEffSerial.data()); // write commands

  //Get the Pot channel number
  std::cout << "Enter the Analog Index Number for Surface Pot (-1 to skip): ";
  std::cin >> AnalogIndx;

  //Get Starting Command
  std::cout << "Enter the Starting Command: ";
  std::cin >> CmdStart;

  //Get Ending Command
  std::cout << "Enter the Ending Command: ";
  std::cin >> CmdEnd;

  // Generate the list of servo commands
  CmdList_.setLinSpaced(NumCmds, CmdStart, CmdEnd);

  // Begin the test
  std::cout << "Beginning Test, Surface: " << ServoIndx << std::endl; 

  std::cout << "Turn OFF Servo Power, Hold Surface at Zero Position, Press Enter to continue . . . ";
  std::cin.ignore();
  std::cout << "Proceeding " << std::endl;
  
  usleep(DelayRead); // delay
  
  /* main loop */
  while (1) {
    // Establish Zero Angle
    float IncAngleSum_deg = 0.0;
    for (int iRead = 0; iRead < NumRead; ++iRead) {

      // Delay
      usleep (DelayRead);

      // Read inclinometer
      incline.GetAngle(&incData);
      
      // Accumulate Values
      IncAngleSum_deg += incData.Angle_deg;
    }

    // Average values, define as zero angle
    float incAngleZero_deg = IncAngleSum_deg / (float) NumRead;

    std::cout << "Zero Angle: " << incAngleZero_deg << std::endl;

    std::cout << "Beginning Automated Test..." << std::endl;
    std::cout << "Turn on Servo Power, Press Enter to continue . . . ";
    std::cin.ignore();
    std::cout << "Proceeding " << std::endl;
    
    usleep(DelayMove); // delay
    
    std::cout << "Starting... " << std::endl;

    // Loop Command Steps
    VecCmd incAngleMeas_deg;
    incAngleMeas_deg.setZero(NumCmds);
  
    VecCmd potValMeas_V;
    potValMeas_V.setZero(NumCmds);
    
    for (int iCmd = 0; iCmd < NumCmds; ++iCmd) {
      // Command Servo to command
      float cmdCurr = CmdList_[iCmd];
      
      std::cout << "Servo Command: " << cmdCurr << std::endl;
      
      // Send the Servo Command
      cmdEff[ServoIndx] = cmdCurr;
      memcpy(cmdEffSerial.data(),cmdEff.data(),cmdEffSerial.size());
      fmu.WriteMessage(kEffectorDirectCmd, cmdEffSerial.size(), cmdEffSerial.data());
      
      usleep(DelayMove); // delay
      
      IncAngleSum_deg = 0.0;
      float potValTemp_V = 0.0;
      float potValSum_V = 0.0;
      // Read the Inclinometer and pot data

      for (int iRead = 0; iRead < NumRead; ++iRead) {
        while (!fmu.GetSensorData(&fmuData))

        // Keep Sending the same servo command
        memcpy(cmdEffSerial.data(),cmdEff.data(),cmdEffSerial.size());
        fmu.WriteMessage(kEffectorDirectCmd,cmdEffSerial.size(),cmdEffSerial.data());
      
        // Delay
        //usleep (DelayRead);

        // Read inclinometer
        incline.GetAngle(&incData);
        
        // Accumulate Values
        IncAngleSum_deg += incData.Angle_deg;
        
        // Read Pot fmu
        if(AnalogIndx >= 0){
        
          potValTemp_V = fmuData.Analog[AnalogIndx].Voltage_V;
          potValSum_V += potValTemp_V;
        }
      }

      // Average values
      incAngleMeas_deg[iCmd] = (IncAngleSum_deg / (float) NumRead) - incAngleZero_deg; // Average minus the 'Zero'
      if(AnalogIndx >= 0){
        potValMeas_V[iCmd] = potValSum_V / (float) NumRead;
      }

    }

    // Surface Commands to free-float
    cmdEff[ServoIndx] = 0.0;
    memcpy(cmdEffSerial.data(), cmdEff.data(), cmdEffSerial.size()); // copy
    fmu.WriteMessage(kEffectorDirectCmd, cmdEffSerial.size(), cmdEffSerial.data()); // write commands
        
    // Print the results
    std::cout << "Calibration Complete" << std::endl;
    std::cout << "%% Surface: " << std::endl;
    std::cout << "% Servo: " << ServoIndx << std::endl;
    std::cout << "CmdList_ = [" << CmdList_.transpose() << "];" << std::endl;
    std::cout << "incAngleMeas_rad = [" << incAngleMeas_deg.transpose() << "] * d2r;" << std::endl;
    std::cout << "fitOrderServo = 1; % Start with Linear, alter as required" << std::endl;
    std::cout << "[polyCoefServo, cmdZero] = ServoCal(CmdList_, incAngleMeas_rad, fitOrderServo);" << std::endl;
    if(AnalogIndx >= 0){
      std::cout << std::endl;
      std::cout << "% Pot: " << AnalogIndx << std::endl;
      std::cout << "incAngleMeas_rad = [" << incAngleMeas_deg.transpose() << "] * d2r;" << std::endl;
      std::cout << "potValMeas_V = [" << potValMeas_V.transpose() << "];" << std::endl;
      std::cout << "fitOrderPot = 1; % Start with Linear, alter as required" << std::endl;
      std::cout << "[polyCoefPot] = PotCal(incAngleMeas_rad, potValMeas_V, fitOrderPot);" << std::endl;
    }
    std::cout << std::endl;
    
	return 0;
	
  }

}

