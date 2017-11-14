/*
Calibration routine

See: LICENSE.md for Copyright and License Agreement

History:
2017-11-12 - Chris Regan - Created
*/

#include "navigation.hxx"
#include "datalogger.hxx"
#include "config.hxx"
#include "fmu.hxx"
#include "hardware-defs.hxx"
#include "global-defs.hxx"
#include "inclinometer.hxx"
#include <iostream>


void printVec (float arg[], int length) {
  for (int i = 0; i < length; ++i) {
    if (i > 0) { std::cout << ", ";}
    std::cout << arg[i];
  }
}

int main(int argc, char* argv[]) {
  if (argc!=2) {
    std::cerr << "ERROR: Incorrect number of input arguments." << std::endl;
    return -1;
  }

  /* initialize classes */
  Fmu Sensors;
  Datalogger Log;
  Navigation NavFilter;
  Incline InclineSens;

  /* initialize structures */
  AircraftConfig Config;
  FmuData Data;
  NavigationData NavData;
  InclineData IncMeas;
  
  /* load configuration file */
  LoadConfigFile(argv[1],Sensors,&Config,&Data);
  
  /* Define the Control Effector Vector */
  std::vector<float> EffectorCmd;
  EffectorCmd.resize(Config.NumberEffectors);
  
  std::vector<uint8_t> EffectorBuffer;
  EffectorBuffer.resize(EffectorCmd.size()*sizeof(float));
  
  /* Setup Inclinometer */
  InclineSens.SetDamping();
  double IncAngle;
  
  // Define Delays
  int DelayMove = 2000000; // delay for 2 second after servo move
  int DelayRead = 50000; // delay for 50 ms between reads
  

  int ServoIndx = -1;
  int AnalogIndx = -1;
  
  // Servo Command Definition
  float CmdStart_nd;
  float CmdEnd_nd;
  int NumCmds = 11;
  float CmdList_nd [11] = { };

  float ServoCmdList_pwm[11] = { };
  float ServoBias_pwm = 1500.0;
  float ServoScale_nd = 500.0;

  float junk;
  
  // Surface Commands to free-float
  memset(&EffectorCmd[0], 0.0, EffectorCmd.size() * sizeof EffectorCmd[0]); // set all effectors to zero
  memcpy(EffectorBuffer.data(), EffectorCmd.data(), EffectorBuffer.size()); // copy
  Sensors.WriteMessage(kEffectorAngleCmd, EffectorBuffer.size(), EffectorBuffer.data()); // write commands
  
  std::cout << "The control surface calibration routine requires the Inclinometer connected to the BBB_UART1 " << std::endl;
  std::cout << "The commands sent to the control surface are Non-Dimensional, based on a -1 to +1 total range." << std::endl;
  std::cout << "Ensure the Config for each servo reflects this normalized range." << std::endl;
  std::cout << "Standard PWM Bias: 1500 and Scale 500." << std::endl;

  //Get the SBUS or PWM channel number
  //std::cout << "Enter the Servo Index Number: ";
  //std::cin >> ServoIndx;
  ServoIndx = 1;

  //Get the Pot channel number
  //std::cout << "Enter the Analog Index Number for Surface Pot (-1 to skip): ";
  //std::cin >> AnalogIndx;
  AnalogIndx = 1;

  //Get Starting Command
  //std::cout << "Enter the Starting Command (Non-Dimensional: -1 to 1): ";
  //std::cin >> CmdStart_nd;
  CmdStart_nd = -0.25;

  //Get Ending Command
  //std::cout << "Enter the Ending Command (Non-Dimensional: -1 to 1): ";
  //std::cin >> CmdEnd_nd;
  CmdEnd_nd = 0.25;


  // Generate the list of servo commands
  for (int iCmd = 0; iCmd < NumCmds; ++iCmd) {
    CmdList_nd[iCmd] = (CmdEnd_nd - CmdStart_nd) * float(iCmd)/float(NumCmds) + CmdStart_nd;
    ServoCmdList_pwm[iCmd] = ServoBias_pwm + CmdList_nd[iCmd] * ServoScale_nd;
  }

  // Begin the test
  std::cout << "Beginning Test, Surface: " << ServoIndx << std::endl; 
  
  std::cout << "Turn OFF Servo Power, Hold Surface at Zero Position, Press any key to continue . . . ";
  std::cin >> junk;
  
  usleep(2*DelayMove); // delay
  
  // Number of iteration to sample
  int NumRead = 100;
    
  /* main loop */
  while (1) {
    // Establish Zero Angle
    float IncAngleSum = 0.0;
    for (int iRead = 0; iRead < NumRead; ++iRead) {

      // Delay
      usleep (DelayRead);

      // Read inclinometer
      InclineSens.GetAngle(&IncMeas);
      
      // Accumulate Values
      IncAngleSum += IncMeas.Angle_deg;
      }

    // Average values, define as zero angle
    float incAngleZero = IncAngleSum / NumRead;

    std::cout << "Zero Angle: " << incAngleZero << std::endl;

    std::cout << "Beginning Automated Test..." << std::endl;
    std::cout << "Turn on Servo Power, Press any key to continue . . . ";
    std::cin >> junk;
    
    usleep(2*DelayMove); // delay
    
    std::cout << "Starting... " << std::endl;

    // Loop Command Steps
    float incAngleMeas[11] = { };
    float potValMeas[11] = { };
    
    for (int iCmd = 0; iCmd < NumCmds; ++iCmd) {
      // Command Servo to command
      float cmdCurr = CmdList_nd[iCmd];
      
      std::cout << "Servo Command: " << cmdCurr << std::endl;
      
      // Send the Servo Command
      EffectorCmd[ServoIndx] = cmdCurr;
      memcpy(EffectorBuffer.data(),EffectorCmd.data(),EffectorBuffer.size());
      Sensors.WriteMessage(kEffectorAngleCmd,EffectorBuffer.size(),EffectorBuffer.data());
      
      usleep(DelayMove); // delay
      
      IncAngleSum = 0.0;
      IncAngle = 0.0;
      float potValTemp = 0.0;
      float potValSum = 0.0;
      // Read the Inclinometer and pot data
      for (int iRead = 0; iRead < NumRead; ++iRead) {
        // Keep Sending the same servo command
        memcpy(EffectorBuffer.data(),EffectorCmd.data(),EffectorBuffer.size());
        Sensors.WriteMessage(kEffectorAngleCmd,EffectorBuffer.size(),EffectorBuffer.data());
      
        // Delay
        usleep (DelayRead);

        // Read inclinometer
        InclineSens.GetAngle(&IncMeas);
        
        // Accumulate Values
        IncAngleSum += IncMeas.Angle_deg;
        
        // Read Pot Sensors
        if(AnalogIndx>=0){
          Sensors.GetSensorData(&Data);
        
          potValTemp = Data.Analog[AnalogIndx].Voltage_V;
          potValSum += potValTemp;
          }
        }

      // Average values
      incAngleMeas[iCmd] = (IncAngleSum / NumRead) - incAngleZero; // Average minus the 'Zero'
      if(AnalogIndx>=0){potValMeas[iCmd] = potValSum / NumRead;}

    }

    // Surface Commands to free-float
    memset(&EffectorCmd[0], 0.0, EffectorCmd.size() * sizeof EffectorCmd[0]); // set all effectors to zero
    memcpy(EffectorBuffer.data(), EffectorCmd.data(), EffectorBuffer.size()); // copy
    Sensors.WriteMessage(kEffectorAngleCmd, EffectorBuffer.size(), EffectorBuffer.data()); // write commands
        
    
    std::cout << "Calibration Complete" << std::endl;
    
    std::cout << "%% " << "Surface: " << ServoIndx << std::endl;

    std::cout << "CmdList_nd = [";
    printVec (CmdList_nd, NumCmds);
    std::cout << "];\n" << std::endl;

    std::cout << "ServoCmdList_pwm = [";
    printVec (ServoCmdList_pwm, NumCmds);
    std::cout << "];\n" << std::endl;
    
    std::cout << "incAngleMeas = [";
    printVec (incAngleMeas, NumCmds);
    std::cout << "];\n" << std::endl;

    if(AnalogIndx>=0){
      std::cout << "potValMeas = [";
      printVec (potValMeas, NumCmds);
      std::cout << "];\n" << std::endl;
      }
    
    std::cout << "fitOrder = 2; % Start with Quadratic, alter as required" << std::endl;
    std::cout << "[polyCoefServo, cmdZero] = SurfCal(ServoCmdList_pwm, incAngleMeas, fitOrder);" << std::endl;
    if(AnalogIndx>=0){std::cout << "[polyCoefPot, potZero] = SurfCal(potValMeas, incAngleMeas, fitOrder);" << std::endl;}
    std::cout << "%%" << std::endl;
    
	return 0;
	
  }

}

