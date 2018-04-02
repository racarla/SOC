/*

*/

#include "inclinometer.hxx"
#include <iostream>


void printVec (float arg[], int length) {
  for (int i = 0; i < length; ++i) {
    if (i > 0) { std::cout << ", ";}
    std::cout << arg[i];
  }
}

int main() {

  /* initialize classes */
  Incline InclineSens;

  /* initialize structures */
  InclineData IncMeas;
    
  /* Setup the Inclinometer */
  InclineSens.SetDamping();
  
  /* main loop */
  while (1) {

    // Number of iteration to sample
    int numRead = 100;
    
    // Establish Zero Angle
    float IncAngleSum = 0.0;
    for (int iRead = 0; iRead < numRead; ++iRead) {

      // Delay
      usleep (1000);

      // Read inclinometer
      InclineSens.GetAngle(&IncMeas);
      
      // Accumulate Values
      IncAngleSum += IncMeas.Angle_deg;
      }

    // Average values, define as zero angle
    float incAngleZero = IncAngleSum / numRead;

    std::cout << "Zero Angle: " << incAngleZero << std::endl;

  }

	return 0;
}

