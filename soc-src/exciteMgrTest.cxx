/*
Simple excitation manager tester

See: LICENSE.md for Copyright and License Agreement

History:
2017-11-21 - Chris Regan - Created
*/

#include <iostream>
#include "exciteMgr.hxx"


int main(void)
{
  // Define Excitation Manager and Excitations
  ExciteMgr exciteMgr; // Create the Excitation Manager
  exciteMgr.Init();    // Initialize the Excitation Manager

  VecChan cmdExcite;

  float timeStep_s = 1.0/50.0;
  float timeEnd_s = 6;

  int numIter = (int) (timeEnd_s / timeStep_s); // Number of Iterations

  uint8_t exciteMode = 1;
  uint8_t indxTest = 17;

  std::cout << "time_s" << "\t" << "excite_nd" << std::endl;
  float time_s = 0.0;
  for(int iIter = 0 ; iIter < numIter ; iIter++){
    time_s = (float) iIter * timeStep_s;

    cmdExcite = exciteMgr.Compute(exciteMode, indxTest, time_s);
  
    std::cout << time_s << "\t" << cmdExcite.transpose() << std::endl;
  }

  indxTest = 19;

  std::cout << "time_s" << "\t" << "excite_nd" << std::endl;
  time_s = 0.0;
  for(int iIter = 0 ; iIter < numIter ; iIter++){
    if (iIter < 10) {exciteMode = 0;} else {exciteMode = 1;}


    time_s = (float) iIter * timeStep_s + 10;

    cmdExcite = exciteMgr.Compute(exciteMode, indxTest, time_s);
  
    std::cout << time_s << "\t" << cmdExcite.transpose() << std::endl;
  }
}
