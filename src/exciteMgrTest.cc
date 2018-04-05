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

  float tStep_s = 1.0/50.0;
  float tEnd_s = 6;

  int numIter = (int) (tEnd_s / tStep_s); // Number of Iterations

  uint8_t exciteMode = 1;
  uint8_t indxTest = 17;

  std::cout << "t_s" << "\t" << "excite_nd" << std::endl;
  float t_s = 0.0;
  for(int iIter = 0 ; iIter < numIter ; iIter++){
    t_s = (float) iIter * tStep_s;

    cmdExcite = exciteMgr.Compute(exciteMode, indxTest, t_s);

    std::cout << t_s << "\t" << cmdExcite.transpose() << std::endl;
  }

  indxTest = 19;

  std::cout << "t_s" << "\t" << "excite_nd" << std::endl;
  t_s = 0.0;
  for(int iIter = 0 ; iIter < numIter ; iIter++){
    if (iIter < 10) {exciteMode = 0;} else {exciteMode = 1;}


    t_s = (float) iIter * tStep_s + 10;

    cmdExcite = exciteMgr.Compute(exciteMode, indxTest, t_s);

    std::cout << t_s << "\t" << cmdExcite.transpose() << std::endl;
  }
}
