
/*
Excite System Manager - Defines Excitations, Computes commands

See: LICENSE.md for Copyright and License Agreement

History:
2017-11-19 - Chris Regan - Created
*/

#include "exciteMgr.hxx"
#include <iostream>

void ExciteMgr::Init()
{

  const float kD2R = M_PI / 180.0;

  uint8_t numChan = 0;
  uint8_t numElem = 0;

  float timeEngage_s_ = 0;

  float timeStart_s;
  float timeDur_s;
  float timeOnePulse_s;
  float amp_nd;

  cmdExcite_.setZero(4); // FIXIT - Hard coded size

  VecChan timeVecStart_s, timeVecDur_s, timeVecOnePulse_s, ampVec_nd;

  MatChanElem freqMat_rps, phaseMat_rad, ampMat_nd;

  float freqLow_rps, freqHigh_rps;
  VecChan freqVecStart_rps, freqVecEnd_rps;
  VecChan ampVecStart_nd, ampVecEnd_nd;

  // Excitation 1: Orthogonal Multi-Sine, Elevator
  timeStart_s = 1.0; timeDur_s = 20.0; amp_nd = 4 * kD2R;

  numChan = 1;
  numElem = 6;

  freqMat_rps.conservativeResize(numChan, numElem);
  phaseMat_rad.conservativeResize(numChan, numElem);
  ampMat_nd.conservativeResize(numChan, numElem);

  freqMat_rps << 1, 10, 20, 30, 40, 50;
  phaseMat_rad << 0, 0, 0, 0, 0, 0;
  ampMat_nd.setConstant(numChan, numElem, amp_nd);

  timeVecStart_s.setConstant(numChan, timeStart_s);
  timeVecDur_s.setConstant(numChan, timeDur_s);

  exciteTest01_.Init(kOMS, timeVecStart_s, timeVecDur_s, freqMat_rps, phaseMat_rad, ampMat_nd);

  // Excitation 2: Orthogonal Multi-Sine, Ailerons
  exciteTest02_.Init(kOMS, timeVecStart_s, timeVecDur_s, freqMat_rps, phaseMat_rad, ampMat_nd);

  // Excitation 3: Orthogonal Multi-Sine, Rudder
  exciteTest03_.Init(kOMS, timeVecStart_s, timeVecDur_s, freqMat_rps, phaseMat_rad, ampMat_nd);

  // Excitation 4: Orthogonal Multi-Sine, Elevator and Ailerons
  timeStart_s = 1.0; timeDur_s = 20.0; amp_nd = 4 * kD2R;

  numChan = 2;
  numElem = 6;

  freqMat_rps.conservativeResize(numChan, numElem);
  phaseMat_rad.conservativeResize(numChan, numElem);
  ampMat_nd.conservativeResize(numChan, numElem);

  freqMat_rps << 1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 
                 3.0, 4.0, 5.0, 6.0, 7.0, 8.0;
  phaseMat_rad << 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                  0.0, 0.0, 0.0, 0.0, 0.0, 0.0;
  ampMat_nd.setConstant(numChan, numElem, amp_nd);

  timeVecStart_s.setConstant(numChan, timeStart_s);
  timeVecDur_s.setConstant(numChan, timeDur_s);

  exciteTest04_.Init(kOMS, timeVecStart_s, timeVecDur_s, freqMat_rps, phaseMat_rad, ampMat_nd);

  // Excitation 5: Orthogonal Multi-Sine, Elevator, Ailerons, and Rudder
  timeStart_s = 1.0; timeDur_s = 20.0; amp_nd = 4 * kD2R;

  numChan = 3;
  numElem = 6;

  freqMat_rps.conservativeResize(numChan, numElem);
  phaseMat_rad.conservativeResize(numChan, numElem);
  ampMat_nd.conservativeResize(numChan, numElem);
  freqMat_rps << 1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 
                 2.0, 3.0, 4.0, 5.0, 6.0, 7.0, 
                 3.0, 4.0, 5.0, 6.0, 7.0, 8.0;
  phaseMat_rad << 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                  0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                  0.0, 0.0, 0.0, 0.0, 0.0, 0.0;
  ampMat_nd.setConstant(numChan, numElem, amp_nd);

  timeVecStart_s.setConstant(numChan, timeStart_s);
  timeVecDur_s.setConstant(numChan, timeDur_s);

  exciteTest05_.Init(kOMS, timeVecStart_s, timeVecDur_s, freqMat_rps, phaseMat_rad, ampMat_nd);

  // Excitation 6: Chrip, Elevator
  timeStart_s = 1.0; timeDur_s = 20.0; amp_nd = 4 * kD2R;
  freqLow_rps = 1 * kHz2Rps; freqHigh_rps = 50 * kHz2Rps;

  numChan = 1;

  timeVecStart_s.setConstant(numChan, timeStart_s);
  timeVecDur_s.setConstant(numChan, timeDur_s);
  freqVecStart_rps.setConstant(numChan, freqLow_rps);
  freqVecEnd_rps.setConstant(numChan, freqHigh_rps);
  ampVecStart_nd.setConstant(numChan, amp_nd);
  ampVecEnd_nd.setConstant(numChan, amp_nd);

  exciteTest06_.Init(kLinear, timeVecStart_s, timeVecDur_s, freqVecStart_rps, freqVecEnd_rps, ampVecStart_nd, ampVecEnd_nd);

  // Excitation 7: Chirp, Aileron
  exciteTest07_.Init(kLinear, timeVecStart_s, timeVecDur_s, freqVecStart_rps, freqVecEnd_rps, ampVecStart_nd, ampVecEnd_nd);

  // Excitation 8: Chirp, Rudder
  exciteTest08_.Init(kLinear, timeVecStart_s, timeVecDur_s, freqVecStart_rps, freqVecEnd_rps, ampVecStart_nd, ampVecEnd_nd);

  // Excitation 9: Simultaneous Chirp, Elevator (increase freq) and Ailerons (decreasing freq)
  timeStart_s = 1.0; timeDur_s = 20.0; amp_nd = 4 * kD2R;
  freqLow_rps = 1 * kHz2Rps; freqHigh_rps = 50 * kHz2Rps;

  numChan = 2;

  timeVecStart_s.setConstant(numChan, timeStart_s);
  timeVecDur_s.setConstant(numChan, timeDur_s);
  freqVecStart_rps.setConstant(numChan, freqLow_rps); freqVecStart_rps[1] = freqHigh_rps;
  freqVecEnd_rps.setConstant(numChan, freqHigh_rps); freqVecEnd_rps[1] = freqLow_rps;
  ampVecStart_nd.setConstant(numChan, amp_nd);
  ampVecEnd_nd.setConstant(numChan, amp_nd);

  exciteTest09_.Init(kLinear, timeVecStart_s, timeVecDur_s, freqVecStart_rps, freqVecEnd_rps, ampVecStart_nd, ampVecEnd_nd);

 // Excitation 10: Doublet, Elevator
  timeStart_s = 1.0; timeOnePulse_s = 0.23; amp_nd = 4 * kD2R;

  numChan = 1;

  timeVecStart_s.setConstant(numChan, timeStart_s);
  timeVecOnePulse_s.setConstant(numChan, timeOnePulse_s);
  ampVec_nd.setConstant(numChan, amp_nd);

  exciteTest10_.Init(kDoublet, timeVecStart_s, timeVecOnePulse_s, ampVec_nd);

  // Excitation 11: Doublet, Aileron
  timeStart_s = 1.0; timeOnePulse_s = 0.54; amp_nd = 4 * kD2R;

  numChan = 1;

  timeVecStart_s.setConstant(numChan, timeStart_s);
  timeVecOnePulse_s.setConstant(numChan, timeOnePulse_s);
  ampVec_nd.setConstant(numChan, amp_nd);

  exciteTest11_.Init(kDoublet, timeVecStart_s, timeVecOnePulse_s, ampVec_nd);

  // Excitation 12: Doublet, Rudder
  exciteTest12_.Init(kDoublet, timeVecStart_s, timeVecOnePulse_s, ampVec_nd);

  // Excitation 13: Pulse, Elevator
  timeStart_s = 1.0; timeOnePulse_s = 0.23; amp_nd = 4 * kD2R;

  numChan = 1;

  timeVecStart_s.setConstant(numChan, timeStart_s);
  timeVecOnePulse_s.setConstant(numChan, timeOnePulse_s);
  ampVec_nd.setConstant(numChan, amp_nd);

  exciteTest13_.Init(kPulse, timeVecStart_s, timeVecOnePulse_s, ampVec_nd);

  // Excitation 14: Pulse, Aileron
  timeStart_s = 1.0; timeOnePulse_s = 0.54; amp_nd = 4 * kD2R;

  numChan = 1;

  timeVecStart_s.setConstant(numChan, timeStart_s);
  timeVecOnePulse_s.setConstant(numChan, timeOnePulse_s);
  ampVec_nd.setConstant(numChan, amp_nd);

  exciteTest14_.Init(kPulse, timeVecStart_s, timeVecOnePulse_s, ampVec_nd);
  // Excitation 15: Pulse, Rudder

  exciteTest15_.Init(kPulse, timeVecStart_s, timeVecOnePulse_s, ampVec_nd);

  // Excitation 16: Pulse, Aileron (wider)
  timeStart_s = 1.0; timeOnePulse_s = 0.6; amp_nd = 25 * kD2R;

  numChan = 1;

  timeVecStart_s.setConstant(numChan, timeStart_s);
  timeVecOnePulse_s.setConstant(numChan, timeOnePulse_s);
  ampVec_nd.setConstant(numChan, amp_nd);

  exciteTest16_.Init(kPulse, timeVecStart_s, timeVecOnePulse_s, ampVec_nd);

  // Excitation 17: Doublet3211, Elevator
  timeStart_s = 1.0; timeOnePulse_s = 0.23; amp_nd = 4 * kD2R;

  numChan = 1;

  timeVecStart_s.setConstant(numChan, timeStart_s);
  timeVecOnePulse_s.setConstant(numChan, timeOnePulse_s);
  ampVec_nd.setConstant(numChan, amp_nd);

  exciteTest17_.Init(kDoublet3211, timeVecStart_s, timeVecOnePulse_s, ampVec_nd);

  // Excitation 18: Doublet3211, Aileron
  timeStart_s = 1.0; timeOnePulse_s = 0.54; amp_nd = 4 * kD2R;

  numChan = 1;

  timeVecStart_s.setConstant(numChan, timeStart_s);
  timeVecOnePulse_s.setConstant(numChan, timeOnePulse_s);
  ampVec_nd.setConstant(numChan, amp_nd);

  exciteTest18_.Init(kDoublet3211, timeVecStart_s, timeVecOnePulse_s, ampVec_nd);

  // Excitation 19: Doublet3211, Rudder
  exciteTest19_.Init(kDoublet3211, timeVecStart_s, timeVecOnePulse_s, ampVec_nd);
}

VecChan ExciteMgr::Compute(bool exciteMode, int indxTest, float time_s)
{
  float timeExcite_s = 0;

  VecChan cmdExciteTemp(4);
  cmdExciteTemp.setZero();
  cmdExcite_.setZero();
  

  if (exciteMode == 1) {

    timeExcite_s = time_s - timeEngage_s_;

    switch (indxTest) {
      case 0:
        break;
      case 1:
        cmdExciteTemp = exciteTest01_.Compute(timeExcite_s);
        cmdExcite_[1] = cmdExciteTemp[0];
        break;
      case 2:
        cmdExciteTemp = exciteTest02_.Compute(timeExcite_s);
        cmdExcite_[0] = cmdExciteTemp[0];
        break;
      case 3:
        cmdExciteTemp = exciteTest03_.Compute(timeExcite_s);
        cmdExcite_[2] = cmdExciteTemp[0];
        break;
      case 4:
        cmdExciteTemp = exciteTest04_.Compute(timeExcite_s);
        cmdExcite_[0] = cmdExciteTemp[0];
        cmdExcite_[1] = cmdExciteTemp[1];
        break;
      case 5:
        cmdExciteTemp = exciteTest05_.Compute(timeExcite_s);
        cmdExcite_[0] = cmdExciteTemp[0];
        cmdExcite_[1] = cmdExciteTemp[1];
        cmdExcite_[2] = cmdExciteTemp[2];
        break;
      case 6:
        cmdExciteTemp = exciteTest06_.Compute(timeExcite_s);
        cmdExcite_[1] = cmdExciteTemp[0];
        break;
      case 7:
        cmdExciteTemp = exciteTest07_.Compute(timeExcite_s);
        cmdExcite_[0] = cmdExciteTemp[0];
        break;
      case 8:
        cmdExciteTemp = exciteTest08_.Compute(timeExcite_s);
        cmdExcite_[2] = cmdExciteTemp[0];
        break;
      case 9:
        cmdExciteTemp = exciteTest09_.Compute(timeExcite_s);
        cmdExcite_[0] = cmdExciteTemp[0];
        cmdExcite_[1] = cmdExciteTemp[1];
        break;
      case 10:
        cmdExciteTemp = exciteTest10_.Compute(timeExcite_s);
        cmdExcite_[1] = cmdExciteTemp[0];
        break;
      case 11:
        cmdExciteTemp = exciteTest11_.Compute(timeExcite_s);
        cmdExcite_[0] = cmdExciteTemp[0];
        break;
      case 12:
        cmdExciteTemp = exciteTest12_.Compute(timeExcite_s);
        cmdExcite_[2] = cmdExciteTemp[0];
        break;
      case 13:
        cmdExciteTemp = exciteTest13_.Compute(timeExcite_s);
        cmdExcite_[1] = cmdExciteTemp[0];
        break;
      case 14:
        cmdExciteTemp = exciteTest14_.Compute(timeExcite_s);
        cmdExcite_[0] = cmdExciteTemp[0];
        break;
      case 15:
        cmdExciteTemp = exciteTest15_.Compute(timeExcite_s);
        cmdExcite_[2] = cmdExciteTemp[0];
        break;
      case 16:
        cmdExciteTemp = exciteTest16_.Compute(timeExcite_s);
        cmdExcite_[0] = cmdExciteTemp[0];
        break;
      case 17:
        cmdExciteTemp = exciteTest17_.Compute(timeExcite_s);
        cmdExcite_[1] = cmdExciteTemp[0];
        break;
      case 18:
        cmdExciteTemp = exciteTest18_.Compute(timeExcite_s);
        cmdExcite_[0] = cmdExciteTemp[0];
        break;
      case 19:
        cmdExciteTemp = exciteTest19_.Compute(timeExcite_s);
        cmdExcite_[2] = cmdExciteTemp[0];
        break;

    }

  } else {

    timeEngage_s_ = time_s;

  }

  return cmdExcite_;

}
