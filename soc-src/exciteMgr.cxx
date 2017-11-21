
/*
Excite System Manager - Defines Excitations, Computes commands

See: LICENSE.md for Copyright and License Agreement

History:
2017-11-19 - Chris Regan - Created
*/

#include "exciteMgr.hxx"

void ExciteMgr::Init()
{
  uint8_t numChan = 0;
  uint8_t numElem = 0;

  VecChan timeStart_s; 
  VecChan timeOnePulse_s;
  VecChan amp_nd;

  VecChan timeStart_s, timeDur_s, timeOnePulse_s, amp_rad;
  VecChan freqStart_rps, freqEnd_rps, freqLow_rps, freqHigh_rps;

  VecElem freqVec_rps, phaseVec_rad, ampVec_rad;
  MatChanElem freqMat_rps, phaseMat_rad, ampMat_rad;

  VecChan freqVecStart_rps, freqVecEnd_rps;

  // Excitation 1: Orthogonal Multi-Sine, Elevator
  timeStart_s = 1.0; timeDur_s = 20.0; amp_rad = 4 * kD2R;

  numChan = 1;
  numElem = 6;

  freqMat_rps.conservativeResize(numChan, numElem);
  phaseMat_rad.conservativeResize(numChan, numElem);
  ampMat_rad.conservativeResize(numChan, numElem);

  freqMat_rps << 1, 10, 20, 30, 40, 50;
  phaseMat_rad << 0,0,0,0,0,0;
  ampMat_rad.Constant(numChan, numElem, amp_rad);

  timeVecStart_s.Constant(numChan, timeStart_s);
  timeVecDur_s.Constant(numChan, timeDur_s);

  exciteTest01_.Init(OMS, timeVecStart_s, timeVecDur_s, freqMat_rps, phaseMat_rad, ampMat_rad);

  // Excitation 2: Orthogonal Multi-Sine, Ailerons
  exciteTest02_.Init(OMS, timeVecStart_s, timeVecDur_s, freqMat_rps, phaseMat_rad, ampMat_rad);

  // Excitation 3: Orthogonal Multi-Sine, Rudder
  exciteTest03_.Init(OMS, timeVecStart_s, timeVecDur_s, freqMat_rps, phaseMat_rad, ampMat_rad);

  // Excitation 4: Orthogonal Multi-Sine, Elevator and Ailerons
  timeStart_s = 1.0; timeDur_s = 20.0; amp_rad = 4 * kD2R;

  numChan = 2;
  numElem = 6;

  freqMat_rps.conservativeResize(numChan, numElem);
  phaseMat_rad.conservativeResize(numChan, numElem);
  ampMat_rad.conservativeResize(numChan, numElem);

  freqMat_rps << 1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 
                 3.0, 4.0, 5.0, 6.0, 7.0, 8.0;
  phaseMat_rad << 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                  0.0, 0.0, 0.0, 0.0, 0.0, 0.0;
  ampMat_rad.Constant(numChan, numElem, amp_rad);

  timeVecStart_s.Constant(numChan, timeStart_s);
  timeVecDur_s.Constant(numChan, timeDur_s);

  exciteTest04_.Init(OMS, timeVecStart_s, timeVecDur_s, freqMat_rps, phaseMat_rad, ampMat_rad);

  // Excitation 5: Orthogonal Multi-Sine, Elevator, Ailerons, and Rudder
  timeStart_s = 1.0; timeDur_s = 20.0; amp_rad = 4 * kD2R;

  numChan = 3;
  numElem = 6;

  freqMat_rps.conservativeResize(numChan, numElem);
  phaseMat_rad.conservativeResize(numChan, numElem);
  ampMat_rad.conservativeResize(numChan, numElem);
  freqMat_rps << 1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 
                 2.0, 3.0, 4.0, 5.0, 6.0, 7.0, 
                 3.0, 4.0, 5.0, 6.0, 7.0, 8.0;
  phaseMat_rad << 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                  0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                  0.0, 0.0, 0.0, 0.0, 0.0, 0.0;
  ampMat_rad.Constant(numChan, numElem, amp_rad);

  timeVecStart_s.Constant(numChan, timeStart_s);
  timeVecDur_s.Constant(numChan, timeDur_s);

  exciteTest05_.Init(OMS, timeVecStart_s, timeVecDur_s, freqMat_rps, phaseMat_rad, ampMat_rad);

  // Excitation 6: Chrip, Elevator
  timeStart_s = 1.0; timeDur_s = 20.0; amp_rad = 4 * kD2R;
  freqLow_rps = 1 * kHz2Rps; freqHigh_rps = 50 * kHz2Rps;

  numChan = 1;

  timeVecStart_s.Constant(numChan, timeStart_s);
  timeVecDur_s.Constant(numChan, timeDur_s);
  freqVecStart_rps << freqLow_rps;
  freqVecEnd_rps << freqHigh_rps;
  ampVecStart_nd.Constant(numChan, amp_rad);
  ampVecEnd_nd.Constant(numChan, amp_rad);

  exciteTest06_.Init(Linear, timeVecStart_s, timeVecDur_s, freqVecStart_rps, freqVecEnd_rps, ampVecStart_nd, ampVecEnd_nd);

  // Excitation 7: Chirp, Aileron
  exciteTest07_.Init(Linear, timeVecStart_s, timeVecDur_s, freqVecStart_rps, freqVecEnd_rps, ampVecStart_nd, ampVecEnd_nd);

  // Excitation 8: Chirp, Rudder
  exciteTest08_.Init(Linear, timeVecStart_s, timeVecDur_s, freqVecStart_rps, freqVecEnd_rps, ampVecStart_nd, ampVecEnd_nd);

  // Excitation 9: Simultaneous Chirp, Elevator (increase freq) and Ailerons (decreasing freq)
  timeStart_s = 1.0; timeDur_s = 20.0; amp_rad = 4 * kD2R;
  freqLow_rps = 1 * kHz2Rps; freqHigh_rps = 50 * kHz2Rps;

  numChan = 2;

  timeVecStart_s.Constant(numChan, timeStart_s);
  timeVecDur_s.Constant(numChan, timeDur_s);
  freqVecStart_rps << freqLow_rps, freqHigh_rps;
  freqVecEnd_rps << freqHigh_rps, freqLow_rps;
  ampVecStart_nd.Constant(numChan, amp_rad);
  ampVecEnd_nd.Constant(numChan, amp_rad);

  exciteTest09_.Init(Linear, timeVecStart_s, timeVecDur_s, freqVecStart_rps, freqVecEnd_rps, ampVecStart_nd, ampVecEnd_nd);

 // Excitation 10: Doublet, Elevator
  timeStart_s = 1.0; timeOnePulse_s = 0.23; amp_rad = 4 * kD2R;

  numChan = 1;

  timeVecStart_s.Constant(numChan, timeStart_s);
  timeVecOnePulse_s.Constant(numChan, timeOnePulse_s);
  ampVec_nd.Constant(numChan, amp_rad);

  exciteTest10_.Init(Doublet, timeStart_s, timeOnePulse_s, ampVec_nd);

  // Excitation 11: Doublet, Aileron
  timeStart_s = 1.0; timeOnePulse_s = 0.54; amp_rad = 4 * kD2R;

  numChan = 1;

  timeVecStart_s.Constant(numChan, timeStart_s);
  timeVecOnePulse_s.Constant(numChan, timeOnePulse_s);
  ampVec_nd.Constant(numChan, amp_rad);

  exciteTest11_.Init(Doublet, timeStart_s, timeOnePulse_s, ampVec_nd);

  // Excitation 12: Doublet, Rudder
  exciteTest12_.Init(Doublet, timeStart_s, timeOnePulse_s, ampVec_nd);

  // Excitation 13: Pulse, Elevator
  timeStart_s = 1.0; timeOnePulse_s = 0.23; amp_rad = 4 * kD2R;

  numChan = 1;

  timeVecStart_s.Constant(numChan, timeStart_s);
  timeVecOnePulse_s.Constant(numChan, timeOnePulse_s);
  ampVec_nd.Constant(numChan, amp_rad);

  exciteTest13_.Init(Pulse, timeStart_s, timeOnePulse_s, ampVec_nd);

  // Excitation 14: Pulse, Aileron
  timeStart_s = 1.0; timeOnePulse_s = 0.54; amp_rad = 4 * kD2R;

  numChan = 1;

  timeVecStart_s.Constant(numChan, timeStart_s);
  timeVecOnePulse_s.Constant(numChan, timeOnePulse_s);
  ampVec_nd.Constant(numChan, amp_rad);

  exciteTest14_.Init(Pulse, timeStart_s, timeOnePulse_s, ampVec_nd);
  // Excitation 15: Pulse, Rudder

  exciteTest15_.Init(Pulse, timeStart_s, timeOnePulse_s, ampVec_nd);

  // Excitation 16: Pulse, Aileron (wider)
  timeStart_s = 1.0; timeOnePulse_s = 0.6; amp_rad = 25 * kD2R;

  numChan = 1;

  timeVecStart_s.Constant(numChan, timeStart_s);
  timeVecOnePulse_s.Constant(numChan, timeOnePulse_s);
  ampVec_nd.Constant(numChan, amp_rad);

  exciteTest16_.Init(Pulse, timeStart_s, timeOnePulse_s, ampVec_nd);

  // Excitation 17: Doublet3211, Elevator
  timeStart_s = 1.0; timeOnePulse_s = 0.23; amp_rad = 4 * kD2R;

  numChan = 1;

  timeVecStart_s.Constant(numChan, timeStart_s);
  timeVecOnePulse_s.Constant(numChan, timeOnePulse_s);
  ampVec_nd.Constant(numChan, amp_rad);

  exciteTest17_.Init(Doublet3211, timeStart_s, timeOnePulse_s, ampVec_nd);

  // Excitation 18: Doublet3211, Aileron
  timeStart_s = 1.0; timeOnePulse_s = 0.54; amp_rad = 4 * kD2R;

  numChan = 1;

  timeVecStart_s.Constant(numChan, timeStart_s);
  timeVecOnePulse_s.Constant(numChan, timeOnePulse_s);
  ampVec_nd.Constant(numChan, amp_rad);

  exciteTest18_.Init(Doublet3211, timeStart_s, timeOnePulse_s, ampVec_nd);

  // Excitation 19: Doublet3211, Rudder
  exciteTest19_.Init(Doublet3211, timeStart_s, timeOnePulse_s, ampVec_nd);
}


VecCmd ExciteMgr::GetSignal()
{
  cmdExcite = cmdExcite_;

  return cmdExcite;
}


bool ExciteMgr::Run(bool exciteMode, int indxTest, float time_s)
{
  cmdPitch_ = 0.0;
  cmdRoll_ = 0.0;
  cmdYaw_ = 0.0;
  cmdThrottle_ = 0.0;

  if (exciteMode == 1) {
    switch (indxTest) {
      case 0:
        break;
      case 1:
        exciteFlag = exciteTest01_.Compute(time_s, cmdExcite_);
        cmdPitch_ = cmdExcite_[0];
        break;
      case 2:
        exciteFlag = exciteTest02_.Compute(time_s, cmdExcite_);
        cmdRoll_ = cmdExcite_[0];
        break;
      case 3:
        exciteFlag = exciteTest03_.Compute(time_s, cmdExcite_);
        cmdYaw_ = cmdExcite_[0];
        break;
      case 4:
        exciteFlag = exciteTest04_.Compute(time_s, cmdExcite_);
        cmdPitch_ = cmdExcite_[0];
        cmdRoll_ = cmdExcite_[1];
        break;
      case 5:
        exciteFlag = exciteTest05_.Compute(time_s, cmdExcite_);
        cmdPitch_ = cmdExcite_[0];
        cmdRoll_ = cmdExcite_[1];
        cmdYaw_ = cmdExcite_[2];
        break;
      case 6:
        exciteFlag = exciteTest06_.Compute(time_s, cmdExcite_);
        cmdPitch_ = cmdExcite_[0];
        break;
      case 7:
        exciteFlag = exciteTest07_.Compute(time_s, cmdExcite_);
        cmdRoll_ = cmdExcite_[0];
        break;
      case 8:
        exciteFlag = exciteTest08_.Compute(time_s, cmdExcite_);
        cmdYaw_ = cmdExcite_[0];
        break;
      case 9:
        exciteFlag = exciteTest09_.Compute(time_s, cmdExcite_);
        cmdPitch_ = cmdExcite_[0];
        cmdRoll_ = cmdExcite_[1];
        break;
      case 10:
        exciteFlag = exciteTest10_.Compute(time_s, cmdExcite_);
        cmdPitch_ = cmdExcite_[0];
        break;
      case 11:
        exciteFlag = exciteTest11_.Compute(time_s, cmdExcite_);
        cmdRoll_ = cmdExcite_[0];
        break;
      case 12:
        exciteFlag = exciteTest12_.Compute(time_s, cmdExcite_);
        cmdYaw_ = cmdExcite_[0];
        break;
      case 13:
        exciteFlag = exciteTest13_.Compute(time_s, cmdExcite_);
        cmdPitch_ = cmdExcite_[0];
        break;
      case 14:
        exciteFlag = exciteTest14_.Compute(time_s, cmdExcite_);
        cmdRoll_ = cmdExcite_[0];
        break;
      case 15:
        exciteFlag = exciteTest15_.Compute(time_s, cmdExcite_);
        cmdYaw_ = cmdExcite_[0];
        break;
      case 16:
        exciteFlag = exciteTest16_.Compute(time_s, cmdExcite_);
        cmdRoll_ = cmdExcite_[0];
        break;
      case 17:
        exciteFlag = exciteTest17_.Compute(time_s, cmdExcite_);
        cmdPitch_ = cmdExcite_[0];
        break;
      case 18:
        exciteFlag = exciteTest18_.Compute(time_s, cmdExcite_);
        cmdRoll_ = cmdExcite_[0];
        break;
      case 19:
        exciteFlag = exciteTest19_.Compute(time_s, cmdExcite_);
        cmdYaw_ = cmdExcite_[0];
        break;

    }

  }

  return exciteFlag;

}
