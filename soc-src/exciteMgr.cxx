
/*
Excite System Manager - Defines Excitations, Computes commands

See: LICENSE.md for Copyright and License Agreement

History:
2017-11-19 - Chris Regan - Created
*/

#include "exciteMgr.hxx"

void ExciteMgr::Init()
{
  timeEngage_s_ = 0;

  const float kD2R = M_PI / 180.0;

  float timeStart_s;
  float timeDur_s;
  float timeOnePulse_s;

  cmdExcite_.setZero(4); // FIXIT - Hard coded size

  VecChan timeVecStart_s, timeVecDur_s, timeVecOnePulse_s, ampVec_nd;

  // Multisine Excitation Setup
  MatChanElem freqMat_rps, phaseMat_rad, ampMat_nd;

  uint8_t numChan = 3;
  uint8_t numElem = 46;
  float amp_nd = 4 * kD2R;

  freqMat_rps.conservativeResize(numChan, numElem);
  phaseMat_rad.conservativeResize(numChan, numElem);
  ampMat_nd.conservativeResize(numChan, numElem);

  freqMat_rps << 0.37699112, 1.5079645, 2.6389378, 3.7699112, 4.9008845, 6.0318579, 7.1628313, 8.2938046, 9.424778, 10.555751, 11.686725, 12.817698, 13.948671, 15.079645, 16.210618, 17.341591, 18.472565, 19.603538, 20.734512, 21.865485, 22.996458, 24.127432, 25.258405, 26.389378, 27.520352, 28.651325, 29.782298, 30.913272, 32.044245, 33.175218, 34.306192, 35.437165, 36.568138, 37.699112, 38.830085, 39.961059, 41.092032, 42.223005, 43.353979, 44.484952, 45.615925, 46.746899, 47.877872, 49.008845, 50.139819, 51.270792, 
                 0.75398224, 1.8849556, 3.0159289, 4.1469023, 5.2778757, 6.408849, 7.5398224, 8.6707957, 9.8017691, 10.932742, 12.063716, 13.194689, 14.325663, 15.456636, 16.587609, 17.718583, 18.849556, 19.980529, 21.111503, 22.242476, 23.373449, 24.504423, 25.635396, 26.766369, 27.897343, 29.028316, 30.159289, 31.290263, 32.421236, 33.55221, 34.683183, 35.814156, 36.94513, 38.076103, 39.207076, 40.33805, 41.469023, 42.599996, 43.73097, 44.861943, 45.992916, 47.12389, 48.254863, 49.385837, 50.51681, 51.647783, 
                 1.1309734, 2.2619467, 3.3929201, 4.5238934, 5.6548668, 6.7858401, 7.9168135, 9.0477868, 10.17876, 11.309734, 12.440707, 13.57168, 14.702654, 15.833627, 16.9646, 18.095574, 19.226547, 20.35752, 21.488494, 22.619467, 23.75044, 24.881414, 26.012387, 27.143361, 28.274334, 29.405307, 30.536281, 31.667254, 32.798227, 33.929201, 35.060174, 36.191147, 37.322121, 38.453094, 39.584067, 40.715041, 41.846014, 42.976988, 44.107961, 45.238934, 46.369908, 47.500881, 48.631854, 49.762828, 50.893801, 52.024774;

  phaseMat_rad << 5.9016971, 6.2538491, 6.1650954, 5.1281552, 4.1778432, 1.6285662, 4.6472567, 3.1525262, 0.59737482, 3.0422108, 4.3487778, 1.057283, 2.4681359, 3.8892176, 4.572762, 4.4076866, 4.2568904, 3.9038365, 3.0647889, 2.0038473, 0.16939258, 4.493568, 1.5003072, 4.9830828, 1.9330228, 4.4933183, 0.39175709, 1.7847209, 3.2378917, 4.082671, 4.8560003, 5.771154, 5.2581741, 3.9548764, 3.2079124, 0.88731294, 0.19942676, 4.1682276, 0.80510679, 3.1808851, 0.52565461, 3.3325046, 4.9277457, 5.97419, 0.47236376, 0.49175571,
                  0.014002804, 5.9246153, 5.7092199, 4.7799348, 3.8593411, 1.6442684, 5.2979782, 2.6258336, 5.1595102, 1.9999655, 3.0089759, 0.10668298, 1.1074845, 1.8899931, 3.1080334, 2.61301, 2.6675991, 2.1992089, 1.2138536, 6.2414265, 3.5767561, 2.3398115, 6.0598285, 2.9409163, 5.7325057, 1.2230763, 3.8703195, 5.0745416, 0.57376572, 1.3722093, 1.4986473, 2.166559, 1.5280901, 0.3251131, 5.8281177, 3.3081948, 2.3525631, 5.6002565, 3.2288363, 6.2020718, 2.6694611, 5.5588073, 0.38239456, 1.1760769, 2.6580879, 2.5563042,
                  6.2602334, 6.1547622, 5.2585879, 5.0002279, 3.0932728, 1.5189961, 5.0744861, 2.8854728, 5.4922339, 1.7069495, 4.5138175, 0.74718441, 2.1882407, 2.0680044, 3.1615818, 2.4686575, 2.9614832, 1.467005, 1.6882923, 5.7396093, 3.6801636, 2.2753249, 0.10608188, 3.1185444, 5.2924911, 1.437739, 3.7808623, 5.1409061, 0.39154347, 1.275829, 1.444546, 1.7425124, 1.4288283, 0.76206931, 5.4719223, 3.5979659, 2.2381699, 5.961423, 2.9512973, 6.2545468, 1.9831563, 4.1344995, 5.3139185, 0.89666996, 1.4347399, 2.3232479;

  ampMat_nd.setConstant(numChan, numElem, amp_nd);

  // Chirp Setup
  float freqLow_rps, freqHigh_rps;
  VecChan freqVecStart_rps, freqVecEnd_rps;
  VecChan ampVecStart_nd, ampVecEnd_nd;

  // Excitation 1: Orthogonal Multi-Sine, Elevator
  timeStart_s = 1.0; timeDur_s = 20.0;

  numChan = 1;
  numElem = 46;

  timeVecStart_s.setConstant(numChan, timeStart_s);
  timeVecDur_s.setConstant(numChan, timeDur_s);

  exciteTest01_.Init(kOMS, timeVecStart_s, timeVecDur_s, freqMat_rps.block(0, 0, 1, numElem), phaseMat_rad.block(0, 0, 1, numElem), ampMat_nd.block(0, 0, 1, numElem));

  // Excitation 2: Orthogonal Multi-Sine, Ailerons
  exciteTest02_.Init(kOMS, timeVecStart_s, timeVecDur_s, freqMat_rps.block(1, 0, 1, numElem), phaseMat_rad.block(1, 0, 1, numElem), ampMat_nd.block(1, 0, 1, numElem));

  // Excitation 3: Orthogonal Multi-Sine, Rudder
  exciteTest03_.Init(kOMS, timeVecStart_s, timeVecDur_s, freqMat_rps.block(2, 0, 1, numElem), phaseMat_rad.block(2, 0, 1, numElem), ampMat_nd.block(2, 0, 1, numElem));

  // Excitation 4: Orthogonal Multi-Sine, Elevator and Ailerons
  timeStart_s = 1.0; timeDur_s = 20.0; amp_nd = 4 * kD2R;

  numChan = 2;
  numElem = 46;

  timeVecStart_s.setConstant(numChan, timeStart_s);
  timeVecDur_s.setConstant(numChan, timeDur_s);

  exciteTest04_.Init(kOMS, timeVecStart_s, timeVecDur_s, freqMat_rps.block(0, 0, 2, numElem), phaseMat_rad.block(0, 0, 2, numElem), ampMat_nd.block(0, 0, 2, numElem));

  // Excitation 5: Orthogonal Multi-Sine, Elevator, Ailerons, and Rudder
  timeStart_s = 1.0; timeDur_s = 20.0; amp_nd = 4 * kD2R;

  numChan = 3;
  numElem = 46;

  timeVecStart_s.setConstant(numChan, timeStart_s);
  timeVecDur_s.setConstant(numChan, timeDur_s);

  exciteTest05_.Init(kOMS, timeVecStart_s, timeVecDur_s, freqMat_rps, phaseMat_rad, ampMat_nd);

  // Excitation 6: Chrip, Elevator
  timeStart_s = 1.0; timeDur_s = 20.0; amp_nd = 4 * kD2R;
  freqLow_rps = 1; freqHigh_rps = 50;

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

  // Excitation 9: Simultaneous Chirp, Ailerons (decreasing freq) and Elevator (increase freq) 
  timeStart_s = 1.0; timeDur_s = 20.0; amp_nd = 4 * kD2R;
  freqLow_rps = 1; freqHigh_rps = 50;

  numChan = 2;

  timeVecStart_s.setConstant(numChan, timeStart_s);
  timeVecDur_s.setConstant(numChan, timeDur_s);
  freqVecStart_rps.setConstant(numChan, freqHigh_rps); freqVecStart_rps[1] = freqLow_rps;
  freqVecEnd_rps.setConstant(numChan, freqLow_rps); freqVecEnd_rps[1] = freqHigh_rps;
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
