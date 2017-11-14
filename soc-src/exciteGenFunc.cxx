/*
Classes and Functions for Excitation Generation

See: LICENSE.md for Copyright and License Agreement

History:
2017-11-12 - Chris Regan - Created
2017-11-13 - Chris Regan - Function are all single channel, vectorization is handled in the header
*/

#include "exciteGenFunc.hxx"

//
// Single Channel Excitation Signal Generators
//
// Pulse with 1-pulse timeDur_s long
float GenPulse(float &time_s, float &timeDur_s, float &amp_nd)
{
  if (time_s < timeDur_s) {
    excite_nd = amp_nd;

  } else { excite_nd = 0.0; }
  
  return excite_nd;
}

// Doublet 1-1 with 1-pulse timeDur_s long, total of 2*timeDur_s excitation
float GenDoublet(float &time_s, float &timeDur_s, float &amp_nd)
{
  float timeDur2_s = 2.0 * timeDur_s;

  if (time_s < timeDur_s) {
    excite_nd = amp_nd;

  } else if (time_s < timeDur2_s) {
    excite_nd = -amp_nd;

  } else { excite_nd = 0.0; }
  
  return excite_nd;
}

// Doublet 1-2-1 - 1-pulse timeDur_s long, total of 4*timeDur_s excitation
float GenDoublet121(float &time_s, float &timeDur_s, float &amp_nd)
{
  float timeDur2_s = 2.0 * timeDur_s;

  if (time_s < timeDur_s) {
    excite_nd = amp_nd;

  } else if (time_s < timeDur_s + timeDur2_s) {
    excite_nd = -amp_nd;

  } else if (time_s < timeDur_s + timeDur2_s + timeDur_s) {
    excite_nd = amp_nd;

  } else { excite_nd = 0.0; }
  
  return excite_nd;
}

// Doublet 3-2-1-1 - 1-pulse timeDur_s long, total of 7*timeDur_s excitation
float GenDoublet3211(float &time_s, float &timeDur_s, float &amp_nd)
{
  float timeDur2_s = 2.0 * timeDur_s;
  float timeDur3_s = 3.0 * timeDur_s;

  if (time_s < timeDur3_s) {
    excite_nd = amp_nd;

  } else if (time_s < timeDur3_s + timeDur2_s) {
    excite_nd = -amp_nd;

  } else if (time_s < timeDur3_s + timeDur2_s + timeDur_s) {
    excite_nd = amp_nd;

  } else if (time_s < timeDur3_s + timeDur2_s + timeDur_s + timeDur_s) {
    excite_nd = -amp_nd;

  } else { excite_nd = 0.0; }
  
  return excite_nd;
}

// Chirp (frequency Sweep), linear varying amplitude and frequency
float GenChirpLinearAmp(float &time_s, float &timeDur_s, float &freqStart_rps, float &freqEnd_rps, float &ampStart_nd, float &ampEnd_nd)
{
  float freq_rps;    // [rps], frequency, linearly changing with time
  float amp_nd = 0.0;  // [nd], amplitude, linearly changing with time
  
  // linear varying instantanious frequency
  freq_rps = freqStart_rps + (freqEnd_rps - freqStart_rps) / (2.0 * timeDur_s) * time_s;

  // linear varying amplitude
  amp_nd = ampStart_nd + (ampEnd_nd - ampStart_nd) * time_s / timeDur_s;

  // chirp Equation
  excite_nd = amp_nd * sin(freq_rps * time_s);

  return excite_nd;
}

// Optimal MultiSine
float GenMultiSineOms(float &time_s, VecElem &freqVec_rps, VecElem &phaseVec_rad, VecElem &ampVec_nd)
{
  // Compute the Excitation - scale * amp .* sum(cos(freq * time + phase))
  excite_nd = ampVec_nd.array() * (freqVec_rps.array() * time_s.array() + phaseVec_rad.array()).cos().array();
 
  // Scale the excitation to preserve unity
  float scale = sqrt(1.0 / numElem);
  excite_nd *= scale; // Apply scaling

  return excite_nd;
}