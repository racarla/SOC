/*
Classes and Functions for  Single Channel Waveform Signal Generators

See: LICENSE.md for Copyright and License Agreement
*/

#include "WaveFunc.hxx"

// Pulse with 1-pulse tOnePulse_s long
void WavePulse(const float &t_s, const float &tOnePulse_s, const float &amp_nd, float *wave_nd)
{
  *wave_nd = 0.0;

  if (t_s < tOnePulse_s) {
    *wave_nd = amp_nd;
  }
}

// Doublet 1-1 with 1-pulse tOnePulse_s long, total of 2*tOnePulse_s waveform
void WaveDoublet(const float &t_s, const float &tOnePulse_s, const float &amp_nd, float *wave_nd)
{
  float tTwoPulse_s = 2.0 * tOnePulse_s;
  *wave_nd = 0.0;

  if (t_s < tOnePulse_s) {
    *wave_nd = amp_nd;
  } else if (t_s < tTwoPulse_s) {
    *wave_nd = -amp_nd;
  }
}

// Doublet 1-2-1 - 1-pulse tOnePulse_s long, total of 4*tOnePulse_s waveform
void WaveDoublet121(const float &t_s, const float &tOnePulse_s, const float &amp_nd, float *wave_nd)
{
  float tTwoPulse_s = 2.0 * tOnePulse_s;
  *wave_nd = 0.0;

  if (t_s < tOnePulse_s) {
    *wave_nd = amp_nd;
  } else if (t_s < tOnePulse_s + tTwoPulse_s) {
    *wave_nd = -amp_nd;
  } else if (t_s < tOnePulse_s + tTwoPulse_s + tOnePulse_s) {
    *wave_nd = amp_nd;
  }
}

// Doublet 3-2-1-1 - 1-pulse tOnePulse_s long, total of 7*tOnePulse_s waveform
void WaveDoublet3211(const float &t_s, const float &tOnePulse_s, const float &amp_nd, float *wave_nd)
{
  float tTwoPulse_s = 2.0 * tOnePulse_s;
  float tThreePulse_s = 3.0 * tOnePulse_s;
  *wave_nd = 0.0;

  if (t_s < tThreePulse_s) {
    *wave_nd = amp_nd;
  } else if (t_s < tThreePulse_s + tTwoPulse_s) {
    *wave_nd = -amp_nd;
  } else if (t_s < tThreePulse_s + tTwoPulse_s + tOnePulse_s) {
    *wave_nd = amp_nd;
  } else if (t_s < tThreePulse_s + tTwoPulse_s + tOnePulse_s + tOnePulse_s) {
    *wave_nd = -amp_nd;
  }
}

// Chirp (frequency Sweep), linear varying amplitude and frequency
void WaveChirpLinear(const float &t_s, const float &tDur_s, const float &freqStart_rps, const float &freqEnd_rps, const float &ampStart_nd, const float &ampEnd_nd, float *wave_nd)
{
  // linear varying instantanious frequency
  float freq_rps = freqStart_rps + (freqEnd_rps - freqStart_rps) / (2.0 * tDur_s) * t_s;

  // linear varying amplitude
  float amp_nd = ampStart_nd + (ampEnd_nd - ampStart_nd) * t_s / tDur_s;

  // chirp Equation
  *wave_nd = amp_nd * sin(freq_rps * t_s);
}

// Optimal MultiSine
void WaveMultisineOms(const float &t_s, const VecElem &freq_rps, const VecElem &phase_rad, const VecElem &amp_nd, float *wave_nd)
{
  // Number of elements in the List
  uint8_t numElem = (uint8_t) freq_rps.size();

  // Scale the waveform to preserve unity
  float scale = sqrt(1.0 / numElem);

  // Compute the Waveform - scale * sum(amp .* cos(freq * t + phase))
  VecElem waveVec_nd = amp_nd.array() * (freq_rps * t_s + phase_rad).array().cos();
  *wave_nd = scale * waveVec_nd.sum();
}
