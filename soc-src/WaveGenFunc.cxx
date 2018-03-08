/*
Classes and Functions for  Single Channel Waveform Signal Generators

See: LICENSE.md for Copyright and License Agreement
*/

#include "WaveGenFunc.hxx"

// Pulse with 1-pulse tOnePulse_s long
float WaveGenPulse(float &t_s, float &tOnePulse_s, float &amp_nd)
{
  float wave_nd;

  if (t_s < tOnePulse_s) {
    wave_nd = amp_nd;

  } else { wave_nd = 0.0; }

  return wave_nd;
}

// Doublet 1-1 with 1-pulse tOnePulse_s long, total of 2*tOnePulse_s waveform
float WaveGenDoublet(float &t_s, float &tOnePulse_s, float &amp_nd)
{
  float tTwoPulse_s = 2.0 * tOnePulse_s;
  float wave_nd;

  if (t_s < tOnePulse_s) {
    wave_nd = amp_nd;

  } else if (t_s < tTwoPulse_s) {
    wave_nd = -amp_nd;

  } else { wave_nd = 0.0; }

  return wave_nd;
}

// Doublet 1-2-1 - 1-pulse tOnePulse_s long, total of 4*tOnePulse_s waveform
float WaveGenDoublet121(float &t_s, float &tOnePulse_s, float &amp_nd)
{
  float tTwoPulse_s = 2.0 * tOnePulse_s;
  float wave_nd;

  if (t_s < tOnePulse_s) {
    wave_nd = amp_nd;

  } else if (t_s < tOnePulse_s + tTwoPulse_s) {
    wave_nd = -amp_nd;

  } else if (t_s < tOnePulse_s + tTwoPulse_s + tOnePulse_s) {
    wave_nd = amp_nd;

  } else { wave_nd = 0.0; }

  return wave_nd;
}

// Doublet 3-2-1-1 - 1-pulse tOnePulse_s long, total of 7*tOnePulse_s waveform
float WaveGenDoublet3211(float &t_s, float &tOnePulse_s, float &amp_nd)
{
  float tTwoPulse_s = 2.0 * tOnePulse_s;
  float tThreePulse_s = 3.0 * tOnePulse_s;
  float wave_nd;

  if (t_s < tThreePulse_s) {
    wave_nd = amp_nd;

  } else if (t_s < tThreePulse_s + tTwoPulse_s) {
    wave_nd = -amp_nd;

  } else if (t_s < tThreePulse_s + tTwoPulse_s + tOnePulse_s) {
    wave_nd = amp_nd;

  } else if (t_s < tThreePulse_s + tTwoPulse_s + tOnePulse_s + tOnePulse_s) {
    wave_nd = -amp_nd;

  } else { wave_nd = 0.0; }

  return wave_nd;
}

// Chirp (frequency Sweep), linear varying amplitude and frequency
float WaveGenChirpLinear(float &t_s, float &tDur_s, float &freqStart_rps, float &freqEnd_rps, float &ampStart_nd, float &ampEnd_nd)
{
  // linear varying instantanious frequency
  float freq_rps = freqStart_rps + (freqEnd_rps - freqStart_rps) / (2.0 * tDur_s) * t_s;

  // linear varying amplitude
  float amp_nd = ampStart_nd + (ampEnd_nd - ampStart_nd) * t_s / tDur_s;

  // chirp Equation
  float wave_nd = amp_nd * sin(freq_rps * t_s);

  return wave_nd;
}

// Optimal MultiSine
float WaveGenMultisine(float &t_s, VecElem &freq_rps, VecElem &phase_rad, VecElem &amp_nd)
{
  // Number of elements in the List
  uint8_t numElem = (uint8_t) freq_rps.size();

  // Scale the waveform to preserve unity
  float scale = sqrt(1.0 / numElem);

  // Compute the Waveform - scale * sum(amp .* cos(freq * t + phase))
  VecElem waveVec_nd = amp_nd.array() * (freq_rps * t_s + phase_rad).array().cos();
  float wave_nd = scale * waveVec_nd.sum();

  return wave_nd;
}
