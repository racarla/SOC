/*
Classes and Functions for Waveform Generation

See: LICENSE.md for Copyright and License Agreement

History:
2017-11-12 - Chris Regan - Created
2017-11-15 - Chris Regan - Added Classes and Eigen for full vectorization
*/

#ifndef WAVESYS_H
#define WAVESYS_H

#include <stdint.h>
#include <iostream>
#include <map>
#include <vector>
#include <memory>

#include <Eigen/Core>

#include "waveFunc.hxx"
#include "utilities.hxx"

#ifndef kVerboseConfig
#define kVerboseConfig 1
#endif

#ifndef kMaxWaveElem
#define kMaxWaveElem 46
#endif

// Matrix<typename Scalar, int RowsAtCompiletime, int ColsAtCompiletime, int Options = 0, int MaxRowsAtCompiletime = RowsAtCompiletime, int MaxColsAtCompiletime = ColsAtCompiletime>
typedef Eigen::Matrix<float, -1, 1, 0, kMaxWaveElem, 1> VecElem;

// Base Class for all Waveforms
class WaveBase {
 public:
  virtual void Config(const ObjJson &objJson) {};
  virtual void Run(const float &tCurr_s, float *wave_nd) {};

  virtual ~WaveBase() {};
};

// Discrete Waveforms
class WaveDisc : public WaveBase  {
 public:
  void Config(const ObjJson &objJson);
  void Run(const float &tCurr_s, float *wave_nd);

  ~WaveDisc() {};

 private:
  enum EnumType {kPulse = 1, kDoublet = 2, kDoublet121 = 3, kDoublet3211 = 4} eType_;
  float tPulse_s_, tDur_s_, amp_nd_;
};

// Chirp Waveforms
class WaveChirp : public WaveBase   {
 public:
  void Config(const ObjJson &objJson);
  void Run(const float &tCurr_s, float *wave_nd);

  ~WaveChirp() {};

 private:
  enum EnumType {kLinear = 1} eType_;
  float tDur_s_;
  float freqStart_rps_, freqEnd_rps_, ampStart_nd_, ampEnd_nd_;
};

// MultiSine Waveforms
class WaveMultisine : public WaveBase   {
 public:
  void Config(const ObjJson &objJson);
  void Run(const float &tCurr_s, float *wave_nd);

  ~WaveMultisine() {};

 private:
  enum EnumType {kOMS = 1} eType_;
  float tDur_s_;
  VecElem freq_rps_, phase_rad_, amp_nd_;
};


// Setup Vector of Pointers to instances of WaveSys

// Factory Class for Waveforms
class WaveFactory {
 public:
  typedef std::shared_ptr<WaveBase> SysPtr;
  typedef std::map <std::string, SysPtr> SysMap;

  static void Config(const ObjJson &objJson, SysMap *sysMap);
  static void ConfigInst(const ObjJson &objJson, SysPtr *sysPtr);
private:
  enum EnumType {kDisc = 1, kChirp = 2, kMultisine = 3}; // Enumerations of the WaveSystem types
};

#endif // WAVESYS_H
