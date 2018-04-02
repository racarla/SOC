/*
Excite System Manager - Defines Excitations, Computes commands

See: LICENSE.md for Copyright and License Agreement
*/

#ifndef EXCITEMGR_HXX_
#define EXCITEMGR_HXX_

#include <math.h>
#include <Eigen/Core>

#ifndef kMaxExciteChan
#define kMaxExciteChan 4
#endif

#ifndef kMaxExciteElem
#define kMaxExciteElem 46
#endif

#include "WaveGenFunc.hxx"

const float kHz2Rps = 2 * M_PI;

// Exitation output structure


class ExciteSys {
 public:

  void Init();     // Initialize excitations
  ExciteSysOut Run(const bool& exciteMode, const uint8_t& indxTest, const float& t_s);

  ExciteSysLog Log(const ExciteSysOut& exciteSysOut);

  ~ExciteSys() {};  // Destructor

  struct Out {
    bool exciteMode;
    uint8_t indxTest;
    float tExcite_s;
    VecChan cmdExcite;
  };

  struct Log {
    bool exciteMode;
    uint8_t indxTest;
    float tExcite_s;
    float cmdExcite[kMaxExciteChan] = {0};
  };

 private:
  Out out_;
  float tEngage_s_;

};

// Setup Vector of Pointers to instances of ExciteSys
typedef std::shared_ptr<ExciteSys> ExciteSysPtr;
typedef std::map <std::string, ExciteSysPtr> ExciteSysMap;

// Factory Class for Waveforms
class ExciteFactory {
 public:
  static ExciteSysMap Config(const ObjJson &objJson);
  static ExciteSysPtr ConfigInst(const ObjJson &objJson);
};


#endif // EXCITEMGR_HXX_
