/*
Classes and Functions for Control Systems

See: LICENSE.md for Copyright and License Agreement
*/

#ifndef CTRLSYS_H
#define CTRLSYS_H

#include <stdint.h>
#include <map>
#include <vector>
#include <memory>

#include <Eigen/Core>

#include "CtrlFunc.hxx"
#include "Utilities.hxx"

#ifndef kVerboseConfig
#define kVerboseConfig 1
#endif

#ifndef kMaxCtrlElem
#define kMaxCtrlElem 6
#endif

// Matrix<typename Scalar, int RowsAtCompiletime, int ColsAtCompiletime, int Options = 0, int MaxRowsAtCompiletime = RowsAtCompiletime, int MaxColsAtCompiletime = ColsAtCompiletime>
typedef Eigen::Matrix<float, -1, 1, 0, kMaxCtrlElem, 1> VecCtrlSig;
typedef rapidjson::Value ObjJson;


struct SigStruct {

};

// Base Class for all Ctrl systems
class CtrlBase {
 public:
  virtual void Config(const ObjJson &objJson) {};
  virtual void Run(const float &dt_s, float *cmd) {};

  virtual ~CtrlBase() {};
};

// Ctrl Empty system
class CtrlNone : public CtrlBase  {
 public:
  void Config(const ObjJson &objJson) {};
  void Run(const float &dt_s, float *cmd) {};

  ~CtrlNone() {};
};

// Ctrl PID2 system
class CtrlPid2 : public CtrlBase  {
 public:
  void Config(const ObjJson &objJson);
  void Run(const float &dt_s, float *cmd);

  ~CtrlPid2() {};
 private:
  CtrlFuncPid2 ctrlFuncPid2_;
};

// Ctrl PI+Damp system
class CtrlPiDamp : public CtrlBase   {
 public:
  void Config(const ObjJson &objJson) {};
  void Run(const float &dt_s, float *cmd) {};

  ~CtrlPiDamp() {};
 private:
  CtrlFuncPiDamp ctrlFuncPiDamp_;
};

// Ctrl PI+Damp system
class CtrlSS : public CtrlBase   {
 public:
  void Config(const ObjJson &objJson) {};
  void Run(const float &dt_s, float *cmd) {};

  ~CtrlSS() {};
 private:
  // CtrlFuncSS ctrlFuncSS;
};

// Factory Class for Ctrl systems
class CtrlSys {
 public:
  typedef std::shared_ptr<CtrlBase> SysInstPtr; // Pointer to an individual controller
  typedef std::vector <SysInstPtr> SysDefVec; // Vector of pointers to controllers
  typedef std::map <std::string, SysDefVec> SysDefMap; // Map of Vectors to Pointer to contoller instances

  static void ConfigDef(const ObjJson &objJson, SysDefMap *sysDefMap); // Configuration of the whole Controller Definition Set
  static void ConfigDefVec(const ObjJson &objJson, SysDefVec *SysDefVec); // Configure vector of controllers, ie. ScasBaseline
  static void ConfigDefInst(const ObjJson &objJson, SysInstPtr *SysInstPtr); // Configure a single instance of a controller, ie. a single PID2 instance

  static void RunVec(const float &tCurr_s, const SysDefVec &SysDefVec); // Loop through and run each of a vector of controllers, ie. ScasBaseline

  typedef std::vector<std::string> SysGroupVec; // Vector of strings descriptors
  typedef std::map <std::string, SysGroupVec> SysGroupMap; // Map of Vectors
  static void ConfigGroup(const ObjJson &objJson, SysGroupMap *sysGroupMap); // Configuration of the Controller Groups, ie. "Baseline" (consisting of "GuidBaseline" and "ScasBaseline")

  static void Run(const float &tCurr_s); // Run a Grouping of Controllers, ie. "Baseline" (consisting of "GuidBaseline" and "ScasBaseline")

 private:
  enum EnumType {kNone = 0, kPid2 = 1, kPiDamp = 2, kSS = 3}; // Enumerations of the WaveSystem types
};


#endif // CTRLSYS_H
