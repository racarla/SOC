/*
Scas System - Configures and Executes SCAS Controllers

See: LICENSE.md for Copyright and License Agreement
*/

#include <iostream>

#include "CtrlFunc.hxx"

typedef rapidjson::Value ObjJson;

typedef std::vector <std::shared_ptr<Ctrl>> ScasLoopVec;
typedef std::map <std::string, ScasLoopVec> ScasSysMap;


class ScasSys {
 public:
  ScasSys() {};
  ~ScasSys() {};
  ScasSysMap Config(const ObjJson &objJson);
  void Init() {};
  void Run() {};

 private:

}


ScasSysMap ScasSys::Config(const ObjJson &objScas) {
  if (kVerboseConfig) printf("SCAS Configuration:\n");

  ScasLoopVec scasLoopVec; // Create a Vector for of SCAS System Loops;
  ScasSysMap scasSysMap; // Create the Map of SCAS Systems;

  // Iterate through each of the ScasSys entities
  // Iterate through each of the ScasSys entities
  for (ObjJson::ConstMemberIterator iScas = objScas.MemberBegin(); iScas != objScas.MemberEnd(); ++iScas) {
    if (kVerboseConfig) printf("\tSCAS: %s\n", iScas->name.GetString()); // Print the ScasSys Name

    const ObjJson &objScasCurr = objScas[iScas->name.GetString()]; // Create JSON object for the Current Scas System

    // Loop through each of the Control loops defined within the current Scas System
    rapidjson::SizeType numLoop = objScasCurr.Size(); // Number of loops defined in the SCAS
    for (rapidjson::SizeType iLoop = 0; iLoop < numLoop; ++iLoop) {
      const ObjJson &objLoopCurr = objScasCurr[iLoop]; // Create JSON object for the Current Loop
      std::string typeLoop = objLoopCurr["Type"].GetString(); // Get the "type" of control system defined
      if (kVerboseConfig) printf("\t\tLoop #: %d\tType: ", iLoop, typeLoop);

      // Send the JSON Object to the relevant Control System
      // Add the current loop to the vector of loop(s) within the current SCAS System
      // scasLoopVec is built up from the defined loops for the Current iScas.
      if (typeLoop == "Pass") {
        float cmdRng = objLoopCurr["cmdRng"];

        loopCur = CtrlFunc::CtrlPid2.Config(1.0, 0, 0, 1.0, 1.0, 1.0, cmdRng);
        scasLoopVec.emplace_back(new loopCur());

      } else if (typeLoop == "Manual") {
        float refScale = objLoopCurr["refScale"];
        float cmdRng = objLoopCurr["cmdRng"];

        loopCur = CtrlFunc::CtrlPid2.Config(1.0, 0, 0, 1.0, 1.0, refScale, cmdRng);
        scasLoopVec.emplace_back(new loopCur());

      } else if (typeLoop == "PID") {
        float Kp = objLoopCurr["Kp"];
        float Ki = objLoopCurr["Ki"];
        float Kd = objLoopCurr["Kd"];
        float refScale = objLoopCurr["refScale"];
        float cmdRng = objLoopCurr["cmdRng"];

        loopCur = CtrlFunc::CtrlPid2.Config(Kp, Ki, Kd, 1.0, 1.0, refScale, cmdRng);
        scasLoopVec.emplace_back(new loopCur());

      } else if (typeLoop == "PI+Damp") {
        float Kp = objLoopCurr["Kp"];
        float Ki = objLoopCurr["Ki"];
        float Kd = objLoopCurr["Kd"];
        float refScale = objLoopCurr["refScale"];
        float cmdRng = objLoopCurr["cmdRng"];

        loopCur = CtrlFunc::CtrlPiDamp.Config(Kp, Ki, Kd, 1.0, refScale, cmdRng);
        scasLoopVec.emplace_back(new loopCur());

      } else if (typeLoop == "SS") {

      } else {
        scasLoopVec.push_back(std::make_shared <Ctrl> ());
      }

      // Use the Class Configurator, need to cast the pointer to the proper derived class.
      scasLoopVec[iLoop]->Config(objScasCurr[iLoop]);

      scasSysMap.insert(std::make_pair(iScas->name.GetString(), scasLoopVec)); // Add the set of loops to the SCAS Map

    } // for iLoop

    scasLoopVec.clear(); // Clear the scasLoopVec
  } // for iScas

  return scasSysMap;
};


void ScasSys::Init()
{

};
