

#ifndef GLOBAL_DEFS_HXX_
#define GLOBAL_DEFS_HXX_

#include <stdint.h>
#include <vector>
#include <variant>
#include <map>

class DefinitionTree {
  public:
    // variable definition
    struct VariableDefinition {
      std::variant<uint64_t*,uint32_t*,uint16_t*,uint8_t*,int64_t*,int32_t*,int16_t*,int8_t*,float*, double*> Value;
      std::string Description;
      bool Datalog;
      bool Telemetry;
    };
    void DefineMember(std::string Name,struct VariableDefinition &VariableDefinitionRef);
    void InitMember(std::string Name);
    void SetValue(std::string Name,std::variant<uint64_t*,uint32_t*,uint16_t*,uint8_t*,int64_t*,int32_t*,int16_t*,int8_t*,float*, double*> Value);
    void SetDescription(std::string Name,std::string Description);
    void SetDatalog(std::string Name,bool Datalog);
    void SetTelemetry(std::string Name,bool Telemetry);
    template <typename T> T* GetValue(std::string Name) {
      if(auto val = std::get_if<T*>(&Data_[Name].Value)) {
        return *val;
      } else {
        return NULL;
      }
    }
    std::string GetDescription(std::string Name);
    bool GetDatalog(std::string Name);
    bool GetTelemetry(std::string Name);
    void GetMember(std::string Name,struct VariableDefinition *VariableDefinitionPtr);
    size_t Size(std::string Name);
    void GetKeys(std::string Name,std::vector<std::string> *KeysPtr);
    void Erase(std::string Name);
    void Clear();
  private:
    std::map<std::string,VariableDefinition> Data_;
};

#endif
