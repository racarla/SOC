
#include "global-defs.hxx"

void DefinitionTree::DefineMember(std::string Name,struct VariableDefinition &VariableDefinitionRef) {
  Data_[Name] = VariableDefinitionRef;
}

void DefinitionTree::InitMember(std::string Name) {
  struct VariableDefinition TempDef;
  Data_[Name] = TempDef;
}

void DefinitionTree::SetValue(std::string Name,std::variant<uint64_t*,uint32_t*,uint16_t*,uint8_t*,int64_t*,int32_t*,int16_t*,int8_t*,float*, double*> Value) {
  Data_[Name].Value = Value;
}

void DefinitionTree::SetDescription(std::string Name,std::string Description) {
  Data_[Name].Description = Description;
}

void DefinitionTree::SetDatalog(std::string Name,bool Datalog) {
  Data_[Name].Datalog = Datalog;
}

void DefinitionTree::SetTelemetry(std::string Name,bool Telemetry) {
  Data_[Name].Telemetry = Telemetry;
}

std::string DefinitionTree::GetDescription(std::string Name) {
  return Data_[Name].Description;
}

bool DefinitionTree::GetDatalog(std::string Name) {
  return Data_[Name].Datalog;
}

bool DefinitionTree::GetTelemetry(std::string Name) {
  return Data_[Name].Telemetry;
}

void DefinitionTree::GetMember(std::string Name,struct VariableDefinition *VariableDefinitionPtr) {
  *VariableDefinitionPtr = Data_[Name];
}

size_t DefinitionTree::Size(std::string Name) {
  size_t retval = 0;
  for (auto const& element : Data_) {
    if (element.first.find(Name)!=std::string::npos) {
      retval++;
    }
  }
  return retval;
}

void DefinitionTree::GetKeys(std::string Name,std::vector<std::string> *KeysPtr) {
  KeysPtr->clear();
  for (auto const& element : Data_) {
    if (element.first.find(Name)!=std::string::npos) {
      KeysPtr->push_back(element.first);
    }
  }
}

void DefinitionTree::Erase(std::string Name) {
  for (auto const& element : Data_) {
    if (element.first.find(Name)!=std::string::npos) {
      Data_.erase(element.first);
    }
  }
}

void DefinitionTree::Clear() {
  Data_.clear();
}
