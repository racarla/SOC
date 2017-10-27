
#include "config.hxx"

void LoadConfigFile(std::string ConfigFileName, Fmu FmuRef, AircraftConfig *AircraftConfigPtr, FmuData *FmuDataPtr) {
  // Load config file
  std::ifstream ConfigFile(ConfigFileName);
  std::string ConfigBuffer((std::istreambuf_iterator<char>(ConfigFile)),std::istreambuf_iterator<char>());
  
  // Parse JSON
  rapidjson::StringStream jsonConfig(ConfigBuffer.c_str());
  rapidjson::Document ConfigDom;
  ConfigDom.ParseStream(jsonConfig);
  assert(ConfigDom.IsObject());

  // Loop through all nodes
  size_t SbusVoltageSensors = 0;
  size_t PwmVoltageSensors = 0;
  assert(ConfigDom.HasMember("Nodes"));
  const rapidjson::Value& Nodes = ConfigDom["Nodes"];
  assert(Nodes.IsArray());
  for (size_t i=0; i < Nodes.Size(); i++) {
    const rapidjson::Value& Node = Nodes[i];
    if (Node.HasMember("Sensors")) {
      const rapidjson::Value& Sensors = Node["Sensors"];
      assert(Sensors.IsArray());

      // Loop through all sensors on node
      for (size_t j=0; j < Sensors.Size(); j++) {
        const rapidjson::Value& Sensor = Sensors[j];
        if (Sensor.HasMember("Type")) {
          if (Sensor["Type"] == "Mpu9250") {
            FmuDataPtr->Mpu9250Ext.resize(FmuDataPtr->Mpu9250Ext.size() + 1);
          }
          if (Sensor["Type"] == "Bme280") {
            FmuDataPtr->Bme280Ext.resize(FmuDataPtr->Bme280Ext.size() + 1);
          }
          if (Sensor["Type"] == "SbusRx") {
            FmuDataPtr->SbusRx.resize(FmuDataPtr->SbusRx.size() + 1);
          }
          if (Sensor["Type"] == "Gps") {
            FmuDataPtr->Gps.resize(FmuDataPtr->Gps.size() + 1);
          }
          if (Sensor["Type"] == "Pitot") {
            FmuDataPtr->Pitot.resize(FmuDataPtr->Pitot.size() + 1);
          }
          if (Sensor["Type"] == "Press") {
            FmuDataPtr->PressureTransducer.resize(FmuDataPtr->PressureTransducer.size() + 1);
          }
          if (Sensor["Type"] == "Ain") {
            FmuDataPtr->Analog.resize(FmuDataPtr->Analog.size() + 1);
          }
        } else {
          // error
        }
      }
    }

    if (Node.HasMember("Act")) {
      size_t SbusVoltageOnNode = 0;
      size_t PwmVoltageOnNode = 0;
      const rapidjson::Value& Effectors = Node["Act"];
      assert(Effectors.IsArray());

      // Loop through all effectors on node
      for (size_t j=0; j < Effectors.Size(); j++) {
        const rapidjson::Value& Effector = Effectors[j];
        if (Effector.HasMember("Type")) {
          if (Effector["Type"] == "SBUS") {
            SbusVoltageOnNode = 1;
          }
          if (Effector["Type"] == "PWM") {
            PwmVoltageOnNode = 1;
          }
          AircraftConfigPtr->NumberEffectors++;
        } else {
          // error
        }
      }
    SbusVoltageSensors += SbusVoltageOnNode;
    PwmVoltageSensors += PwmVoltageOnNode;
    }
  }

  FmuDataPtr->SbusVoltage.resize(SbusVoltageSensors);
  FmuDataPtr->PwmVoltage.resize(PwmVoltageSensors);

  FmuRef.WriteMessage(kConfig,ConfigBuffer.size(),(uint8_t *)ConfigBuffer.c_str());
}
