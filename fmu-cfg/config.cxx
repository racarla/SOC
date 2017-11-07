
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

  // Switch FMU to standby mode
  uint8_t StandbyPayload[1];
  StandbyPayload[0] = (uint8_t) kStandby;
  FmuRef.WriteMessage(kMode,sizeof(StandbyPayload),StandbyPayload);

  // Loop through all nodes
  size_t SbusVoltageSensors = 0;
  size_t PwmVoltageSensors = 0;
  assert(ConfigDom.HasMember("Nodes"));
  const rapidjson::Value& Nodes = ConfigDom["Nodes"];
  assert(Nodes.IsArray());
  for (size_t i=0; i < Nodes.Size(); i++) {
    const rapidjson::Value& Node = Nodes[i];
    if (Node.HasMember("Rotation"){
      const rapidjson::Value& Rotation = Node["Rotation"];
      rapidjson::StringBuffer StringBuf;
      rapidjson::Writer<rapidjson::StringBuffer> writer(StringBuf);
      Rotation.Accept(writer);
      std::string OutputString = StringBuf.GetString();
      std::string ConfigString;
      ConfigString = "{\"Nodes\":[{\"BfsAddr\":" + std::to_string(Node["BfsAddr"].GetInt()) + ",\"Rotation\":[" +  OutputString + "]}]}";
      FmuRef.WriteMessage(kConfig,ConfigString.size(),(uint8_t *)ConfigString.c_str());
    } 
    if (Node.HasMember("Sensors")) {
      const rapidjson::Value& Sensors = Node["Sensors"];
      assert(Sensors.IsArray());
      // Loop through all sensors on node
      for (size_t j=0; j < Sensors.Size(); j++) {
        const rapidjson::Value& Sensor = Sensors[j];
        if (Sensor.HasMember("Type")) {
          if (Sensor["Type"] == "Mpu9250") {
            FmuDataPtr->Mpu9250Ext.resize(FmuDataPtr->Mpu9250Ext.size() + 1);
            rapidjson::StringBuffer StringBuf;
            rapidjson::Writer<rapidjson::StringBuffer> writer(StringBuf);
            Sensor.Accept(writer);
            std::string OutputString = StringBuf.GetString();
            std::string ConfigString;
            ConfigString = "{\"Nodes\":[{\"BfsAddr\":" + std::to_string(Node["BfsAddr"].GetInt()) + ",\"Sensors\":[" +  OutputString + "]}]}";
            FmuRef.WriteMessage(kConfig,ConfigString.size(),(uint8_t *)ConfigString.c_str());
          }
          if (Sensor["Type"] == "Bme280") {
            FmuDataPtr->Bme280Ext.resize(FmuDataPtr->Bme280Ext.size() + 1);
            rapidjson::StringBuffer StringBuf;
            rapidjson::Writer<rapidjson::StringBuffer> writer(StringBuf);
            Sensor.Accept(writer);
            std::string OutputString = StringBuf.GetString();
            std::string ConfigString;
            ConfigString = "{\"Nodes\":[{\"BfsAddr\":" + std::to_string(Node["BfsAddr"].GetInt()) + ",\"Sensors\":[" +  OutputString + "]}]}";
            FmuRef.WriteMessage(kConfig,ConfigString.size(),(uint8_t *)ConfigString.c_str());
          }
          if (Sensor["Type"] == "SbusRx") {
            FmuDataPtr->SbusRx.resize(FmuDataPtr->SbusRx.size() + 1);
            rapidjson::StringBuffer StringBuf;
            rapidjson::Writer<rapidjson::StringBuffer> writer(StringBuf);
            Sensor.Accept(writer);
            std::string OutputString = StringBuf.GetString();
            std::string ConfigString;
            ConfigString = "{\"Nodes\":[{\"BfsAddr\":" + std::to_string(Node["BfsAddr"].GetInt()) + ",\"Sensors\":[" +  OutputString + "]}]}";
            FmuRef.WriteMessage(kConfig,ConfigString.size(),(uint8_t *)ConfigString.c_str());
          }
          if (Sensor["Type"] == "Gps") {
            FmuDataPtr->Gps.resize(FmuDataPtr->Gps.size() + 1);
            rapidjson::StringBuffer StringBuf;
            rapidjson::Writer<rapidjson::StringBuffer> writer(StringBuf);
            Sensor.Accept(writer);
            std::string OutputString = StringBuf.GetString();
            std::string ConfigString;
            ConfigString = "{\"Nodes\":[{\"BfsAddr\":" + std::to_string(Node["BfsAddr"].GetInt()) + ",\"Sensors\":[" +  OutputString + "]}]}";
            FmuRef.WriteMessage(kConfig,ConfigString.size(),(uint8_t *)ConfigString.c_str());
          }
          if (Sensor["Type"] == "Pitot") {
            FmuDataPtr->Pitot.resize(FmuDataPtr->Pitot.size() + 1);
            rapidjson::StringBuffer StringBuf;
            rapidjson::Writer<rapidjson::StringBuffer> writer(StringBuf);
            Sensor.Accept(writer);
            std::string OutputString = StringBuf.GetString();
            std::string ConfigString;
            ConfigString = "{\"Nodes\":[{\"BfsAddr\":" + std::to_string(Node["BfsAddr"].GetInt()) + ",\"Sensors\":[" +  OutputString + "]}]}";
            FmuRef.WriteMessage(kConfig,ConfigString.size(),(uint8_t *)ConfigString.c_str());
          }
          if (Sensor["Type"] == "PressureSensor") {
            FmuDataPtr->PressureTransducer.resize(FmuDataPtr->PressureTransducer.size() + 1);
            rapidjson::StringBuffer StringBuf;
            rapidjson::Writer<rapidjson::StringBuffer> writer(StringBuf);
            Sensor.Accept(writer);
            std::string OutputString = StringBuf.GetString();
            std::string ConfigString;
            ConfigString = "{\"Nodes\":[{\"BfsAddr\":" + std::to_string(Node["BfsAddr"].GetInt()) + ",\"Sensors\":[" +  OutputString + "]}]}";
            FmuRef.WriteMessage(kConfig,ConfigString.size(),(uint8_t *)ConfigString.c_str());
          }
          if (Sensor["Type"] == "Analog") {
            FmuDataPtr->Analog.resize(FmuDataPtr->Analog.size() + 1);
            rapidjson::StringBuffer StringBuf;
            rapidjson::Writer<rapidjson::StringBuffer> writer(StringBuf);
            Sensor.Accept(writer);
            std::string OutputString = StringBuf.GetString();
            std::string ConfigString;
            ConfigString = "{\"Nodes\":[{\"BfsAddr\":" + std::to_string(Node["BfsAddr"].GetInt()) + ",\"Sensors\":[" +  OutputString + "]}]}";
            FmuRef.WriteMessage(kConfig,ConfigString.size(),(uint8_t *)ConfigString.c_str());
          }
        } else {
          // error
        }
      }
    }

    if (Node.HasMember("Effectors")) {
      size_t SbusVoltageOnNode = 0;
      size_t PwmVoltageOnNode = 0;
      const rapidjson::Value& Effectors = Node["Effectors"];
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
        rapidjson::StringBuffer StringBuf;
        rapidjson::Writer<rapidjson::StringBuffer> writer(StringBuf);
        Effector.Accept(writer);
        std::string OutputString = StringBuf.GetString();
        std::string ConfigString;
        ConfigString = "{\"Nodes\":[{\"BfsAddr\":" + std::to_string(Node["BfsAddr"].GetInt()) + ",\"Effectors\":[" +  OutputString + "]}]}";
        FmuRef.WriteMessage(kConfig,ConfigString.size(),(uint8_t *)ConfigString.c_str());
      }
    SbusVoltageSensors += SbusVoltageOnNode;
    PwmVoltageSensors += PwmVoltageOnNode;
    }
  }

  FmuDataPtr->SbusVoltage.resize(SbusVoltageSensors);
  FmuDataPtr->PwmVoltage.resize(PwmVoltageSensors);

  // Switch FMU to run mode
  StandbyPayload[0] = (uint8_t) kRun;
  FmuRef.WriteMessage(kMode,sizeof(StandbyPayload),StandbyPayload);
}
