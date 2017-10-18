
#include "config.hxx"

void LoadConfigFile(std::string ConfigFileName, FmuData *FmuDataPtr, FmuConfig *FmuConfigPtr) {
  // Load config file
  std::ifstream ConfigFile(ConfigFileName);
  std::string ConfigBuffer((std::istreambuf_iterator<char>(ConfigFile)),std::istreambuf_iterator<char>());
  
  // Parse JSON
  rapidjson::StringStream jsonConfig(ConfigBuffer.c_str());
  rapidjson::Document ConfigDom;
  ConfigDom.ParseStream(jsonConfig);
  assert(ConfigDom.IsObject());

  // Indices to count how many of each sensor we have
  size_t Mpu9250Index = 0;
  size_t Bme280Index = 0;
  size_t SbusRxIndex = 0;
  size_t GpsIndex = 0;
  size_t PitotIndex = 0;
  size_t PressureTransducerIndex = 0;
  size_t AnalogIndex = 0;

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
            FmuConfigPtr->Mpu9250Names.resize(FmuConfigPtr->Mpu9250Names.size() + 1);
            if (Sensor.HasMember("FieldName")) {
              FmuConfigPtr->Mpu9250Names[Mpu9250Index] = Sensor["FieldName"].GetString();
            } else {
              FmuConfigPtr->Mpu9250Names[Mpu9250Index] = "Mpu9250_" + std::to_string(Mpu9250Index);
            }
            Mpu9250Index++;
          }
          if (Sensor["Type"] == "Bme280") {
            FmuDataPtr->Bme280Ext.resize(FmuDataPtr->Bme280Ext.size() + 1);
            FmuConfigPtr->Bme280Names.resize(FmuConfigPtr->Bme280Names.size() + 1);
            if (Sensor.HasMember("FieldName")) {
              FmuConfigPtr->Bme280Names[Bme280Index] = Sensor["FieldName"].GetString();
            } else {
              FmuConfigPtr->Bme280Names[Bme280Index] = "Bme280_" + std::to_string(Bme280Index);
            }
            Bme280Index++;
          }
          if (Sensor["Type"] == "SbusRx") {
            FmuDataPtr->SbusRx.resize(FmuDataPtr->SbusRx.size() + 1);
            FmuConfigPtr->SbusRxNames.resize(FmuConfigPtr->SbusRxNames.size() + 1);
            if (Sensor.HasMember("FieldName")) {
              FmuConfigPtr->SbusRxNames[SbusRxIndex] = Sensor["FieldName"].GetString();
            } else {
              FmuConfigPtr->SbusRxNames[SbusRxIndex] = "SbusRx_" + std::to_string(SbusRxIndex);
            }
            SbusRxIndex++;
          }
          if (Sensor["Type"] == "Gps") {
            FmuDataPtr->Gps.resize(FmuDataPtr->Gps.size() + 1);
            FmuConfigPtr->GpsNames.resize(FmuConfigPtr->GpsNames.size() + 1);
            if (Sensor.HasMember("FieldName")) {
              FmuConfigPtr->GpsNames[GpsIndex] = Sensor["FieldName"].GetString();
            } else {
              FmuConfigPtr->GpsNames[GpsIndex] = "Gps_" + std::to_string(GpsIndex);
            }
            GpsIndex++;
          }
          if (Sensor["Type"] == "Pitot") {
            FmuDataPtr->Pitot.resize(FmuDataPtr->Pitot.size() + 1);
            FmuConfigPtr->PitotNames.resize(FmuConfigPtr->PitotNames.size() + 1);
            if (Sensor.HasMember("FieldName")) {
              FmuConfigPtr->PitotNames[PitotIndex] = Sensor["FieldName"].GetString();
            } else {
              FmuConfigPtr->PitotNames[PitotIndex] = "Pitot_" + std::to_string(PitotIndex);
            }
            PitotIndex++;
          }
          if (Sensor["Type"] == "Press") {
            FmuDataPtr->PressureTransducer.resize(FmuDataPtr->PressureTransducer.size() + 1);
            FmuConfigPtr->PressureTransducerNames.resize(FmuConfigPtr->PressureTransducerNames.size() + 1);
            if (Sensor.HasMember("FieldName")) {
              FmuConfigPtr->PressureTransducerNames[PressureTransducerIndex] = Sensor["FieldName"].GetString();
            } else {
              FmuConfigPtr->PressureTransducerNames[PressureTransducerIndex] = "PressureTransducer_" + std::to_string(PressureTransducerIndex);
            }
            PressureTransducerIndex++;
          }
          if (Sensor["Type"] == "Ain") {
            FmuDataPtr->Analog.resize(FmuDataPtr->Analog.size() + 1);
            FmuConfigPtr->AnalogNames.resize(FmuConfigPtr->AnalogNames.size() + 1);
            if (Sensor.HasMember("FieldName")) {
              FmuConfigPtr->AnalogNames[AnalogIndex] = Sensor["FieldName"].GetString();
            } else {
              FmuConfigPtr->AnalogNames[AnalogIndex] = "Analog_" + std::to_string(AnalogIndex);
            }
            AnalogIndex++;
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
}
