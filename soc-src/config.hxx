
#ifndef CONFIG_HXX_
#define CONFIG_HXX_

#include <iostream>
#include <fstream>
#include <stdlib.h>
#include <stdio.h>
#include <exception>
#include <stdexcept>
#include <fcntl.h>
#include <unistd.h>

#include "global-defs.hxx"
#include "fmu.hxx"

#include "../soc-includes/rapidjson/document.h"
#include "../soc-includes/rapidjson/stringbuffer.h"
#include "../soc-includes/rapidjson/writer.h"

void LoadConfigFile(std::string ConfigFileName, Fmu FmuRef, AircraftConfig *AircraftConfigPtr, FmuData *FmuDataPtr);

#endif
