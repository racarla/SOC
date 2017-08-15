
#ifndef CONFIG_HXX_
#define CONFIG_HXX_

#include "global-defs.hxx"
#include "fmu.hxx"

#include "../soc-includes/rapidjson/document.h"

#include <iostream>
#include <fstream>
#include <stdlib.h>
#include <stdio.h>
#include <exception>
#include <stdexcept>
#include <fcntl.h>
#include <unistd.h>

void LoadConfigFile(std::string ConfigFileName, Fmu FmuRef, FmuData *FmuDataPtr);

#endif
