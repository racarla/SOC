
#ifndef CONFIG_HXX_
#define CONFIG_HXX_

#include "global-defs.hxx"

#include "../bin2hdf-includes/rapidjson/document.h"

#include <iostream>
#include <fstream>
#include <stdlib.h>
#include <stdio.h>
#include <exception>
#include <stdexcept>
#include <fcntl.h>
#include <unistd.h>
#include <string>

void LoadConfigFile(std::string ConfigFileName, FmuData *FmuDataPtr, FmuConfig *FmuConfigPtr);

#endif
