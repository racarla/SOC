#
# MAKEFILE
# 
# See: LICENSE.md for Copyright and License Agreement
#
# History:
# Chris Regan
# 2017-11-12 - Chris Regan - Merged seperate makefiles into a single source
#
#

# compiler
CC_FLIGHT = arm-linux-gnueabihf-g++ -std=c++0x
CC_CONFIG = arm-linux-gnueabihf-g++ -std=c++0x
CC_BIN2HDF = g++ -std=c++0x

# src
SRC_FLIGHT = soc-src
SRC_CONFIG = fmu-cfg
SRC_BIN2HDF = bin2hdf-src

# includes
IFLAGS_FLIGHT = -Isoc-includes
IFLAGS_CONFIG = -Isoc-includes
IFLAGS_BIN2HDF = -Ibin2hdf-includes -I/usr/local/include -I/usr/include/hdf5/serial

# configuration
LFLAGS_FLIGHT =
LFLAGS_CONFIG =
LFLAGS_BIN2HDF = -lz -lm -lhdf5_cpp -lhdf5_serial -L/usr/lib/x86_64-linux-gnu

# code to be compiled
OBJ_FLIGHT =\
$(SRC_FLIGHT)/navigation.cxx \
$(SRC_FLIGHT)/EKF_15state.cxx \
$(SRC_FLIGHT)/nav_functions.cxx \
$(SRC_FLIGHT)/datalogger.cxx \
$(SRC_FLIGHT)/config.cxx \
$(SRC_FLIGHT)/fmu.cxx \
$(SRC_FLIGHT)/main.cxx

OBJ_CONFIG =\
$(SRC_CONFIG)/config.cxx \
$(SRC_CONFIG)/fmu.cxx \
$(SRC_CONFIG)/main.cxx

OBJ_BIN2HDF =\
$(SRC_BIN2HDF)/hdf5class.cxx \
$(SRC_BIN2HDF)/config.cxx \
$(SRC_BIN2HDF)/main.cxx


# rules
all: flightcode config bin2hdf display

flightcode: $(OBJ_FLIGHT)
	@ echo "Building flightcode ..."	
	$(CC_FLIGHT) $(IFLAGS_FLIGHT) -o $@ $^ $(LFLAGS_FLIGHT) $(CFLAGS_FLIGHT)

config: $(OBJ_CONFIG)
	@ echo "Building config ..."	
	$(CC_CONFIG) $(IFLAGS_CONFIG) -o $@ $^ $(LFLAGS_CONFIG) $(CFLAGS_CONFIG)

bin2hdf: $(OBJ_BIN2HDF)
	@ echo "Building bin2hdf ..."	
	$(CC_BIN2HDF) $(IFLAGS_BIN2HDF) $^ -o $@ $(LFLAGS_BIN2HDF) $(CFLAGS_BIN2HDF)
	

clean:
	-rm flightcode config bin2hdf

display: 
	@ echo
	@ echo "Successful build."
	@ echo ""
	@ echo "Bolder Flight Systems, Bolder by Design!"
	@ echo "Copyright (c) 2017 Bolder Flight Systems"
	@ echo "bolderflight.com"
	@ echo "" 
