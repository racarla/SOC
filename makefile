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
CC_ARM := arm-linux-gnueabihf-g++-5 -O3
CC := g++-5
CFLAGS := -std=c++0x -Wall

# src
DIR_FLIGHT := soc-src
DIR_CONFIG := fmu-cfg
DIR_BIN2HDF := bin2hdf-src

# includes
IFLAGS := -Isoc-includes -I/usr/local/include -I/usr/include/hdf5/serial

# configuration
LFLAGS_FLIGHT :=
LFLAGS_CONFIG :=
LFLAGS_BIN2HDF := -lz -lm -lhdf5_cpp -lhdf5_serial

# source code to be compiled
SRC_FLIGHT :=\
$(DIR_FLIGHT)/navigation.cxx \
$(DIR_FLIGHT)/airdata.cxx \
$(DIR_FLIGHT)/EKF_15state.cxx \
$(DIR_FLIGHT)/nav_functions.cxx \
$(DIR_FLIGHT)/datalogger.cxx \
$(DIR_FLIGHT)/config.cxx \
$(DIR_FLIGHT)/fmu.cxx \
$(DIR_FLIGHT)/missionMgr.cxx \
$(DIR_FLIGHT)/cntrlMgr.cxx \
$(DIR_FLIGHT)/cntrlFunc.cxx \
$(DIR_FLIGHT)/exciteMgr.cxx \
$(DIR_FLIGHT)/exciteGenFunc.cxx \
$(DIR_FLIGHT)/cntrlAllocFunc.cxx \
$(DIR_FLIGHT)/main.cxx

SRC_CAL :=\
$(DIR_FLIGHT)/config.cxx \
$(DIR_FLIGHT)/fmu.cxx \
$(DIR_FLIGHT)/missionMgr.cxx \
$(DIR_FLIGHT)/inclinometer.cxx \
$(DIR_FLIGHT)/mainCal.cxx

SRC_CONFIG :=\
$(DIR_CONFIG)/config.cxx \
$(DIR_CONFIG)/fmu.cxx \
$(DIR_CONFIG)/main.cxx

SRC_BIN2HDF :=\
$(DIR_BIN2HDF)/hdf5class.cxx \
$(DIR_BIN2HDF)/config.cxx \
$(DIR_BIN2HDF)/main.cxx

# Create lists of object code
OBJ_FLIGHT := ${SRC_FLIGHT:%.cxx=%.o}
OBJ_CAL := ${SRC_CAL:%.cxx=%.o}
OBJ_CONFIG := ${SRC_CONFIG:%.cxx=%.o}
OBJ_BIN2HDF := ${SRC_BIN2HDF:%.cxx=%.o}

# Build rules
all: flightcode calib config bin2hdf display

#%.o: %.cxx
#	$(CC_ARM) $(CFLAGS) $(IFLAGS) -c -o $@ $<

flightcode: $(SRC_FLIGHT)
	@ echo "Building flightcode ..."	
	$(CC_ARM) $(CFLAGS) $(IFLAGS) -o $@ $? $(LFLAGS_FLIGHT)

calib: $(SRC_CAL)
	@ echo "Building calibration ..."	
	$(CC_ARM) $(CFLAGS) $(IFLAGS) -o $@ $? $(LFLAGS_FLIGHT)

config: $(SRC_CONFIG)
	@ echo "Building config ..."	
	$(CC_ARM) $(CFLAGS) $(IFLAGS) -o $@ $? $(LFLAGS_CONFIG)

bin2hdf: $(SRC_BIN2HDF)
	@ echo "Building bin2hdf ..."	
	$(CC) $(CFLAGS) $(IFLAGS) -o $@ $? $(LFLAGS_BIN2HDF)

clean:
	rm -f $(OBJ_FLIGHT) $(OBJ_CAL) ${OBJ_CONFIG} ${OBJ_CONFIG}
	rm -f flightcode calib config bin2hdf

display: 
	@ echo
	@ echo "Successful build."
	@ echo ""
	@ echo "Bolder Flight Systems, Bolder by Design!"
	@ echo "Copyright (c) 2017 Bolder Flight Systems"
	@ echo "bolderflight.com"
	@ echo "" 
