#
# MAKEFILE
#
# Brian R Taylor
# brian.taylor@bolderflight.com
#
# Copyright (c) 2018 Bolder Flight Systems
# Permission is hereby granted, free of charge, to any person obtaining a copy of this software
# and associated documentation files (the "Software"), to deal in the Software without restriction,
# including without limitation the rights to use, copy, modify, merge, publish, distribute,
# sublicense, and/or sell copies of the Software, and to permit persons to whom the Software is
# furnished to do so, subject to the following conditions:
# The above copyright notice and this permission notice shall be included in all copies or
# substantial portions of the Software.
# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING
# BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND
# NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM,
# DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
# OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
#

# compiler
CC_FLIGHT=arm-linux-gnueabihf-g++-7 -std=c++17 -Wno-psabi
CC_DATALOG=arm-linux-gnueabihf-g++-7 -std=c++17 -Wno-psabi

# includes
IFLAGS_FLIGHT=-I../includes/
IFLAGS_DATALOG=-I../includes/

# configuration
LFLAGS_FLIGHT=-L/usr/lib/arm-linux-gnueabihf
LFLAGS_DATALOG=-L/usr/lib/arm-linux-gnueabihf

# code to be compiled
OBJ_FLIGHT =\
configuration.cxx \
definition-tree.cxx \
datalog.cxx \
fmu.cxx \
flightcode.cxx

OBJ_DATALOG =\
definition-tree.cxx \
datalog.cxx \
datalog-server.cxx

# rules
all: flightcode datalog-server display

flightcode: $(OBJ_FLIGHT)
	@ echo "Building flight code..."
	$(CC_FLIGHT) $(IFLAGS_FLIGHT) -o $@ $^ $(LFLAGS_FLIGHT) $(CFLAGS_FLIGHT)
	mv flightcode ../bin

datalog-server: $(OBJ_DATALOG)
	@ echo "Building datalog server..."
	$(CC_DATALOG) $(IFLAGS_DATALOG) -o $@ $^ $(LFLAGS_DATALOG) $(CFLAGS_DATALOG)
	mv datalog-server ../bin

clean:
	-rm ../bin/flightcode ../bin/datalog-server

display:
	@ echo
	@ echo "Successful build."
	@ echo ""
	@ echo "Bolder Flight Systems, Bolder by Design!"
	@ echo "Copyright (c) 2018 Bolder Flight Systems"
	@ echo "bolderflight.com"
	@ echo ""
