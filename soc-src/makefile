#
# MAKEFILE
#
# Brian R Taylor
# brian.taylor@bolderflight.com
# 2017-04-18
#
# Copyright (c) 2017 Bolder Flight Systems
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
CC=arm-linux-gnueabihf-g++-7 -std=c++17 -Wno-psabi

# includes
IFLAGS=-I../soc-includes/ -I/usr/local/include  -I/usr/include/hdf5/serial

# configuration
LFLAGS=-lz -lm -lhdf5_cpp -lhdf5_serial -L/usr/lib/arm-linux-gnueabihf

# code to be compiled
OBJ =\
global-defs.cxx \
datalogger.cxx \
fmu.cxx \
main.cxx

# rules
all: output display

output: $(OBJ)
	@ echo "Building..."
	$(CC) $(IFLAGS) -o $@ $^ $(LFLAGS) $(CFLAGS)

clean:
	-rm output

display:
	@ echo
	@ echo "Successful build."
	@ echo ""
	@ echo "Bolder Flight Systems, Bolder by Design!"
	@ echo "Copyright (c) 2017 Bolder Flight Systems"
	@ echo "bolderflight.com"
	@ echo ""
