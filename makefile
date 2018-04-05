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

# output targets
TARGET_FLIGHT := flightcode
TARGET_DATALOG := datalog-server
# compiler
CXX := arm-linux-gnueabihf-g++-7
# cxx flags
override CXXFLAGS += -std=c++17 -O3 -Wno-psabi -I includes/
# directory structure
OBJDIR := obj
BINDIR := bin
SRCDIR := src
# flight code objects
OBJECTS_FLIGHT := \
uNavINS.o \
AirData.o \
sensor-processing.o \
configuration.o \
definition-tree.o \
datalog.o \
fmu.o \
flightcode.o
# datalog server objects
OBJECTS_DATALOG := \
definition-tree.o \
datalog.o \
datalog-server.o
# add prefix to objects
OBJS_FLIGHT := $(addprefix $(OBJDIR)/,$(OBJECTS_FLIGHT))
OBJS_DATALOG := $(addprefix $(OBJDIR)/,$(OBJECTS_DATALOG))

# rules
all: flightcode datalog-server | $(OBJDIR)

flightcode: $(addprefix $(BINDIR)/,$(TARGET_FLIGHT))

datalog-server: $(addprefix $(BINDIR)/,$(TARGET_DATALOG))

$(addprefix $(BINDIR)/,$(TARGET_FLIGHT)): $(OBJS_FLIGHT) | $(BINDIR)
	@ echo
	@ echo "Building flight code..."
	@ echo
	$(CXX) $(CXXFLAGS) -o $(addprefix $(BINDIR)/,$(TARGET_FLIGHT)) $(OBJS_FLIGHT)
	@ echo
	@ echo "Successful build."
	@ echo ""
	@ echo "Bolder Flight Systems, by Design!"
	@ echo "Copyright (c) 2018 Bolder Flight Systems"
	@ echo "bolderflight.com"
	@ echo ""

$(addprefix $(BINDIR)/,$(TARGET_DATALOG)): $(OBJS_DATALOG) | $(BINDIR)
	@ echo
	@ echo "Building datalog server..."
	@ echo
	$(CXX) $(CXXFLAGS) -o $(addprefix $(BINDIR)/,$(TARGET_DATALOG)) $(OBJS_DATALOG)
	@ echo
	@ echo "Successful build."
	@ echo ""
	@ echo "Bolder Flight Systems, by Design!"
	@ echo "Copyright (c) 2018 Bolder Flight Systems"
	@ echo "bolderflight.com"
	@ echo ""

$(OBJS_FLIGHT): $(addprefix $(OBJDIR)/,%.o): $(addprefix $(SRCDIR)/,%.cc) | $(OBJDIR)
	$(CXX) $(CXXFLAGS) -c $< -o $@

$(addprefix $(OBJDIR)/,datalog-server.o): $(addprefix $(SRCDIR)/,datalog-server.cc) | $(OBJDIR)
	$(CXX) $(CXXFLAGS) -c $< -o $@

$(OBJDIR):
	mkdir $(OBJDIR)

$(BINDIR):
	mkdir $(BINDIR)

# clean targets and objects
.PHONY: clean
clean:
	-rm $(addprefix $(BINDIR)/,$(TARGET_FLIGHT)) $(addprefix $(BINDIR)/,$(TARGET_DATALOG)) $(OBJS_FLIGHT) $(addprefix $(OBJDIR)/,datalog-server.o)
