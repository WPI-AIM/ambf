#  Software License Agreement (BSD License)
#  Copyright (c) 2003-2016, CHAI3D.
#  (www.chai3d.org)
#
#  All rights reserved.
#
#  Redistribution and use in source and binary forms, with or without
#  modification, are permitted provided that the following conditions
#  are met:
#
#  * Redistributions of source code must retain the above copyright
#  notice, this list of conditions and the following disclaimer.
#
#  * Redistributions in binary form must reproduce the above
#  copyright notice, this list of conditions and the following
#  disclaimer in the documentation and/or other materials provided
#  with the distribution.
#
#  * Neither the name of CHAI3D nor the names of its contributors may
#  be used to endorse or promote products derived from this software
#  without specific prior written permission.
#
#  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
#  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
#  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
#  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
#  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
#  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
#  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
#  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
#  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
#  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
#  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
#  POSSIBILITY OF SUCH DAMAGE.
#
#  $Author: seb $
#  $Date: 2016-01-21 16:13:27 +0100 (Thu, 21 Jan 2016) $
#  $Rev: 1906 $


# project layout
TOP_DIR = ../../..
include $(TOP_DIR)/Makefile.common

# GLFW dependency
CXXFLAGS += -I$(GLFW_DIR)/include
LDFLAGS  += -L$(GLFW_DIR)/lib/$(CFG)/$(OS)-$(ARCH)-$(COMPILER)
LDLIBS   += $(LDLIBS_GLFW)

# platform-dependent adjustments
ifeq ($(OS), mac)
  DEPLOY = Rez -append $(TOP_DIR)/bin/resources/icons/chai3d.rsrc -o $(OUTPUT); SetFile -a C $(OUTPUT)
endif

# local configuration
SRC_DIR   = .
HDR_DIR   = .
OBJ_DIR   = ./obj/$(CFG)/$(OS)-$(ARCH)-$(COMPILER)
PROG      = $(notdir $(shell pwd)) 
SOURCES   = $(wildcard $(SRC_DIR)/*.cpp)
INCLUDES  = $(wildcard $(HDR_DIR)/*.h)
OBJECTS   = $(patsubst %.cpp, $(OBJ_DIR)/%.o, $(notdir $(SOURCES)))
OUTPUT    = $(BIN_DIR)/$(PROG)

# local settings
CXXFLAGS += -DTETLIBRARY

all: $(OUTPUT)

$(OBJECTS): $(INCLUDES)

$(OUTPUT): $(OBJ_DIR) $(LIB_TARGET) $(LIB_CHAI3D) $(OBJECTS)
	$(CXX) $(CXXFLAGS) -I$(HDR_DIR) $(OBJECTS) $(LDFLAGS) $(LDLIBS) -o $(OUTPUT)
	$(DEPLOY)

$(OBJ_DIR):
	mkdir -p $@

$(OBJ_DIR)/%.o : $(SRC_DIR)/%.cpp
	$(CXX) $(CXXFLAGS) -c -o $@ $<

clean:
	rm -f $(OUTPUT) $(OBJECTS) *~
	-rm -rf $(OBJ_DIR)
