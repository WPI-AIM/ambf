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
#  $Date: 2013-07-26 18:04:11 +1200 (Fri, 26 Jul 2013) $
#  $Rev: 1158 $


# top level directory
TOP_DIR = .

# determine build configuration
ifneq ($(CFG),debug)
CFG = release
endif

# determine platform
ARCH   ?= $(shell echo `uname -m`)
SYSTEM ?= $(shell echo `uname -s`)
ifeq ($(SYSTEM), Linux)
OS       = lin
CC      ?= gcc
g++     ?= g++
COMPILER = $(CC)
endif
ifeq ($(SYSTEM), Darwin)
OS       = mac
CC      ?= clang
g++     ?= clang++
COMPILER = $(CC)
endif

# paths
LIB_DIR = $(TOP_DIR)/lib/$(CFG)/$(OS)-$(ARCH)-$(COMPILER)
INC_DIR = $(TOP_DIR)/include/theoraplayer
SRC_DIR = $(TOP_DIR)/src
OBJ_DIR = $(TOP_DIR)/obj/$(CFG)/$(OS)-$(ARCH)-$(COMPILER)

# compiler flags
FLAGS   = -I$(INC_DIR) -I$(SRC_DIR) -fsigned-char

# librarian flags
ARFLAGS = rs

# build configuration specific flags
ifneq ($(CFG),debug)
FLAGS += -O3 -w
else
FLAGS += -O0 -g -DDEBUG
endif

# set target architecture
ifeq ($(OS), lin)
ifeq ($(ARCH), x86_64)
FLAGS += -m64
endif
ifeq ($(ARCH), i686)
FLAGS += -m32 -march=i686 -msse
endif
endif
ifeq ($(OS), mac)
CFLAGS   += -arch $(ARCH)
CXXFLAGS += -arch $(ARCH) -std=c++0x -stdlib=libc++
endif

# target
LIB_TARGET = $(LIB_DIR)/libtheoraplayer.a

# source files
SOURCES  = $(wildcard $(SRC_DIR)/*.cpp)
INCLUDES = $(wildcard $(INC_DIR)/*.h)
VPATH    = $(SRC_DIR)

# internal dependencies
YUV_DIR    = $(TOP_DIR)/src/YUV/C
FLAGS     += -I$(YUV_DIR)
CSOURCES  += $(wildcard $(YUV_DIR)/*.c)
VPATH     += $(YUV_DIR)
WRAPPER_DIR = $(TOP_DIR)/src/Theora
CXXFLAGS  += -I$(WRAPPER_DIR)
SOURCES   += $(wildcard $(WRAPPER_DIR)/*.cpp)
VPATH     += $(WRAPPER_DIR)

# external dependencies
OGG_DIR    = $(TOP_DIR)/external/ogg
FLAGS     += -I$(OGG_DIR)/include
CSOURCES  += $(wildcard $(OGG_DIR)/src/*.c)
VPATH     += $(OGG_DIR)/src
VORBIS_DIR = $(TOP_DIR)/external/vorbis
FLAGS     += -I$(VORBIS_DIR)/include -I$(VORBIS_DIR)/lib
CSOURCES  += $(wildcard $(VORBIS_DIR)/lib/*.c)
VPATH     += $(VORBIS_DIR)/lib
THEORA_DIR = $(TOP_DIR)/external/theora
FLAGS     += -I$(THEORA_DIR)/include
CSOURCES  += $(wildcard $(THEORA_DIR)/lib/*.c)
VPATH     += $(THEORA_DIR)/lib
CSOURCES  += $(wildcard $(THEORA_DIR)/lib/x86/*.c)
VPATH     += $(THEORA_DIR)/lib/x86
FLAGS     += -I$(TOP_DIR)/../openal/include

# objects
OBJECTS   = $(patsubst %.cpp, $(OBJ_DIR)/%.o, $(SOURCES))
OBJECTS  += $(patsubst %.c, $(OBJ_DIR)/%.o, $(CSOURCES))
OBJ_TREE  = $(sort $(dir $(OBJECTS)))

# additional flags
CXXFLAGS += -D__THEORA

# target rules

all: lib

lib: $(LIB_TARGET)

$(LIB_TARGET): $(OBJECTS) | $(LIB_DIR)
	$(AR) $(ARFLAGS) $(LIB_TARGET) $?

$(OBJECTS):  $(INCLUDES) | $(OBJ_TREE)

$(LIB_DIR) $(OBJ_TREE):
	mkdir -p $@

$(OBJ_DIR)/%.o : %.c
	$(CC) $(FLAGS) $(CFLAGS) -c -o $@ $<
$(OBJ_DIR)/%.o : %.cpp
	$(CXX) $(FLAGS) $(CXXFLAGS) -c -o $@ $<	

tags:
	find ../.. -name \*.cpp -o -name \*h | xargs etags -o TAGS

clean:
	-rm -f $(LIB_TARGET) *~
	-rm -rf $(OBJ_DIR)

