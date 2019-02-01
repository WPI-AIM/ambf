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
COMPILER = $(CC)
endif
ifeq ($(SYSTEM), Darwin)
OS       = mac
CC      ?= clang
COMPILER = $(CC)
endif

# paths
LIB_DIR = $(TOP_DIR)/lib/$(CFG)/$(OS)-$(ARCH)-$(COMPILER)
INC_DIR = $(TOP_DIR)/include
SRC_DIR = $(TOP_DIR)
OBJ_DIR = $(TOP_DIR)/obj/$(CFG)/$(OS)-$(ARCH)-$(COMPILER)

# compiler flags
CFLAGS  = -I$(INC_DIR) -I$(SRC_DIR)/OpenAL32/Include -I$(SRC_DIR)/Alc -fsigned-char -DAL_ALEXT_PROTOTYPES

# librarian flags
ARFLAGS  = rs

# build configuration specific flags
ifneq ($(CFG),debug)
CFLAGS += -O3 -w
else
CFLAGS += -O0 -g -DDEBUG
endif

# set target architecture
ifeq ($(OS), lin)
ifeq ($(ARCH), x86_64)
CFLAGS += -DLINUX -m64
endif
ifeq ($(ARCH), i686)
CFLAGS += -DLINUX -m32 -march=i686 -msse
endif
endif
ifeq ($(OS), mac)
CFLAGS += -DMACOSX -arch $(ARCH)
endif

# target
LIB_TARGET = $(LIB_DIR)/libopenal.a

# source files
CFLAGS    += -DAL_ALEXT_PROTOTYPES -DAL_BUILD_LIBRARY
CFLAGS    += -I$(SRC_DIR)/include -I$(SRC_DIR)/OpenAL32/Include -I$(SRC_DIR)/Alc
CSOURCES  += $(wildcard $(SRC_DIR)/OpenAL32/*.c)
CSOURCES  += $(wildcard $(SRC_DIR)/Alc/backends/*.c)
CSOURCES  += $(wildcard $(SRC_DIR)/Alc/*.c)
OBJECTS    = $(patsubst %.c, $(OBJ_DIR)/%.o, $(CSOURCES))
OBJ_TREE   = $(sort $(dir $(OBJECTS)))
INCLUDES   = $(wildcard $(INC_DIR)/AL/*.h)
VPATH     += $(SRC_DIR)/OpenAL32
VPATH     += $(SRC_DIR)/Alc
VPATH     += $(SRC_DIR)/Alc/backends
VPATH     += $(SRC_DIR)/Alc/effects

# target rules

all: lib

lib: $(LIB_TARGET)

$(LIB_TARGET): $(OBJECTS) | $(LIB_DIR)
	$(AR) $(ARFLAGS) $(LIB_TARGET) $?

$(OBJECTS): $(INCLUDES) | $(OBJ_TREE)

$(LIB_DIR) $(OBJ_TREE):
	mkdir -p $@

$(OBJ_DIR)/%.o : $(SRC_DIR)/%.c
	$(CC) $(CFLAGS) -c -o $@ $<

tags:
	find ../.. -name \*.cpp -o -name \*h | xargs etags -o TAGS

clean:
	-rm -f $(LIB_TARGET) *~
	-rm -rf $(OBJ_DIR)

