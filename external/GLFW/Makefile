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
#  $Date: 2016-07-09 06:15:56 +0200 (Sat, 09 Jul 2016) $
#  $Rev: 2068 $


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
SRC_DIR = $(TOP_DIR)/src
OBJ_DIR = $(TOP_DIR)/obj/$(CFG)/$(OS)-$(ARCH)-$(COMPILER)

# librarian flags
ARFLAGS  = rs

# build configuration specific flags
ifneq ($(CFG),debug)
CFLAGS += -O3 -w
else
CFLAGS += -O0 -g -DDEBUG
endif

# set common source files
CSOURCES += $(SRC_DIR)/context.c \
            $(SRC_DIR)/init.c \
            $(SRC_DIR)/input.c \
            $(SRC_DIR)/monitor.c \
            $(SRC_DIR)/vulkan.c \
            $(SRC_DIR)/window.c

# set target architecture
ifeq ($(OS), lin)
CFLAGS   += -DLINUX -D_GLFW_X11
CSOURCES += $(SRC_DIR)/x11_init.c \
            $(SRC_DIR)/x11_monitor.c \
            $(SRC_DIR)/x11_window.c \
            $(SRC_DIR)/xkb_unicode.c \
            $(SRC_DIR)/linux_joystick.c \
            $(SRC_DIR)/posix_time.c \
            $(SRC_DIR)/posix_tls.c \
            $(SRC_DIR)/glx_context.c \
            $(SRC_DIR)/egl_context.c
ifeq ($(ARCH), x86_64)
CFLAGS   += -m64
endif
ifeq ($(ARCH), i686)
CFLAGS   += -m32 -march=i686 -msse2 -mtune=generic
endif
ifeq ($(ARCH), native)
CFLAGS   += -march=native
endif
endif
ifeq ($(OS), mac)
CFLAGS   += -DMACOSX -D_GLFW_COCOA -arch $(ARCH)
CSOURCES += $(SRC_DIR)/cocoa_init.m \
            $(SRC_DIR)/cocoa_joystick.m \
            $(SRC_DIR)/cocoa_monitor.m \
            $(SRC_DIR)/cocoa_window.m \
            $(SRC_DIR)/cocoa_time.c \
            $(SRC_DIR)/posix_tls.c \
            $(SRC_DIR)/nsgl_context.m
endif

# target
LIB_TARGET = $(LIB_DIR)/libglfw.a

# source files
CFLAGS    += -I$(INC_DIR) -I$(SRC_DIR)
OBJECTS    = $(patsubst %.c, $(OBJ_DIR)/%.o, $(CSOURCES))
OBJECTS   += $(patsubst %.m, $(OBJ_DIR)/%.o, $(CSOURCES))
OBJ_TREE   = $(sort $(dir $(OBJECTS)))
INCLUDES   = $(wildcard $(INC_DIR)/GLFW/*.h)
VPATH     += $(SRC_DIR)

# target rules

all: lib

lib: $(LIB_TARGET)

$(LIB_TARGET): $(OBJECTS) | $(LIB_DIR)
	$(AR) $(ARFLAGS) $(LIB_TARGET) $?

$(OBJECTS): $(INCLUDES) | $(OBJ_TREE)

$(LIB_DIR) $(OBJ_TREE):
	mkdir -p $@

$(OBJ_DIR)/%.o : %.c
	$(CC) $(CFLAGS) -c -o $@ $<

$(OBJ_DIR)/%.o : %.m
	$(CC) $(CFLAGS) -c -o $@ $<

tags:
	find ../.. -name \*.cpp -o -name \*h | xargs etags -o TAGS

clean:
	-rm -f $(LIB_TARGET) *~
	-rm -rf $(OBJ_DIR)

