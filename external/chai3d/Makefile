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
#  $Date: 2016-11-23 22:47:30 +0100 (Wed, 23 Nov 2016) $
#  $Rev: 2183 $


# global path
TOP_DIR = .

# commong settings
include $(TOP_DIR)/Makefile.common

# directories
OBJ_DIR   = $(TOP_DIR)/obj/$(CFG)/$(OS)-$(ARCH)-$(COMPILER)
SRC_DIR   = $(TOP_DIR)/src
VPATH     = $(dir $(wildcard $(SRC_DIR)/*/*))

# main sources
SOURCES   = $(shell find $(SRC_DIR) -name '*.cpp')

# include dependencies
INCLUDES  = $(shell find $(SRC_DIR) -name '*.h')

# external dependencies
EXTERNAL  = $(TOP_DIR)/external

# required dependencies
FLAGS        += -DGLEW_STATIC
GLEW_DIR      = $(TOP_DIR)/external/glew
FLAGS        += -I$(GLEW_DIR)/include
VPATH        += $(GLEW_DIR)/src
CSOURCES     += $(wildcard $(GLEW_DIR)/src/*.c)

# optional dependencies to include in static library (see Makefile.config)
ifeq ($(USE_EXTERNAL_LIBPNG), no)
PNG_DIR       = $(EXTERNAL)/libpng
FLAGS        += -I$(PNG_DIR)/include
VPATH        += $(PNG_DIR)/src
EXCLUDE       = $(wildcard $(PNG_DIR)/src/*_core.c)
CSOURCES     += $(filter-out $(EXCLUDE), $(wildcard $(PNG_DIR)/src/*.c))
endif
ifeq ($(USE_EXTERNAL_LIBJPEG), no)
JPG_DIR       = $(EXTERNAL)/libjpeg
FLAGS        += -I$(JPG_DIR)/include
VPATH        += $(JPG_DIR)/src
CSOURCES     += $(wildcard $(JPG_DIR)/src/*.c)
endif
ifeq ($(USE_EXTERNAL_GIFLIB), no)
GIF_DIR       = $(EXTERNAL)/giflib
FLAGS        += -I$(GIF_DIR)/include
VPATH        += $(GIF_DIR)/src
CSOURCES     += $(wildcard $(GIF_DIR)/src/*.c)
endif
ifeq ($(USE_EXTERNAL_LIB3DS), no)
3DS_DIR       = $(EXTERNAL)/lib3ds
FLAGS        += -I$(3DS_DIR)/include
VPATH        += $(3DS_DIR)/src
CSOURCES     += $(wildcard $(3DS_DIR)/src/*.c)
endif
ifeq ($(USE_EXTERNAL_PUGIXML), no)
PUGIXML_DIR   = $(EXTERNAL)/pugixml
FLAGS        += -I$(PUGIXML_DIR)/include
VPATH        += $(PUGIXML_DIR)/src
SOURCES      += $(wildcard $(PUGIXML_DIR)/src/*.cpp)
endif
ifeq ($(USE_EXTERNAL_OPENAL), no)
FLAGS        += -DAL_ALEXT_PROTOTYPES -DAL_BUILD_LIBRARY -DAL_LIBTYPE_STATIC
OPENAL_DIR    = $(EXTERNAL)/openal
FLAGS        += -I$(OPENAL_DIR)/include -I$(OPENAL_DIR)/OpenAL32/Include -I$(OPENAL_DIR)/Alc
CSOURCES     += $(wildcard $(OPENAL_DIR)/OpenAL32/*.c)
CSOURCES     += $(wildcard $(OPENAL_DIR)/Alc/backends/*.c)
CSOURCES     += $(wildcard $(OPENAL_DIR)/Alc/*.c)
VPATH        += $(OPENAL_DIR)/OpenAL32
VPATH        += $(OPENAL_DIR)/Alc
VPATH        += $(OPENAL_DIR)/Alc/backends
VPATH        += $(OPENAL_DIR)/Alc/effects
endif
ifeq ($(USE_EXTERNAL_THEORAPLAYER), no)
FLAGS        += -D__THEORA -D_LIB
LTP_DIR       = $(EXTERNAL)/theoraplayer
FLAGS        += -I$(LTP_DIR)/include/theoraplayer
SOURCES      += $(wildcard $(LTP_DIR)/src/*.cpp)
VPATH        += $(LTP_DIR)/src
YUV_DIR       = $(LTP_DIR)/src/YUV/C
FLAGS        += -I$(YUV_DIR)
CSOURCES     += $(wildcard $(YUV_DIR)/*.c)
VPATH        += $(YUV_DIR)
WRAPPER_DIR   = $(LTP_DIR)/src/Theora
FLAGS        += -I$(WRAPPER_DIR)
SOURCES      += $(wildcard $(WRAPPER_DIR)/*.cpp)
VPATH        += $(WRAPPER_DIR)
OGG_DIR       = $(LTP_DIR)/external/ogg
FLAGS        += -I$(OGG_DIR)/include
CSOURCES     += $(wildcard $(OGG_DIR)/src/*.c)
VPATH        += $(OGG_DIR)/src
VORBIS_DIR    = $(LTP_DIR)/external/vorbis
FLAGS        += -I$(VORBIS_DIR)/include -I$(VORBIS_DIR)/lib
CSOURCES     += $(wildcard $(VORBIS_DIR)/lib/*.c)
VPATH        += $(VORBIS_DIR)/lib
THEORA_DIR    = $(LTP_DIR)/external/theora
FLAGS        += -I$(THEORA_DIR)/include  -I$(THEORA_DIR)/lib
CSOURCES     += $(wildcard $(THEORA_DIR)/lib/*.c)
VPATH        += $(THEORA_DIR)/lib
CSOURCES     += $(wildcard $(THEORA_DIR)/lib/x86/*.c)
VPATH        += $(THEORA_DIR)/lib/x86
endif

# objects
OBJECTS   = $(patsubst %.cpp, $(OBJ_DIR)/%.o, $(SOURCES))
OBJECTS  += $(patsubst %.c,   $(OBJ_DIR)/%.o, $(CSOURCES))
OBJ_TREE  = $(sort $(dir $(OBJECTS)))

# additional dependencies
EXTRAS += extras/GLFW

# optional examples
ifneq (,$(wildcard examples))
SUBDIRS += examples
endif

# optional utilities
ifneq (,$(wildcard utils))
SUBDIRS += utils
endif

# optional applications
ifneq (,$(wildcard applications))
SUBDIRS += applications
endif

# optional modules
ifneq (,$(wildcard modules/BULLET))
MODULES += modules/BULLET
endif
ifneq (,$(wildcard modules/GEL))
MODULES += modules/GEL
endif
ifneq (,$(wildcard modules/ODE))
MODULES += modules/ODE
endif
ifneq (,$(wildcard modules/V-REP))
MODULES += modules/V-REP
endif

# build rules

all: lib $(SUBDIRS)

with-modules: all $(MODULES)

$(SUBDIRS) $(MODULES): lib | $(BIN_DIR)
	$(MAKE) -C $@

lib: $(LIB_TARGET)

$(LIB_TARGET): $(OBJECTS) | $(LIB_DIR)
	$(AR) $(ARFLAGS) $@ $?

$(OBJECTS):  $(INCLUDES) | $(OBJ_TREE)

$(BIN_DIR) $(LIB_DIR) $(OBJ_TREE):
	mkdir -p $@

$(OBJ_DIR)/%.o : %.cpp
	$(CXX) $(FLAGS) $(CXXFLAGS) -fPIC -c -o $@ $<

$(OBJ_DIR)/%.o : %.c
	$(CC) $(FLAGS) $(CFLAGS) -fPIC -c -o $@ $<

.PHONY: $(SUBDIRS) $(MODULES) $(EXTRAS)

clean:
	@for T in $(SUBDIRS); do make -C $$T $@; done
	-rm -f $(LIB_TARGET) *~
	-rm -rf $(LIB_DIR) $(OBJ_DIR)
