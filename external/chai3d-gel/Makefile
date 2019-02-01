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
#  $Date: 2016-08-22 16:06:31 +0200 (Mon, 22 Aug 2016) $
#  $Rev: 2093 $


# global path
TOP_DIR = .

# commong settings
include $(TOP_DIR)/Makefile.common

# directories
OBJ_DIR   = $(TOP_DIR)/obj/$(CFG)/$(OS)-$(ARCH)-$(COMPILER)
SRC_DIR   = $(TOP_DIR)/src
VPATH     = $(SRC_DIR)

# main sources
SOURCES   = $(wildcard $(SRC_DIR)/*.cpp)

# main includes
INCLUDES  = $(wildcard $(SRC_DIR)/*.h)

# objects
OBJECTS   = $(patsubst %.cpp, $(OBJ_DIR)/%.o, $(SOURCES))
OBJ_TREE  = $(sort $(dir $(OBJECTS)))

# optional examples
ifneq (,$(wildcard examples))
SUBDIRS += examples
endif

# target rules
all: lib $(SUBDIRS)

$(SUBDIRS): chai3d lib | $(BIN_DIR)
	$(MAKE) -C $@

lib: $(LIB_TARGET)

$(LIB_TARGET): $(OBJECTS) | $(LIB_DIR)
	$(AR) $(ARFLAGS) $@ $?

$(OBJECTS): $(INCLUDES) | $(OBJ_TREE)

$(BIN_DIR) $(LIB_DIR) $(OBJ_TREE):
	mkdir -p $@

chai3d:
	$(MAKE) -C $(CHAI3D) lib

.PHONY: $(SUBDIRS)
	
# object file compilation
$(OBJ_DIR)/%.o : %.cpp
	$(CXX) $(FLAGS) $(CXXFLAGS) -fPIC -c -o $@ $<
$(OBJ_DIR)/%.o : %.c
	$(CC) $(FLAGS) $(CFLAGS) -fPIC -c -o $@ $<	

clean:
	@for T in $(SUBDIRS); do make -C $$T $@; done
	-rm -f $(LIB_TARGET) *~
	-rm -rf $(LIB_DIR) $(OBJ_DIR)
