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
#  $Date: 2016-02-25 20:00:08 +0100 (Thu, 25 Feb 2016) $
#  $Rev: 1991 $


# import settings
TOP_DIR = ../..
include $(TOP_DIR)/Makefile.common

# examples
SUBDIRS  = 01-mydevice \
           02-multi-devices \
           03-analytics \
           04-shapes \
           05-fonts \
           06-images \
           07-mouse-select \
           08-shaders \
           09-magnets \
           10-oring \
           11-effects \
           12-polygons \
           13-primitives \
           14-textures \
           15-paint \
           16-friction \
           17-shading \
           18-endoscope \
           19-space \
           20-map \
           21-object \
           22-chrome \
           23-tooth \
           24-turntable \
           25-sounds \
           26-video \
           27-multiframes \
           28-voxel-basic \
           29-voxel-isosurface \
           30-voxel-colormap \
           31-pointcloud

# rules
all: $(SUBDIRS)

$(SUBDIRS): glfw
	$(MAKE) -C $@

glfw:
	$(MAKE) -C $(TOP_DIR)/extras/GLFW

.PHONY: $(SUBDIRS)

clean:
	@for T in $(SUBDIRS); do make -C $$T $@; done
	-rm -f core *~ *.bak #*
