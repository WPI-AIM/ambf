#! /bin/bash

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
#  $Author: conti $
#  $Date: 2015-12-17 06:26:05 +0100 (Thu, 17 Dec 2015) $
#  $Rev: 1869 $


# make sure we have the rights to install a shared library
if [ $(id -u) != 0 ]; then
  echo "This script requires su privileges"
  sudo "$0"
  exit
fi

# determine output location
ARCH=`uname -m`
CMPNAME='gcc'
COMPILER=$CMPNAME
LIB_DIR=lib/release/lin-$ARCH-$COMPILER

echo
echo "Installing libhdPhantom.so on $HOSTNAME ($ARCH)"
echo

# build
echo -n "building libhdPhantom.so shared library... "
make CC=gcc CXX=g++ > log.txt 2>&1
if [ $? -ne 0 ]; then
  echo; echo "*** build failed, see log.txt for details"
  exit -1
fi
echo "ok"

# copy
file=`basename $LIB_DIR/libhdPhantom.so*`
echo -n "copying $file to /usr/local/lib... "
cp $LIB_DIR/$file /usr/local/lib >> log.txt 2>&1
if [ $? -ne 0 ]; then
  echo; echo "*** copy failed, see log.txt for details"
  exit -1
fi
echo "ok"

# configure
echo -n "configuring dynamic linker bindings... "
ldconfig >> log.txt 2>&1
if [ $? -ne 0 ]; then
  echo; echo "*** configuration failed, see log.txt for details"
  exit -1
fi
echo "ok"

# create symlink
echo -n "setting symlink to libhdPhantom.so library... "
rm /usr/local/lib/libhdPhantom.so >> log.txt 2>&1
ln -s /usr/local/lib/$file /usr/local/lib/libhdPhantom.so >> log.txt 2>&1
if [ $? -ne 0 ]; then
  echo; echo "*** symlink creation failed, see log.txt for details"
  exit -1
fi
echo "ok"

# configure dynamic linker run-time bindings
sudo ldconfig

echo
echo libhdPhantom.so successfully installed
echo
