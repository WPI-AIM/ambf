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
#  $Date: 2016-05-12 16:00:10 +0200 (Thu, 12 May 2016) $
#  $Rev: 2019 $
#
#  ----------------------------------------------------------------------
#
#  This file is adapted from
#  github.com/tcbrindle/sdl2-cmake-scripts/blob/master/FindSDL2.cmake
#  available with the following license:
#  
#  Copyright 2003-2009 Kitware, Inc.
#  
#  Distributed under the OSI-approved BSD License (the "License");
#  see accompanying file Copyright.txt for details.
#  
#  This software is distributed WITHOUT ANY WARRANTY; without even the
#  implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
#  See the License for more information.


# platform detection
if (${CMAKE_SYSTEM_NAME} MATCHES Windows)
  set (OS "win")
  if (${CMAKE_CL_64})
    set (ARCH "x64")
  else ()
    set (ARCH "Win32")
  endif ()
elseif (${CMAKE_SYSTEM_NAME} MATCHES Darwin)
  set (OS "mac")
  if (${CMAKE_SIZEOF_VOID_P} EQUAL 8)
    set (ARCH "x86_64")
  else ()
    set (ARCH "i386")
  endif ()
elseif (${CMAKE_SYSTEM_NAME} MATCHES Linux) 
  set (OS "lin")
  if (${CMAKE_SIZEOF_VOID_P} EQUAL 8)
    set (ARCH "x86_64")
  else ()
    set (ARCH "i686")
  endif ()
endif ()

SET(SDL2_SEARCH_PATHS
	~/Library/Frameworks
	/Library/Frameworks
	/usr/local
	/usr
	/sw # Fink
	/opt/local # DarwinPorts
	/opt/csw # Blastwave
	/opt
	${CMAKE_CURRENT_LIST_DIR}
)

FIND_PATH(SDL2_INCLUDE_DIR SDL.h
	HINTS
	$ENV{SDL2DIR}
	PATH_SUFFIXES include/SDL2 include
	PATHS ${SDL2_SEARCH_PATHS}
)

FIND_LIBRARY(SDL2_LIBRARY_TEMP
	NAMES SDL2
	HINTS
	$ENV{SDL2DIR}
	PATH_SUFFIXES lib64 lib lib/${OS}-${ARCH}
	PATHS ${SDL2_SEARCH_PATHS}
)

IF(NOT SDL2_BUILDING_LIBRARY)
	IF(NOT ${SDL2_INCLUDE_DIR} MATCHES ".framework")
		# Non-OS X framework versions expect you to also dynamically link to
		# SDL2main. This is mainly for Windows and OS X. Other (Unix) platforms
		# seem to provide SDL2main for compatibility even though they don't
		# necessarily need it.
		FIND_LIBRARY(SDL2MAIN_LIBRARY
			NAMES SDL2main
			HINTS
			$ENV{SDL2DIR}
			PATH_SUFFIXES lib64 lib
			PATHS ${SDL2_SEARCH_PATHS}
		)
	ENDIF(NOT ${SDL2_INCLUDE_DIR} MATCHES ".framework")
ENDIF(NOT SDL2_BUILDING_LIBRARY)

# SDL2 may require threads on your system.
# The Apple build may not need an explicit flag because one of the
# frameworks may already provide it.
# But for non-OSX systems, I will use the CMake Threads package.
IF(NOT APPLE)
	FIND_PACKAGE(Threads)
ENDIF(NOT APPLE)

# MinGW needs an additional library, mwindows
# It's total link flags should look like -lmingw32 -lSDL2main -lSDL2 -lmwindows
# (Actually on second look, I think it only needs one of the m* libraries.)
IF(MINGW)
	SET(MINGW32_LIBRARY mingw32 CACHE STRING "mwindows for MinGW")
ENDIF(MINGW)

IF(SDL2_LIBRARY_TEMP)

	# For SDL2main
	IF(NOT SDL2_BUILDING_LIBRARY)
		IF(SDL2MAIN_LIBRARY)
			SET(SDL2_LIBRARY_TEMP ${SDL2MAIN_LIBRARY} ${SDL2_LIBRARY_TEMP})
		ENDIF(SDL2MAIN_LIBRARY)
	ENDIF(NOT SDL2_BUILDING_LIBRARY)

    # VisualStudio needs additional libraries
    IF(MSVC)
	    SET(SDL2_LIBRARY_TEMP ${SDL2_LIBRARY_TEMP} "glu32.lib;imm32.lib;version.lib")
    ENDIF(MSVC)

	# For OS X, SDL2 uses Cocoa as a backend so it must link to Cocoa.
	# CMake doesn't display the -framework Cocoa string in the UI even
	# though it actually is there if I modify a pre-used variable.
	# I think it has something to do with the CACHE STRING.
	# So I use a temporary variable until the end so I can set the
	# "real" variable in one-shot.
	IF(APPLE)
		SET(SDL2_LIBRARY_TEMP ${SDL2_LIBRARY_TEMP} "-framework Cocoa -framework CoreVideo -framework Carbon -framework ForceFeedback")
	ENDIF(APPLE)

	# For threads, as mentioned Apple doesn't need this.
	# In fact, there seems to be a problem if I used the Threads package
	# and try using this line, so I'm just skipping it entirely for OS X.
	IF(NOT APPLE)
		SET(SDL2_LIBRARY_TEMP ${SDL2_LIBRARY_TEMP} ${CMAKE_THREAD_LIBS_INIT})
	ENDIF(NOT APPLE)

	# For MinGW library
	IF(MINGW)
		SET(SDL2_LIBRARY_TEMP ${MINGW32_LIBRARY} ${SDL2_LIBRARY_TEMP})
	ENDIF(MINGW)

	# Set the final string here so the GUI reflects the final state.
	SET(SDL2_LIBRARY ${SDL2_LIBRARY_TEMP} CACHE STRING "Where the SDL2 Library can be found")
	# Set the temp variable to INTERNAL so it is not seen in the CMake GUI
	SET(SDL2_LIBRARY_TEMP "${SDL2_LIBRARY_TEMP}" CACHE INTERNAL "")
ENDIF(SDL2_LIBRARY_TEMP)

INCLUDE(FindPackageHandleStandardArgs)

FIND_PACKAGE_HANDLE_STANDARD_ARGS(SDL2 REQUIRED_VARS SDL2_LIBRARY SDL2_INCLUDE_DIR)