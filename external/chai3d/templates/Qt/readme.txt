
CHAI3D - Qt template
installing dependencies and getting started
-------------------------------------------


1. DEPENDENCIES
---------------

This application requires the free Qt library and tools to be installed on your system.
Qt can be downloaded from the qt.io website.

Additionally, we recommend installing CMake (available from the www.cmake.org website).


2. GETTING STARTED
------------------

On all platforms, with Qt and CMake installed, proceed as follows to build this Qt application:

1. open chai3d/CMakeLists.txt as a project in the Qt Creator application

2. in the configuration dialog, add the following options to CMake:

  -DCMAKE_BUILD_TYPE=Release -Wno-dev
  
  and press "Run CMake"
  
3. build the CHAI3D project

4. open the file CMakeLists.txt from this application folder in the Qt Creator application

5. in the configuration dialog, add the following options to CMake:

  -DCMAKE_BUILD_TYPE=Release -Wno-dev
  
  and press "Run CMake". CMake should automatically locate the CHAI3D and Qt dependencies.
 
6. build the application


3. PLATFORM SPECIFIC TIPS
-------------------------

3.1. USING CMAKE FROM THE COMMAND LINE

On Microsoft Windows and Mac OS X, CMake requires the CMAKE_PREFIX_PATH
environment variable to include the Qt development folder:

* on Microsoft Windows, you can define CMAKE_PREFIX_PATH from a DOS prompt
  before running CMake by using the following command:

  set CMAKE_PREFIX_PATH=C:\Qt\5.4\msvc2013_opengl
  
* on Mac OS X, you can define CMAKE_PREFIX_PATH from a terminal
  before running CMake by using the following command:

  export CMAKE_PREFIX_PATH=/Developer/Qt/5.4/clang_64


3.2 USING THE QT VISUAL STUDIO ADD-IN

On Microsoft Windows, you can build this application using the provided Visual Studio solution
and project files. This requires the Qt Visual Studio Add-in, which is available from the qt.io website.
Once the add-in is installed and configured, the project will build like any other Visual Studio applciation.


________________________
(C) 2003-2016 by CHAI3D
All Rights Reserved.