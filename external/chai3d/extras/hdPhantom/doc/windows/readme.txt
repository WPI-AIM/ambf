
RELEASE NOTE:

1.
In order for hdPhantom32.dll and hdPhantom64.dll to operate 
correctly under Windows XP, it is important to compile these 
files using Miscrosoft Visual Studio 2010.

The problem comes from the fact that procedure entry point 
GetTickCount64 cannot be located in the dynamic link library 
KERNEL32.dll while running in Windows XP.

2.
Make sure to compile in RELEASE mode and to also copy output 
files to ./bin directory of each module.

________________________

(C) 2003-2016 by CHAI3D

All Rights Reserved.

