//===========================================================================
/*
    Software License Agreement (BSD License)
    Copyright (c) 2003-2016, CHAI3D.
    (www.chai3d.org)

    All rights reserved.

    Redistribution and use in source and binary forms, with or without
    modification, are permitted provided that the following conditions
    are met:

    * Redistributions of source code must retain the above copyright
    notice, this list of conditions and the following disclaimer.

    * Redistributions in binary form must reproduce the above
    copyright notice, this list of conditions and the following
    disclaimer in the documentation and/or other materials provided
    with the distribution.

    * Neither the name of CHAI3D nor the names of its contributors may
    be used to endorse or promote products derived from this software
    without specific prior written permission.

    THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
    "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
    LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
    FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
    COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
    INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
    BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
    LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
    CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
    LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
    ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
    POSSIBILITY OF SUCH DAMAGE. 

    \author     Sebastien Grange
*/
//===========================================================================

//---------------------------------------------------------------------------
#include "tdLeap.h"
using namespace Leap;
//---------------------------------------------------------------------------
#include "chai3d.h"
using namespace chai3d;
//---------------------------------------------------------------------------

//=============================================================================
// GLOBAL VARIABLES
//=============================================================================

// Leap Motion controller
static Controller* _controller = NULL;

// internal Leap objects
static Frame*  _lastFrame;
static int64_t _lastFrameID = -1;
static Hand*   _lastHand[2];


//=============================================================================
// library internal management
// =============================================================================

void
_sleepMs(const unsigned int a_interval)
{
#if defined(WIN32) | defined(WIN64)
    Sleep(a_interval);
#endif

#if defined(LINUX) | defined (MACOSX)
    struct timespec t;
    t.tv_sec  = a_interval/1000;
    t.tv_nsec = (a_interval-t.tv_sec*1000)*1000000;
    nanosleep (&t, NULL);
#endif
}


void
_allocate()
{
    if (!_controller) {
        _controller  = new Controller;
        _lastFrame   = new Frame;
        _lastHand[0] = new Hand;
        _lastHand[1] = new Hand;
        _sleepMs(100);
    }
}


void
_deallocate()
{
    if (_controller) 
    {
        delete _controller;
        delete _lastFrame;
        delete _lastHand[0];
        delete _lastHand[1];
        _controller = NULL;
    }
}


//==========================================================================
// FUNCTIONS ACCESSIBLE FROM OUTSIDE
//==========================================================================

//==========================================================================
/*!
    Retrieves the number of devices of type Leap Motion.

    \fn     int __FNCALL tdLeapGetNumDevices()

    \return Returns the number of devices found, -1 if controller is not ready.
*/
//==========================================================================
int __FNCALL tdLeapGetNumDevices()
{
    if (!_controller || !_controller->isConnected())
    {
        return (-1);
    }
    else
    {
        return (int)(_controller->devices().count());
    }
}


//==========================================================================
/*!
    Open a connection to the device selected.

    \fn     int __FNCALL tdLeapOpen()

     \return Return 0 if success, otherwise -1.
*/
//==========================================================================
int __FNCALL tdLeapOpen()
{
    // make sure controller is allocated
    _allocate();

    // check if device is physically available
    if (!_controller || !_controller->isConnected())
    {
        return (-1);
    }

    // success
    return (0);
}


//==========================================================================
/*!
    Closes a connection to the device.

    \fn     int __FNCALL tdLeapClose()
    
    \return Return 0 if success, otherwise -1.
*/
//==========================================================================
int __FNCALL tdLeapClose()
{  
   _deallocate ();
  
    // error
    return (0);
}


//==========================================================================
/*!
    Update the global frame to the latest available.

    \fn       bool __FNCALL tdLeapUpdate()

    \return Return __true__ on success, __false__ otherwise.
*/
//==========================================================================
bool __FNCALL tdLeapUpdate()
{
    // check if device is physically available
    if (!_controller || !_controller->isConnected())
    {
        return (false);
    }

    // get latest frame
    Frame frame = _controller->frame(0);
    int64_t id = frame.id();

    // if the frame is valid
    if (frame.isValid() && id != _lastFrameID)
    {
        // store current (valid) frame into global static frame
       *_lastFrame   = frame;
        _lastFrameID = id;

        // get list of hands
        HandList hands = frame.hands();

        // two hands case
        if (hands.count() > 1)
        {
            // preserve previous assignment when possible
            if (hands[0].id() == _lastHand[0]->id() || hands[1].id() == _lastHand[1]->id())
            {
                *_lastHand[0] = hands[0];
                *_lastHand[1] = hands[1];
            }
            else if (hands[1].id() == _lastHand[0]->id() || hands[0].id() == _lastHand[1]->id())
            {
                *_lastHand[0] = hands[1];
                *_lastHand[1] = hands[0];
            }

            // otherwise, make sure the first hand is right-handed if there's a right-hand
            else if (hands[0].isLeft() && hands[0].isRight())
            {
                *_lastHand[0] = hands[1];
                *_lastHand[1] = hands[0];
            }
            else
            {
                *_lastHand[0] = hands[0];
                *_lastHand[1] = hands[1];
            }
        }

        // one hand case
        else if (hands.count() > 0)
        {
            // preserve previous assignment when possible
            if (hands[0].id() == _lastHand[0]->id())
            {
                *_lastHand[0] = hands[0];
            }
            else if (hands[0].id() == _lastHand[1]->id())
            {
                *_lastHand[1] = hands[0];
            }

            // otherwise, make sure that left hands are assigned to the second hand
            else if (hands[0].isLeft())
            {
                *_lastHand[1] = hands[0];
            }
            else
            {
                *_lastHand[0] = hands[0];
            }
        }
    }

    // one way or another, it worked
    return (true);
}


//==========================================================================
/*!
    Return the hand position from the tracking data retrieved by the last
    \ref tdLeapUpdate() call.

    \fn       bool __FNCALL tdLeapGetPosition(cVector3d a_position[2])

    \param a_position   Returned hand position.

    \return Return __true__ on success, __false__ otherwise.
*/
//==========================================================================
bool __FNCALL tdLeapGetPosition(cVector3d a_position[2])
{
    // check if device is physically available
    if (!_controller || !_controller->isConnected())
    {
        return (false);
    }

    // copy hand data
    for (int i=0; i<2; i++)
    {
        if (_lastHand[i]->isValid())
        {
            Vector center, pos;
            center = _lastHand[i]->frame().interactionBox().center();
            pos    = _lastHand[i]->palmPosition();
            a_position[i].set((pos.z-center.z)*1e-3,
                              (pos.x-center.x)*1e-3,
                              (pos.y-center.y)*1e-3);
        }
        else
        {
            a_position[i].set (0.0, 0.0, 0.0);
        }
    }

    return (true);
}


//==========================================================================
/*!
    Return the hand rotation from the tracking data retrieved by the last
    \ref tdLeapUpdate() call.

    \fn       bool __FNCALL tdLeapGetRotation(cMatrix3d a_rotation[2])

    \param a_rotation   Returned hand rotation.

    \return Return __true__ on success, __false__ otherwise.
*/
//==========================================================================
bool __FNCALL tdLeapGetRotation(cMatrix3d a_rotation[2])
{
    // check if device is physically available and we have a valid hand
    if (!_controller || !_controller->isConnected())
    {
        return (false);
    }

    // copy hand data
    for (int i=0; i<2; i++)
    {
        if (_lastHand[i]->isValid())
        {
            cMatrix3d frame;
            frame.identity();

            // populate data
            Matrix rot = _lastHand[i]->basis();

            // return result
            if (_lastHand[i]->isRight())
            {
                a_rotation[i].set(rot.zBasis.z, rot.xBasis.z, rot.yBasis.z,
                                  rot.zBasis.x, rot.xBasis.x, rot.yBasis.x,
                                  rot.zBasis.y, rot.xBasis.y, rot.yBasis.y);
                a_rotation[i].rotateAboutLocalAxisDeg(cVector3d(1,0,0),  90.0);
                a_rotation[i].rotateAboutLocalAxisDeg(cVector3d(0,0,1),  45.0);
            }
            else
            {
                a_rotation[i].set( rot.zBasis.z, -rot.xBasis.z,  rot.yBasis.z,
                                   rot.zBasis.x, -rot.xBasis.x,  rot.yBasis.x,
                                   rot.zBasis.y, -rot.xBasis.y,  rot.yBasis.y);
                a_rotation[i].rotateAboutLocalAxisDeg(cVector3d(1,0,0), -90.0);
                a_rotation[i].rotateAboutLocalAxisDeg(cVector3d(0,0,1), -45.0);
            }
        }
        else
        {
            a_rotation[i].identity();
        }
    }

    return (true);
}


//==========================================================================
/*!
    Return the hand pinching motion angle from the tracking data retrieved by the last
    \ref tdLeapUpdate() call.

    \fn       bool __FNCALL tdLeapGetGripperAngleRad(double &a_angle[2])

    \param a_angle   Returned hand pinching angle.

    \return Return __true__ on success, __false__ otherwise.
*/
//==========================================================================
bool __FNCALL tdLeapGetGripperAngleRad(double a_angle[2])
{
    // gripper maximum opening angle (in deg)
    const double OPEN_ANGLE = 30.0;

    // check if device is physically available and we have a valid hand
    if (!_controller || !_controller->isConnected())
    {
        return (false);
    }

    // copy hand data
    for (int i=0; i<2; i++)
    {
        if (_lastHand[i]->isValid())
        {
            // read gripper angle
            a_angle[i] = (1.0-_lastHand[i]->pinchStrength()) * OPEN_ANGLE * DEG_TO_RAD;
        }
        else
        {
            a_angle[i] = OPEN_ANGLE * DEG_TO_RAD;
        }
    }

    return (true);
}


//==========================================================================
/*!
    Return the hand open/close status from the tracking data retrieved by the last
    \ref tdLeapUpdate() call.

    \fn       bool __FNCALL tdLeapGetUserSwitches(unsigned int &a_userSwitches[2])

    \param a_userSwitches   Returned hand open/close status.

    \return Return __true__ on success, __false__ otherwise.
*/
//==========================================================================
bool __FNCALL tdLeapGetUserSwitches(unsigned int a_userSwitches[2])
{
    // check if device is physically available and we have a valid hand
    if (!_controller || !_controller->isConnected())
    {
        return (false);
    }

    // copy hand data
    for (int i=0; i<2; i++)
    {
        if (_lastHand[i]->isValid())
        {
            // return result
            if (_lastHand[i]->grabStrength() > 0.75) a_userSwitches[i] = 0x01;
            else                                    a_userSwitches[i] = 0x00;
        }
        else
        {
            a_userSwitches[i] = 0x00;
        }
    }

    return (true);
}


//==========================================================================
/*!
    Returns the latest frame from a given Leap Motion device.

    \fn       bool __FNCALL tdLeapGetFrame(void* &a_frame)

    \param a_frame  An (unallocated) pointer that will point to a new frame on the heap.
                    Cast to Leap::Frame to access all the LeapSDK functionality.

    \return Return __true__ on success, __false__ otherwise.

    \note
    It is the responsibility of the caller to deallocate (delete)
    the frame pointer afterwards.
*/
//==========================================================================
bool __FNCALL tdLeapGetFrame(void* &a_frame)
{
    // check if device is physically available
    if (!_controller || !_controller->isConnected())
    {
        return (false);
    }

    // return appropriate value
    if (_lastFrame->isValid())
    {
        a_frame = (void*)(new Frame(*_lastFrame));
        return (true);
    }
    else
    {
        // error
        a_frame = NULL;
        return (false);
    }
}
