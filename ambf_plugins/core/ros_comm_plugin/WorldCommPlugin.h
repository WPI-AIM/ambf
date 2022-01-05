//==============================================================================
/*
    Software License Agreement (BSD License)
    Copyright (c) 2019-2021, AMBF
    (https://github.com/WPI-AIM/ambf)

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

    * Neither the name of authors nor the names of its contributors may
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

    \author    <amunawar@wpi.edu>
    \author    Adnan Munawar
*/
//==============================================================================

#ifndef AF_WORLDCOMM_PLUGIN
#define AF_WORLDCOMM_PLUGIN

#include "afFramework.h"


#ifdef AF_ENABLE_AMBF_COMM_SUPPORT
#include "ambf_server/World.h"
#endif

using namespace ambf;

class afWorldCommunicationPlugin: public afWorldPlugin{
public:
#ifdef AF_ENABLE_AMBF_COMM_SUPPORT
    virtual int init(const afWorldPtr a_afWorld, const afWorldAttribsPtr a_worldAttribs) override;
    virtual void graphicsUpdate() override;
    virtual void physicsUpdate(double dt) override;
    virtual bool close() override;

    void worldFetchCommand(afWorldPtr, double);
    void worldUpdateState(afWorldPtr, double);

private:
    bool m_paramsSet=false;

    // Counter for the times we have written to ambf_comm API
    // This is only for internal use as it could be reset
    unsigned int m_write_count = 0;

    // Counter for the times we have read from ambf_comm API
    // This is only for internal use as it could be reset
    unsigned int m_read_count = 0;


    std::shared_ptr<ambf_comm::World> m_afWorldCommPtr;
#else
    virtual int init(const afWorldPtr a_afWorld, const afWorldAttribsPtr a_worldAttribs){
        return -1;
    }
#endif
};

#endif
