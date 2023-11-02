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

#include "dvrk_arm/FootPedals.h"

DVRK_FootPedals::DVRK_FootPedals(){
}

void DVRK_FootPedals::init_footpedals(std::shared_ptr<ros::NodeHandle> n){
    std::string prefix = "/footpedals/";
    m_buttonHandles[ButtonEnum::CLUTCH] = new ButtonHandle(n, prefix + "clutch");
    m_buttonHandles[ButtonEnum::COAG] = new ButtonHandle(n, prefix + "coag");
    m_buttonHandles[ButtonEnum::OPERATORPRESENT] = new ButtonHandle(n, prefix + "operatorpresent");
    m_buttonHandles[ButtonEnum::CAMERA] = new ButtonHandle(n, prefix + "camera");
    m_buttonHandles[ButtonEnum::CAM_PLUS] = new ButtonHandle(n, prefix + "cam_plus");
    m_buttonHandles[ButtonEnum::CAM_MINUS] = new ButtonHandle(n, prefix + "cam_minus");

}

DVRK_FootPedals::~DVRK_FootPedals(){
    std::map<ButtonEnum, ButtonHandle*>::iterator it;
    it = m_buttonHandles.begin();
    for (; it!= m_buttonHandles.end() ; ++it){
        delete it->second;
    }
//    std::cerr << "CLOSING DVRK_FOOTPEDALS" << std::endl;
}
