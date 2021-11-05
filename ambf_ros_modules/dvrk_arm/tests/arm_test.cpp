//==============================================================================
/*
    Software License Agreement (BSD License)
    Copyright (c) 2019, AMBF
    (http://practicepoint.wpi.edu)

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

    \author    <http://practicepoint.wpi.edu>
    \author    <amunawar@wpi.edu>
    \author    Adnan Munawar
    \version   $
*/
//==============================================================================

#include "dvrk_arm/Arm.h"




int main(int argc, char** argv){
    DVRK_Arm mtm("MTMR");
    sleep(1);
    std::vector<double> joint_pos;
    std::vector<double> joint_vel;
    std::vector<double> joint_effort;
    double fx, fy, fz;
//    for (int cnt = 0 ; cnt < 5000 ; cnt++) {
//        mtm.measured_jp(joint_pos);
//        mtm.measured_jf(joint_effort);
//        std::cout << '\r' << "Effort [" ;
//        for (int i = 0; i < joint_pos.size(); i++) {
//            std::cout << " " << joint_effort[i];
//        }
//        std::cout << "]";
//        usleep(10000);
//    }
    for (int cnt = 0 ; cnt < 5000 ; cnt++) {
        mtm.measured_cf_force(fx, fy, fz);
        std::cout << '\r' << " Fx: " << fx << std::endl << " Fy: " << fy << std::endl << " Fz: " << fz;
        std::cout << std::endl << "---------------------------" << std::endl;
        usleep(10000);
    }

    std::cout << std::endl;
    return 0;
}
