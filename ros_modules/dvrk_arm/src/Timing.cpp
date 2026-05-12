/*********************************************************************
*
* Software License Agreement (BSD License)
*
*  Copyright (c) 2009, Willow Garage, Inc.
*  All rights reserved.
*
*  Redistribution and use in source and binary forms, with or without
*  modification, are permitted provided that the following conditions
*  are met:
*
*   * Redistributions of source code must retain the above copyright
*     notice, this list of conditions and the following disclaimer.
*   * Redistributions in binary form must reproduce the above
*     copyright notice, this list of conditions and the following
*     disclaimer in the documentation and/or other materials provided
*     with the distribution.
*   * Neither the name of the Willow Garage nor the names of its
*     contributors may be used to endorse or promote products derived
*     from this software without specific prior written permission.
*
*  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
*  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
*  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
*  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
*  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
*  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
*  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
*  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
*  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
*  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
*  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
*  POSSIBILITY OF SUCH DAMAGE.
*
* Author: Eitan Marder-Eppstein
* Contributor: Adnan Munawar
*********************************************************************/

#include "dvrk_arm/Timing.h"

namespace ros {

DVRK_Rate::DVRK_Rate(double frequency, bool print_time_info)
: start_(ambf_ral::wall_now())
, expected_cycle_time_(ambf_ral::duration_from_seconds(1.0 / frequency))
, actual_cycle_time_(ambf_ral::duration_from_seconds(0.0))
, min_cycle_time_(ambf_ral::duration_from_seconds(1.0))
, max_cycle_time_(ambf_ral::duration_from_seconds(0.0))
, prnt_info(print_time_info)
, delay_no(0)
, packet_no(0)
, mean_cycle_time(0.0)
, total_cycle_time(0.0)
{ }

DVRK_Rate::DVRK_Rate(const ambf_ral::duration_t & d)
  : start_(ambf_ral::wall_now())
  , expected_cycle_time_(d)
  , actual_cycle_time_(ambf_ral::duration_from_seconds(0.0))
  , min_cycle_time_(ambf_ral::duration_from_seconds(1.0))
  , max_cycle_time_(ambf_ral::duration_from_seconds(0.0))
  , delay_no(0)
  , packet_no(0)
  , mean_cycle_time(0.0)
  , total_cycle_time(0.0)
  , prnt_info(false)
{ }



bool DVRK_Rate::sleep()
{
  ambf_ral::time_t expected_end = start_ + expected_cycle_time_;

  ambf_ral::time_t actual_end = ambf_ral::wall_now();

  // detect backward jumps in time
  if (actual_end < start_)
  {
    std::cerr << "Time shift backwards" << std::endl;
    expected_end = actual_end + expected_cycle_time_;
  }

  //calculate the time we'll sleep for
  ambf_ral::duration_t sleep_time = expected_end - actual_end;
  ambf_ral::time_t sleep_till = start_ + expected_cycle_time_;

  //set the actual amount of time the loop took in case the user wants to know
  actual_cycle_time_ = actual_end - start_;

  //make sure to reset our start time
  start_ = expected_end;
  packet_no++;
    total_cycle_time += ambf_ral::to_sec(actual_cycle_time_);
  mean_cycle_time = (total_cycle_time/packet_no);
    if (ambf_ral::to_sec(actual_cycle_time_) > mean_cycle_time){
      max_cycle_time_ = actual_cycle_time_;
  }
    if (ambf_ral::to_sec(actual_cycle_time_) < mean_cycle_time){
      min_cycle_time_ = actual_cycle_time_;
  }

  //if we've taken too much time we won't sleep
  if(sleep_time <= ambf_ral::duration_from_seconds(0.0))
  {
      delay_no++;
      if(prnt_info){
      std::cerr<< "Delay No: "<<delay_no<<std::endl
               << "Packets Received: "<<packet_no<<std::endl
               << "Min      Cycle Time(s) :"<< ambf_ral::to_sec(min_cycle_time_) << std::endl
               << "Max      Cycle Time(s) :"<< ambf_ral::to_sec(max_cycle_time_) << std::endl
               << "Mean     Cycle Time(s) :"<< mean_cycle_time << std::endl
               << "Actual   Cycle Time(s) :"<< ambf_ral::to_sec(actual_cycle_time_) << std::endl
               << "Expected Cycle Time(s) :"<< ambf_ral::to_sec(expected_cycle_time_) << std::endl
               << "-----------------------------------------------"<< std::endl;
      }
      // if we've jumped forward in time, or the loop has taken more than a full extra
      // cycle, reset our cycle
      if (actual_end > expected_end + expected_cycle_time_)
      {
          start_ = actual_end;
      }
      return true;
  }

//  std::cerr <<" Cur   Time  : " << ros::Time::now().toSec() << std::endl
//            <<" Sleep Target: "<< sleep_till.toSec() << std::endl;
  while (ambf_ral::to_sec(ambf_ral::wall_now()) <= ambf_ral::to_sec(sleep_till)){

  }
//  std::cerr << " After Sleep: " << ros::Time::now().toSec()<< std::endl
//  << "-----------------------------------------------"<< std::endl;

  return true;
}

void DVRK_Rate::reset()
{
  start_ = ambf_ral::wall_now();
}

ambf_ral::duration_t DVRK_Rate::cycleTime() const
{
  return actual_cycle_time_;
}
}
