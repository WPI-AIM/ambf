//==============================================================================
/*
    Software License Agreement (BSD License)
    Copyright (c) 2020, AMBF
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
    \version   1.0$
*/
//==============================================================================

#ifndef CDVRK_ArmH
#define CDVRK_ArmH

#include "Bridge.h"
#include "tf/tf.h"
#include "tf/LinearMath/Matrix3x3.h"
#include "mutex"
#include "dvrk_arm/Frame.h"
#include "tf/transform_broadcaster.h"
#include <cmath>

struct Command: public Frame{
public:
    Command(){
        force.setZero();
        moment.setZero();
    }
    ~Command(){}
    tf::Vector3 force;
    tf::Vector3 moment;
};


class DVRK_Arm: public States{
public:
    DVRK_Arm(const std::string &arm_name);
    ~DVRK_Arm();

    void set_origin_frame_pos(const double &x, const double &y, const double &);
    void set_origin_frame_pos(const geometry_msgs::Point &pos);
    void set_origin_frame_pos(const tf::Vector3 &pos);

    void set_origin_frame_rot(const double &roll, const double &pitch, const double &yaw);
    void set_origin_frame_rot(const tf::Quaternion &tf_quat);
    void set_origin_frame_rot(const geometry_msgs::Quaternion &gm_quat);
    void set_origin_frame_rot(const tf::Matrix3x3 &mat);

    void set_origin_frame(const tf::Vector3 &pos, const tf::Quaternion &tf_quat);
    void set_origin_frame(const tf::Vector3 &pos, const tf::Matrix3x3 &tf_mat);
    void set_origin_frame(const tf::Transform &trans);

    void affix_tip_frame_pos(const double &x, const double &y, const double &z);
    void affix_tip_frame_pos(const tf::Vector3 &pos);
    void affix_tip_frame_pos(const geometry_msgs::Point &pos);

    void affix_tip_frame_rot(const tf::Quaternion &tf_quat);
    void affix_tip_frame_rot(const geometry_msgs::Quaternion &gm_quat);
    void affix_tip_frame_rot(const double &quat_x, const double &quat_y, const double &quat_z, const double &quat_w);
    void affix_tip_frame_rot(const double &roll, const double &pitch, const double &yaw);

    void affix_tip_frame(const tf::Vector3 &pos, const tf::Quaternion &tf_quat);
    void affix_tip_frame(const tf::Vector3 &pos, const tf::Matrix3x3 &tf_mat);
    void affix_tip_frame(const tf::Transform &trans);

    bool set_force(const double &fx,const double &fy,const double &fz);
    bool set_moment(const double &nx,const double &ny,const double &nz);
    bool set_wrench(const double &fx,const double &fy,const double &fz,const double &nx,const double &ny,const double &nz);

    bool move_cp_pos(const double &x, const double &y, const double &z);
    bool move_cp_pos(const geometry_msgs::Point &pos);
    bool move_cp_pos(const tf::Vector3 &pos);

    bool move_cp_ori(const double &roll, const double &pitch, const double &yaw);
    bool move_cp_ori(const double &x, const double &y, const double &z, const double &w);
    bool move_cp_ori(const tf::Quaternion &tf_quat);
    bool move_cp_ori(const geometry_msgs::Quaternion &gm_quat);
    bool move_cp_ori(const tf::Matrix3x3 &mat);

    bool move_cp(geometry_msgs::PoseStamped &pose);
    bool move_cp(tf::Transform &trans);

    void measured_cp_pos(double &x, double &y, double &z);
    void measured_cp_pos(tf::Vector3 &pos);
    void measured_cp_pos(geometry_msgs::Point &pos);

    void measured_cp_ori(double &roll, double &pitch, double &yaw);
    void measured_cp_ori(double &x, double &y, double &z, double &w);
    void measured_cp_ori(tf::Quaternion &tf_quat);
    void measured_cp_ori(geometry_msgs::Quaternion &gm_quat);
    void measured_cp_ori(tf::Matrix3x3 &mat);

    void measured_cp(geometry_msgs::Pose &pose);
    void measured_cp(tf::Transform &trans);
    void measured_cf_force(double &fx, double &fy, double &fz);
    void measured_cf_moment(double &nx, double &ny, double &nz);
    void measured_cf(double &fx, double &fy, double &fz, double &nx, double &ny, double &nz);

    void measured_jp(std::vector<double> &jnt_pos);
    void measured_jv(std::vector<double> &jnt_vel);
    void measured_jf(std::vector<double> &jnt_effort);

    void measured_gripper_angle(double &pos);

    bool is_gripper_pressed(); //Presed or Released, for MTM
    bool is_clutch_pressed();
    bool is_coag_pressed();

    void set_mode(const std::string &state, bool lock_wrench_ori = true);

    inline bool is_available(){return m_bridge->_is_available();}
    inline bool in_effort_mode(){return m_bridge->_in_effort_mode();}
    inline bool in_cart_pos_mode(){return m_bridge->_in_cart_pos_mode();}
    inline bool in_jnt_pos_mode(){return m_bridge->_in_jnt_pos_mode();}

    bool start_pubs;
    bool gripper_closed;

    bool close();

private:

    void init();
    void handle_frames();
    void pose_fcn_cb(const geometry_msgs::PoseStamped &pose);
    void gripper_state_fcn_cb(const sensor_msgs::JointState &state);
    void joint_state_fcn_cb(const sensor_msgs::JointState &jnt);
    void wrench_fcn_cb(const geometry_msgs::WrenchStamped &wrench);
    void move_arm_cartesian(tf::Transform trans);
    void set_arm_wrench(tf::Vector3 &force, tf::Vector3 &wrench);
    // afxdTipFrame is the affixedTipFrame;

    typedef std::shared_ptr<Frame> FramePtr;
    FramePtr m_originFramePtr, m_afxdTipFramePtr, m_eeFramePtr, m_freeFramePtr;
    tf::Vector3 m_wrenchForce, m_wrenchMoment;
    std::vector<double> m_jointPos, m_jointVel, m_jointEffort;
    Command m_eeCmd;
    std::vector<FramePtr> m_frameptrVec;
    std::vector<FramePtr>::iterator m_frameIter;
//    boost::shared_ptr<tf::TransformBroadcaster> frame_broadcasterPtr;
    double m_gripper_angle;
    int m_counter;

    std::shared_ptr<DVRK_Bridge> m_bridge;
    std::mutex m_mutex;
};
#endif
