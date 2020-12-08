//==============================================================================
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

    \author    <http://www.aimlab.wpi.edu>
    \author    Adnan Munawar, WPI.
    \author    <http://www.chai3d.org>
    \author    Francois Conti.
    \version   3.2.0 $Rev: 1869 $
*/
//==============================================================================


//------------------------------------------------------------------------------
#include "system/CGlobals.h"
#include "devices/CdVRKDevices.h"
//------------------------------------------------------------------------------
#if defined(C_ENABLE_AMBF_DVRK_DEVICE_SUPPORT)
//------------------------------------------------------------------------------

//------------------------------------------------------------------------------
namespace chai3d {
//------------------------------------------------------------------------------
std::vector<std::string> cDvrkDevice::m_dev_names;
cDvrkDevice::factory_create cDvrkDevice::create_fcn;
cDvrkDevice::factory_destroy cDvrkDevice::destroy_fcn;


//==============================================================================
/*!
    Constructor of cDvrkDevice.
*/
//==============================================================================
cDvrkDevice::cDvrkDevice(unsigned int a_deviceNumber)
{
    std::string dev_name = m_dev_names[a_deviceNumber];
    mtm_device = create_fcn(dev_name);
    // the connection to your device has not yet been established.
    m_deviceReady = false;
    //--------------------------------------------------------------------------
    // NAME:
    //--------------------------------------------------------------------------

    // haptic device model (see file "CGenericHapticDevice.h")
    m_specifications.m_model                         = C_HAPTIC_DVRK_MTM;

    // name of the device manufacturer, research lab, university.
    m_specifications.m_manufacturerName              = "Intuitive Surgical";

    // name of your device
    m_specifications.m_modelName                     = dev_name;


    //--------------------------------------------------------------------------
    // CHARACTERISTICS: (The following values must be positive or equal to zero)
    //--------------------------------------------------------------------------

    // the maximum force [N] the device can produce along the x,y,z axis.
    m_specifications.m_maxLinearForce                = 5.0;     // [N]

    // the maximum amount of torque your device can provide arround its
    // rotation degrees of freedom.
    m_specifications.m_maxAngularTorque              = 0.1;     // [N*m]


    // the maximum amount of torque which can be provided by your gripper
    m_specifications.m_maxGripperForce                = 3.0;     // [N]

    // the maximum closed loop linear stiffness in [N/m] along the x,y,z axis
    m_specifications.m_maxLinearStiffness             = 1000.0; // [N/m]

    // the maximum amount of angular stiffness
    m_specifications.m_maxAngularStiffness            = 1.0;    // [N*m/Rad]

    // the maximum amount of stiffness supported by the gripper
    m_specifications.m_maxGripperLinearStiffness      = 1000;   // [N*m]

    // the radius of the physical workspace of the device (x,y,z axis)
    m_specifications.m_workspaceRadius                = 0.2;     // [m]

    // the maximum opening angle of the gripper
    m_gripper_max_angle = cDegToRad(60);
    m_gripper_min_angle = cDegToRad(-60);
    m_specifications.m_gripperMaxAngleRad             = m_gripper_max_angle;


    ////////////////////////////////////////////////////////////////////////////
    /*
        DAMPING PROPERTIES:

        Start with small values as damping terms can be high;y sensitive to 
        the quality of your velocity signal and the spatial resolution of your
        device. Try gradually increasing the values by using example "01-devices" 
        and by enabling viscosity with key command "2".
    */
    ////////////////////////////////////////////////////////////////////////////
    
    // Maximum recommended linear damping factor Kv
    m_specifications.m_maxLinearDamping             = 20.0;   // [N/(m/s)]

    //! Maximum recommended angular damping factor Kv (if actuated torques are available)
    m_specifications.m_maxAngularDamping            = 0.0;    // [N*m/(Rad/s)]

    //! Maximum recommended angular damping factor Kv for the force gripper. (if actuated gripper is available)
    m_specifications.m_maxGripperAngularDamping     = 0.0;    // [N*m/(Rad/s)]


    //--------------------------------------------------------------------------
    // CHARACTERISTICS: (The following are of boolean type: (true or false)
    //--------------------------------------------------------------------------

    // does your device provide sensed position (x,y,z axis)?
    m_specifications.m_sensedPosition                = true;

    // does your device provide sensed rotations (i.e stylus)?
    m_specifications.m_sensedRotation                = true;

    // does your device provide a gripper which can be sensed?
    m_specifications.m_sensedGripper                 = true;

    // is you device actuated on the translation degrees of freedom?
    m_specifications.m_actuatedPosition              = true;

    // is your device actuated on the rotation degrees of freedom?
    m_specifications.m_actuatedRotation              = false;

    // is the gripper of your device actuated?
    m_specifications.m_actuatedGripper               = false;

    if(std::strcmp(dev_name.c_str(), "MTMR") == 0){
    // can the device be used with the left hand?
    m_specifications.m_leftHand                      = false;
    // can the device be used with the right hand?
    m_specifications.m_rightHand                     = true;
    }
    else if(std::strcmp(dev_name.c_str(), "MTML") == 0){
    // can the device be used with the left hand?
    m_specifications.m_leftHand                      = true;
    // can the device be used with the right hand?
    m_specifications.m_rightHand                     = false;
    }




        
    //sleep(8.0);
    if(mtm_device->is_available()){
        m_deviceAvailable = true;
        tf::Transform home_trans, tip_trans;
        tf::Quaternion home_rot, tip_rot;
        if(std::strcmp(dev_name.c_str(), "MTMR") == 0){
            home_trans.setOrigin(tf::Vector3(-0.181025, -0.0163, -0.2620));
            tip_trans.setOrigin(tf::Vector3(0,0,0));

            tip_rot.setRPY(0, M_PI/2, 0);
            home_rot.setRPY(0, 0, -1.57079);
        }
        else if(std::strcmp(dev_name.c_str(), "MTML") == 0){
            home_trans.setOrigin(tf::Vector3(0.181025, -0.0163, -0.2620));
            tip_trans.setOrigin(tf::Vector3(0,0,0));

            tip_rot.setRPY(0, M_PI/2, 0);
            home_rot.setRPY(0, 0, -1.57079);
        }

        home_trans.setRotation(home_rot);
        tip_trans.setRotation(tip_rot);

        mtm_device->set_origin_frame(home_trans);
        mtm_device->affix_tip_frame(tip_trans);

    }
    else{
        m_deviceAvailable = false; // this value should become 'true' when the device is available.
    }
}


//==============================================================================
/*!
    Destructor of cDvrkDevice.
*/
//==============================================================================
cDvrkDevice::~cDvrkDevice()
{
    // close connection to device
    if (m_deviceReady)
    {
        close();
    }
    destroy_fcn(mtm_device);

}


//==============================================================================
/*!
    This method opens a connection to your device.

    \return __true__ if the operation succeeds, __false__ otherwise.
*/
//==============================================================================
bool cDvrkDevice::open()
{
    // check if the system is available
    if (!m_deviceAvailable) return (C_ERROR);

    // if system is already opened then return
    if (m_deviceReady) return (C_ERROR);

    bool result = C_ERROR; // this value will need to become "C_SUCCESS" for the device to be marked as ready.

    // result = openConnectionToMyDevice();
    result = mtm_device->is_available();
    mtm_device->set_mode(mtm_device->_m_effort_mode);


    // update device status
    if (result)
    {
        m_deviceReady = true;
        return (C_SUCCESS);
    }
    else
    {
        m_deviceReady = false;
        return (C_ERROR);
    }
}


//==============================================================================
/*!
    This method closes the connection to your device.

    \return __true__ if the operation succeeds, __false__ otherwise.
*/
//==============================================================================
bool cDvrkDevice::close()
{
    // check if the system has been opened previously
    if (!m_deviceReady) return (C_ERROR);

    bool result = C_SUCCESS; // if the operation fails, set value to C_ERROR.

    // *** INSERT YOUR CODE HERE ***
    // result = closeConnectionToMyDevice()

    // update status
    mtm_device->set_force(0,0,0);
    mtm_device->set_moment(0,0,0);
    m_deviceReady = false;
    result = mtm_device->close();

    return (result);
}


//==============================================================================
/*!
    This method calibrates your device.

    \return __true__ if the operation succeeds, __false__ otherwise.
*/
//==============================================================================
bool cDvrkDevice::calibrate(bool a_forceCalibration)
{
    // check if the device is read. See step 3.
    if (!m_deviceReady) return (C_ERROR);

    bool result = C_SUCCESS;

    // *** INSERT YOUR CODE HERE ***

    // error = calibrateMyDevice()
    //result = mtmr_device.set_mode(mtmr_device._m_effort_mode);
    return (result);
}


//==============================================================================
/*!
    This method returns the number of devices available from this class of device.

    \return __true__ if the operation succeeds, __false__ otherwise.
*/
//==============================================================================
unsigned int cDvrkDevice::getNumDevices()
{
    int numberOfDevices = 0;
    const std::string libname = "libdvrk_arm.so";
    void* handle = dlopen(libname.c_str(), RTLD_NOW);
    if (handle == NULL){
        std::cerr << dlerror();
        return 0;
    }
    else{
        std::cerr << "Found Lib: " << libname.c_str() << std::endl;
    }
    typedef std::vector<std::string> (*fcn_signature)();
    dlerror();
    fcn_signature get_num_devs = (fcn_signature)dlsym(handle, "get_active_arms");
    if(get_num_devs == NULL){
        std::cerr << dlerror() << std::endl;
        return 0;
    }
    int n_devs = get_num_devs().size();
    if ( n_devs > 0){
        m_dev_names.resize(n_devs);
        m_dev_names = get_num_devs();
    }
    dlerror();
    create_fcn = (factory_create)dlsym(handle, "create");
    destroy_fcn = (factory_destroy)dlsym(handle, "destroy");

    if((!create_fcn) || (!destroy_fcn)){
        std::cerr << dlerror() << std::endl;
        return 0;
    }
    std::cout << "No of dVRK Masters detected: "<< n_devs << std::endl;

    numberOfDevices = n_devs;
    dlclose(handle);
    return (numberOfDevices);
}


//==============================================================================
/*!
    This method returns the position of your device. Units are meters [m].

    \param   a_position  Return value.

    \return __true__ if the operation succeeds, __false__ otherwise.
*/
//==============================================================================
bool cDvrkDevice::getPosition(cVector3d& a_position)
{
    // check if the device is read. See step 3.
    if (!m_deviceReady) return (C_ERROR);

    bool result = C_SUCCESS;
    double x,y,z;

    mtm_device->measured_cp_pos(x,y,z);

    // store new position values
    a_position.set(x, y, z);

    // estimate linear velocity
    estimateLinearVelocity(a_position);

    // exit
    return (result);
}


//==============================================================================
/*!
    This method returns the orientation frame of your device end-effector

    \param   a_rotation  Return value.

    \return __true__ if the operation succeeds, __false__ otherwise.
*/
//==============================================================================
bool cDvrkDevice::getRotation(cMatrix3d& a_rotation)
{
    // check if the device is read. See step 3.
    if (!m_deviceReady) return (C_ERROR);

    bool result = C_SUCCESS;

    // variables that describe the rotation matrix
    double r00, r01, r02, r10, r11, r12, r20, r21, r22;
    cMatrix3d frame;
    tf::Matrix3x3 rot_mat;
    mtm_device->measured_cp_ori(rot_mat);
    tf::Vector3 col0 = rot_mat.getColumn(0);
    tf::Vector3 col1 = rot_mat.getColumn(1);
    tf::Vector3 col2 = rot_mat.getColumn(2);

    r00 = col0.getX();  r01 = col1.getX();  r02 = col2.getX();
    r10 = col0.getY();  r11 = col1.getY();  r12 = col2.getY();
    r20 = col0.getZ();  r21 = col1.getZ();  r22 = col2.getZ();

    frame.set(r00, r01, r02, r10, r11, r12, r20, r21, r22);

    // store new rotation matrix
    a_rotation = frame;

    // estimate angular velocity
    estimateAngularVelocity(a_rotation);

    // exit
    return (result);
}


//==============================================================================
/*!
    This method returns the gripper angle in radian.

    \param   a_angle  Return value.

    \return __true__ if the operation succeeds, __false__ otherwise.
*/
//==============================================================================
bool cDvrkDevice::getGripperAngleRad(double& a_angle)
{
    // check if the device is read. See step 3.
    if (!m_deviceReady) return (C_ERROR);

    ////////////////////////////////////////////////////////////////////////////
    /*
        STEP 9:
        Here you may implement code which reads the position angle of your
        gripper. The result must be returned in radian.

        If the operation fails return an error code such as C_ERROR for instance.
    */
    ////////////////////////////////////////////////////////////////////////////

    bool result = C_SUCCESS;

    // *** INSERT YOUR CODE HERE, MODIFY CODE below ACCORDINGLY ***

    // return gripper angle in radian
    // a_angle = getGripperAngleInRadianFromMyDevice();
    mtm_device->measured_gripper_angle(a_angle);
    a_angle = cClamp(a_angle, m_gripper_min_angle, m_gripper_max_angle);
    a_angle = (a_angle - m_gripper_min_angle) / (m_gripper_max_angle - m_gripper_min_angle);
    // estimate gripper velocity
    estimateGripperVelocity(a_angle);

    // exit
    return (result);
}


//==============================================================================
/*!
    This method sends a force [N] and a torque [N*m] and gripper torque [N*m] 
    to your haptic device.

    \param   a_force  Force command.
    \param   a_torque  Torque command.
    \param   a_gripperForce  Gripper force command.

    \return __true__ if the operation succeeds, __false__ otherwise.
*/
//==============================================================================
bool cDvrkDevice::setForceAndTorqueAndGripperForce(const cVector3d& a_force,
                                                       const cVector3d& a_torque,
                                                       const double a_gripperForce)
{
    // check if the device is read. See step 3.
    if (!m_deviceReady) return (C_ERROR);

    bool result = C_SUCCESS;

    // store new force value.
    m_prevForce = a_force;
    m_prevTorque = a_torque;
    m_prevGripperForce = a_gripperForce;

    // retrieve force, torque, and gripper force components in individual variables
    double fx = a_force(0);
    double fy = a_force(1);
    double fz = a_force(2);

    double tx = a_torque(0);
    double ty = a_torque(1);
    double tz = a_torque(2);

    double gf = a_gripperForce;

    mtm_device->set_wrench(fx, fy, fz, 0, 0, 0);
    // setForceToMyDevice(fx, fy, fz);
    // setTorqueToMyDevice(tx, ty, tz);
    // setForceToGripper(fg);
    // exit
    return (result);
}


//==============================================================================
/*!
    This method returns status of all user switches 
    [__true__ = __ON__ / __false__ = __OFF__].

    \param  a_userSwitches  Return the 32-bit binary mask of the device buttons.

    \return __true__ if the operation succeeds, __false__ otherwise.
*/
//==============================================================================
bool cDvrkDevice::getUserSwitches(unsigned int& a_userSwitches)
{
    // check if the device is read. See step 3.
    if (!m_deviceReady) return (C_ERROR);

    ////////////////////////////////////////////////////////////////////////////
    /*
        STEP 11:

        Here you shall implement code that reads the status all user switches 
        on your device. For each user switch, set the associated bit on variable
        a_userSwitches. If your device only has one user switch, then set 
        a_userSwitches to 1, when the user switch is engaged, and 0 otherwise.
    */
    ////////////////////////////////////////////////////////////////////////////

    // *** INSERT YOUR CODE HERE **
    a_userSwitches = 0;
    int gripper_bit = 0;
    int clutch_bit = 1;
    int coag_bit = 2;
    if(mtm_device->is_gripper_pressed())
        a_userSwitches |= (1<<gripper_bit);
    else
        a_userSwitches &= ~(1<<gripper_bit);
    if(mtm_device->is_clutch_pressed())
        a_userSwitches |= (1<<clutch_bit);
    else
        a_userSwitches &= ~(1<<clutch_bit);
    if(mtm_device->is_coag_pressed())
        a_userSwitches |= (1<<coag_bit);
    else
        a_userSwitches &= ~(1<<coag_bit);

    return (C_SUCCESS);
}


//------------------------------------------------------------------------------
}       // namespace chai3d
//------------------------------------------------------------------------------
#endif  // C_ENABLE_CUSTOM_DEVICE_SUPPORT
//------------------------------------------------------------------------------
