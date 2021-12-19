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

#ifndef AF_COMMUNICATION_PLUGINS
#define AF_COMMUNICATION_PLUGINS

#include "afFramework.h"


#ifdef AF_ENABLE_AMBF_COMM_SUPPORT
#include "ambf_server/Actuator.h"
#include "ambf_server/Camera.h"
#include "ambf_server/Light.h"
#include "ambf_server/Object.h"
#include "ambf_server/RigidBody.h"
#include "ambf_server/Sensor.h"
#include "ambf_server/Vehicle.h"
#include "ambf_server/World.h"

#include "sensor_msgs/PointCloud2.h"
#include "sensor_msgs/point_cloud2_iterator.h"
#endif

#ifdef AF_ENABLE_OPEN_CV_SUPPORT
#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>
#endif

using namespace ambf;

struct afRigidBodyState{
//    afRigidBody()
    void setChildrenNames(afRigidBodyPtr baseObj);
    void setJointNames(afRigidBodyPtr baseObj);
    void setJointPositions(afRigidBodyPtr baseObj);
    void setJointVelocities(afRigidBodyPtr baseObj);
    void setJointEfforts(afRigidBodyPtr baseObj);

    vector<float> m_jointPositions;
    vector<float> m_jointVelocities;
    vector<float> m_jointEfforts;
    vector<string> m_jointNames;
    vector<string> m_childrenNames;
};


class afObjectCommunicationPlugin: public afObjectPlugin{
public:
#ifdef AF_ENABLE_AMBF_COMM_SUPPORT
    virtual int init(const afBaseObjectPtr a_afObjectPtr, const afBaseObjectAttribsPtr a_objectAttribs) override;
    virtual void graphicsUpdate() override;
    virtual void physicsUpdate(double dt) override;
    virtual bool close() override;

    //! This method applies updates Wall and Sim Time for State Message.
    virtual void afUpdateTimes(const double a_wall_time, const double a_sim_time);

    void actuatorFetchCommand(afActuatorPtr, double);
    void actuatorUpdateState(afActuatorPtr, double);

    void cameraFetchCommand(afCameraPtr, double);
    void cameraUpdateState(afCameraPtr, double);

    void jointFetchCommand(afJointPtr, double);
    void jointUpdateState(afJointPtr, double);

    void lightFetchCommand(afLightPtr, double);
    void lightUpdateState(afLightPtr, double);

    void rigidBodyFetchCommand(afRigidBodyPtr, double);
    void rigidBodyUpdateState(afRigidBodyPtr, double);

    void sensorFetchCommand(afSensorPtr, double);
    void sensorUpdateState(afSensorPtr, double);

    void vehicleFetchCommand(afVehiclePtr, double);
    void vehicleUpdateState(afVehiclePtr, double);

    void pointCloudFetchCommand(afPointCloudPtr, double);
    void pointCloudUpdateState(afPointCloudPtr, double);

    void volumeFetchCommand(afVolumePtr, double);
    void volumeUpdateState(afVolumePtr, double);

public:

    //! AMBF ROS COMM
    std::shared_ptr<ambf_comm::Actuator> m_afActuatorCommPtr;
    std::shared_ptr<ambf_comm::Camera> m_afCameraCommPtr;
    std::shared_ptr<ambf_comm::Light> m_afLightCommPtr;
    std::shared_ptr<ambf_comm::Object> m_afObjectCommPtr;
    std::shared_ptr<ambf_comm::RigidBody> m_afRigidBodyCommPtr;
    std::shared_ptr<ambf_comm::Sensor> m_afSensorCommPtr;
    std::shared_ptr<ambf_comm::Vehicle> m_afVehicleCommPtr;
    std::shared_ptr<ambf_comm::PointCloudHandler> m_afPointCloudCommPtr;
protected:
    afRigidBodyState m_rbState;

private:
    afType m_objectType;


    afType m_commType;


    bool m_paramsSet=false;

    // Counter for the times we have written to ambf_comm API
    // This is only for internal use as it could be reset
    unsigned short m_write_count = 0;

    // Counter for the times we have read from ambf_comm API
    // This is only for internal use as it could be reset
    unsigned short m_read_count = 0;
#else
    virtual int init(const afBaseObjectPtr a_afObjectPtr, const afBaseObjectAttribsPtr a_objectAttribs){
        return -1;
    }
#endif
};


class afCameraDepthStreamerPlugin: public afObjectPlugin{
public:
#ifdef AF_ENABLE_AMBF_COMM_SUPPORT
    virtual int init(const afBaseObjectPtr a_afObjectPtr, const afBaseObjectAttribsPtr a_objectAttribs) override;
    virtual void graphicsUpdate() override;
    virtual void physicsUpdate(double) override;
    virtual bool close() override;

private:
    unsigned int m_publishInterval=10;
    afCameraPtr m_cameraPtr = nullptr;

    // Counter for the times we have written to ambf_comm API
    // This is only for internal use as it could be reset
    unsigned short m_write_count = 0;

    // Counter for the times we have read from ambf_comm API
    // This is only for internal use as it could be reset
    unsigned short m_read_count = 0;

    // Image Transport ROS Node
    ros::NodeHandle* m_rosNode;
    sensor_msgs::PointCloud2::Ptr m_depthPointCloudMsg;
    sensor_msgs::PointCloud2Modifier* m_depthPointCloudModifier = nullptr;
    ros::Publisher m_depthPointCloudPub;
#else
    virtual int init(const afBaseObjectPtr a_afObjectPtr, const afBaseObjectAttribsPtr a_objectAttribs){
        return -1;
    }
#endif
};


class afCameraVideoStreamerPlugin: public afObjectPlugin{
public:
#ifdef AF_ENABLE_OPEN_CV_SUPPORT
    virtual int init(const afBaseObjectPtr a_afObjectPtr, const afBaseObjectAttribsPtr a_objectAttribs) override;
    virtual void graphicsUpdate() override;
    virtual void physicsUpdate(double) override;
    virtual bool close() override;

private:
    unsigned int m_publishInterval=1;

    // Counter for the times we have written to ambf_comm API
    // This is only for internal use as it could be reset
    unsigned short m_write_count = 0;

    // Counter for the times we have read from ambf_comm API
    // This is only for internal use as it could be reset
    unsigned short m_read_count = 0;
    // Image Transport ROS Node
    ros::NodeHandle* m_rosNode;
    afCameraPtr m_cameraPtr = nullptr;
    // Open CV Image Matrix
    cv::Mat m_imageMatrix;
    // Image Transport CV Bridge Node
    static image_transport::ImageTransport *s_imageTransport;
    // Image Transport Publisher
    image_transport::Publisher m_imagePublisher;
#else
    virtual int init(const afBaseObjectPtr a_afObjectPtr, const afBaseObjectAttribsPtr a_objectAttribs){
        return -1;
    }
#endif
};


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
    unsigned short m_write_count = 0;

    // Counter for the times we have read from ambf_comm API
    // This is only for internal use as it could be reset
    unsigned short m_read_count = 0;


    std::shared_ptr<ambf_comm::World> m_afWorldCommPtr;
#else
    virtual int init(const afWorldPtr a_afWorld, const afWorldAttribsPtr a_worldAttribs){
        return -1;
    }
#endif
};

#endif
