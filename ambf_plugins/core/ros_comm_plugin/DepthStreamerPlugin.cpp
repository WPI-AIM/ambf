#include "DepthStreamerPlugin.h"

#ifdef AF_ENABLE_OPEN_CV_SUPPORT
int afCameraDepthStreamerPlugin::init(const afBaseObjectPtr a_afObjectPtr, const afBaseObjectAttribsPtr a_objectAttribs)
{
    m_objectPtr = a_afObjectPtr;
    m_cameraPtr = (afCameraPtr)a_afObjectPtr;
    afCameraAttributes* camAttribs = (afCameraAttributes*) a_objectAttribs;

    m_depthPointCloudMsg.reset(new sensor_msgs::PointCloud2());
    m_depthPointCloudModifier = new sensor_msgs::PointCloud2Modifier(*m_depthPointCloudMsg);
    m_depthPointCloudModifier->setPointCloud2FieldsByString(2, "xyz", "rgb");
    m_depthPointCloudModifier->resize(camAttribs->m_publishImageResolution.m_width*camAttribs->m_publishImageResolution.m_height);
    m_rosNode = afROSNode::getNode();
    m_depthPointCloudPub = m_rosNode->advertise<sensor_msgs::PointCloud2>(m_cameraPtr->getQualifiedName() + "/DepthData", 1);

    m_publishInterval = camAttribs->m_publishDepthInterval;

    return 1;
}

void afCameraDepthStreamerPlugin::graphicsUpdate()
{
    if (m_write_count % m_publishInterval == 0){
        sensor_msgs::PointCloud2Iterator<float> pcMsg_x(*m_depthPointCloudMsg, "x");
        sensor_msgs::PointCloud2Iterator<float> pcMsg_y(*m_depthPointCloudMsg, "y");
        sensor_msgs::PointCloud2Iterator<float> pcMsg_z(*m_depthPointCloudMsg, "z");
        sensor_msgs::PointCloud2Iterator<uint8_t> pcMsg_r(*m_depthPointCloudMsg, "r");
        sensor_msgs::PointCloud2Iterator<uint8_t> pcMsg_g(*m_depthPointCloudMsg, "g");
        sensor_msgs::PointCloud2Iterator<uint8_t> pcMsg_b(*m_depthPointCloudMsg, "b");

        int width = m_cameraPtr->m_depthBufferColorImage->getWidth();
        int height = m_cameraPtr->m_depthBufferColorImage->getHeight();

        double noise = 0;

        for (int idx = 0 ; idx < width * height ; idx++, ++pcMsg_x, ++pcMsg_y, ++pcMsg_z, ++pcMsg_r, ++pcMsg_g, ++pcMsg_b){
            if (m_cameraPtr->getDepthNoiseModel()->isEnabled()){
                noise = m_cameraPtr->getDepthNoiseModel()->generate();
            }
            *pcMsg_x = m_cameraPtr->getDepthPointCloud()->getData()[idx * m_cameraPtr->getDepthPointCloud()->getNumFields() + 0] + noise;
            *pcMsg_y = m_cameraPtr->getDepthPointCloud()->getData()[idx * m_cameraPtr->getDepthPointCloud()->getNumFields() + 1];
            *pcMsg_z = m_cameraPtr->getDepthPointCloud()->getData()[idx * m_cameraPtr->getDepthPointCloud()->getNumFields() + 2];

            *pcMsg_r = m_cameraPtr->m_bufferColorImage->getData()[idx * 4 + 0];
            *pcMsg_g = m_cameraPtr->m_bufferColorImage->getData()[idx * 4 + 1];
            *pcMsg_b = m_cameraPtr->m_bufferColorImage->getData()[idx * 4 + 2];
        }

        m_depthPointCloudMsg->header.frame_id = m_cameraPtr->getName();
        m_depthPointCloudMsg->header.stamp.fromSec(m_cameraPtr->getRenderTimeStamp());

        m_depthPointCloudPub.publish(m_depthPointCloudMsg);
    }
    m_write_count++;
}

void afCameraDepthStreamerPlugin::physicsUpdate(double)
{

}

bool afCameraDepthStreamerPlugin::close()
{
    if (m_depthPointCloudModifier != nullptr){
        delete m_depthPointCloudModifier;
        m_depthPointCloudModifier = 0;
    }
    return true;
}
#endif

