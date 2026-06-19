#include "VideoStreamerPlugin.h"
#include <cmath>

AF_REGISTER_OBJECT_PLUGIN(afCameraVideoStreamerPlugin);

#ifdef AF_ENABLE_OPEN_CV_SUPPORT

image_transport::ImageTransport* afCameraVideoStreamerPlugin::s_imageTransport = nullptr;
int afCameraVideoStreamerPlugin::init(const afBaseObjectPtr a_afObjectPtr, const afBaseObjectAttribsPtr a_objectAttribs)
{
    m_objectPtr = a_afObjectPtr;
    m_cameraPtr = (afCameraPtr)a_afObjectPtr;
    afCameraAttributes* camAttribs = (afCameraAttributes*) a_objectAttribs;
    m_rosNode = afROSNode::getNodeAndRegister(m_cameraPtr->getQualifiedName());
    if (s_imageTransport == nullptr) {
#if AMBF_ROS1
        s_imageTransport = new image_transport::ImageTransport(*m_rosNode);
#elif AMBF_ROS2
        s_imageTransport = new image_transport::ImageTransport(m_rosNode);
#endif
    }
    m_imagePublisher = s_imageTransport->advertise(m_cameraPtr->getQualifiedName() + "/ImageData", 1);
    ambf_ral::create_publisher<AMBF_RAL_MSG(sensor_msgs, CameraInfo)>
        (m_cameraInfoPublisher,
         m_rosNode,
         m_cameraPtr->getQualifiedName() + "/CameraInfo",
         1, false);

    m_publishInterval = camAttribs->m_publishImageInterval;
    return 1;
}

void afCameraVideoStreamerPlugin::updateCameraInfoMsg(){
    const int imageWidth = static_cast<int>(m_cameraPtr->m_bufferColorImage->getWidth());
    const int imageHeight = static_cast<int>(m_cameraPtr->m_bufferColorImage->getHeight());
    m_cameraInfoMsg.width = imageWidth;
    m_cameraInfoMsg.height = imageHeight;
    m_cameraInfoMsg.distortion_model = "plumb_bob";

    // AMBF projection from intrinsics is defined in afCamera::computeProjectionFromIntrinsics:
    // P00=2*fx/W, P01=2*s/W, P02=(W-2*cx)/W, P11=2*fy/H, P12=(-H+2*cy)/H.
    // Invert these to recover the pinhole intrinsics from the active OpenGL projection matrix.
    auto &proj = m_cameraPtr->getInternalCamera()->m_projectionMatrix;
    const double W = static_cast<double>(imageWidth);
    const double H = static_cast<double>(imageHeight);
    const double fx = 0.5 * W * proj(0, 0);
    const double s = 0.5 * W * proj(0, 1);
    const double cx = 0.5 * W * (1.0 - proj(0, 2));
    const double fy = 0.5 * H * proj(1, 1);
    const double cy = 0.5 * H * (1.0 + proj(1, 2));

#if AMBF_ROS1
    m_cameraInfoMsg.K = {fx, s, cx,
                         0.0, fy, cy,
                         0.0, 0.0, 1.0};;
    m_cameraInfoMsg.R = {1.0, 0.0, 0.0,
                         0.0, 1.0, 0.0,
                         0.0, 0.0, 1.0};
    m_cameraInfoMsg.P = {fx, s, cx, 0.0,
                         0.0, fy, cy, 0.0,
                         0.0, 0.0, 1.0, 0.0};;
#elif AMBF_ROS2
    m_cameraInfoMsg.k = {fx, s, cx,
                         0.0, fy, cy,
                         0.0, 0.0, 1.0};;
    m_cameraInfoMsg.r = {1.0, 0.0, 0.0,
                         0.0, 1.0, 0.0,
                         0.0, 0.0, 1.0};
    m_cameraInfoMsg.p = {fx, s, cx, 0.0,
                         0.0, fy, cy, 0.0,
                         0.0, 0.0, 1.0, 0.0};;
#endif
}

void afCameraVideoStreamerPlugin::graphicsUpdate()
{
    if (m_write_count % m_publishInterval == 0){
        // UGLY HACK TO FLIP ONCES BEFORE PUBLISHING AND THEN AGAIN AFTER TO HAVE CORRECT MAPPING
        // WITH THE COLORED DETPH POINT CLOUD
        m_cameraPtr->m_bufferColorImage->flipHorizontal();
        m_imageMatrix = cv::Mat(m_cameraPtr->m_bufferColorImage->getHeight(), m_cameraPtr->m_bufferColorImage->getWidth(), CV_8UC4, m_cameraPtr->m_bufferColorImage->getData());
        cv::cvtColor(m_imageMatrix, m_imageMatrix, cv::COLOR_RGBA2RGB);
        AMBF_RAL_MSG_PTR(sensor_msgs, Image) rosMsg = cv_bridge::CvImage(AMBF_RAL_MSG(std_msgs, Header)(), "rgb8", m_imageMatrix).toImageMsg();
        rosMsg->header.frame_id = m_cameraPtr->getName();
        rosMsg->header.stamp = ambf_ral::time_from_seconds(m_cameraPtr->getRenderTimeStamp());
        m_imagePublisher.publish(rosMsg);
        updateCameraInfoMsg();
        m_cameraInfoMsg.header.frame_id = rosMsg->header.frame_id;
        m_cameraInfoMsg.header.stamp = rosMsg->header.stamp;
        m_cameraInfoPublisher->publish(m_cameraInfoMsg);
        m_cameraPtr->m_bufferColorImage->flipHorizontal();
    }
    m_write_count++;
}

void afCameraVideoStreamerPlugin::physicsUpdate(double)
{

}

bool afCameraVideoStreamerPlugin::close()
{
    if (s_imageTransport != nullptr){
        delete s_imageTransport;
        s_imageTransport = nullptr;
    }
    afROSNode::destroyNode(m_cameraPtr->getQualifiedName());
    return true;
}

#endif // AF_ENABLE_OPEN_CV_SUPPORT
