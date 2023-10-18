#include "WorldCommPlugin.h"


#ifdef AF_ENABLE_AMBF_COMM_SUPPORT
int afWorldCommunicationPlugin::init(const afWorldPtr a_afWorld, const afWorldAttribsPtr a_worldAttribs)
{
    m_worldPtr = a_afWorld;

    if (m_worldPtr == nullptr){
        cerr << "ERROR! WORLD IS NULLPTR, FAILED TO INITIALIZE COMMUNICATION PLUGIN" << endl;
        return 0;
    }

    string objName = m_worldPtr->getName();
    string objNamespace = m_worldPtr->getNamespace();
    int minFreq = m_worldPtr->getMinPublishFrequency();
    int maxFreq = m_worldPtr->getMaxPublishFrequency();
    double timeOut = 0.5;

    bool success = false;

    m_afWorldCommPtr.reset(new ambf_comm::World(objName, objNamespace, minFreq, maxFreq, timeOut));
    m_afWorldCommPtr->enableComm();
    success = true;

    return success;
}

void afWorldCommunicationPlugin::graphicsUpdate()
{
    if (m_paramsSet == false){
        // Create a default point cloud to listen to
        m_afWorldCommPtr->append_point_cloud_topic(m_worldPtr->getQualifiedName() + "/" + "point_cloud");
        m_afWorldCommPtr->set_params_on_server();
        m_paramsSet = true;
    }
}

void afWorldCommunicationPlugin::physicsUpdate(double dt)
{
    worldFetchCommand(m_worldPtr, dt);
    worldUpdateState(m_worldPtr, dt);

}

bool afWorldCommunicationPlugin::close()
{
    afROSNode::destroyNode();
    return 1;
}

void afWorldCommunicationPlugin::worldFetchCommand(afWorldPtr worldPtr, double)
{

    // If throttling is enabled, wait here until the step clock is toggled before
    // progressing towards next step
    while (!m_afWorldCommPtr->step_sim()){
        usleep(1);
    }

    m_read_count++;
    if(m_read_count % worldPtr->m_updateCounterLimit == 0){
        m_afWorldCommPtr->update_params_from_server();
        if (m_afWorldCommPtr->m_paramsChanged){
            // Do the stuff

            vector<string> def_topics = m_afWorldCommPtr->get_defunct_topic_names();
            vector<string> new_topics = m_afWorldCommPtr->get_new_topic_names();

            for (int i = 0 ; i < def_topics.size() ; i++){
                string topic_name = def_topics[i];
                if (worldPtr->m_pcMap.find(topic_name) != worldPtr->m_pcMap.end()){
                    // Cleanup
                    afPointCloudPtr afPC = worldPtr->m_pcMap.find(topic_name)->second;
                    worldPtr->m_pcMap.erase(topic_name);
                    delete afPC;
                }
            }

            for (int i = 0 ; i < new_topics.size() ; i++){
                string topic_name = new_topics[i];
                afPointCloudPtr afPC = new afPointCloud(worldPtr);
                afPC->m_topicName = topic_name;
                afPC->loadCommunicationPlugin(afPC, nullptr);
                worldPtr->m_pcMap[topic_name] = afPC;
            }
        }
        m_read_count = 0;
    }

    if (m_afWorldCommPtr->get_reset_flag()){
        std::cerr << "INFO! RESET CALLED FROM WORLD COMM" << std::endl;
        m_worldPtr->setResetFlag();
        m_afWorldCommPtr->clear_reset_flag();
    }

    if (m_afWorldCommPtr->get_reset_bodies_flag()){
        std::cerr << "INFO! RESET BODIES CALLED FROM WORLD COMM" << std::endl;
        m_worldPtr->setResetBodiesFlag();
        m_afWorldCommPtr->clear_reset_bodies_flag();
    }

}

void afWorldCommunicationPlugin::worldUpdateState(afWorldPtr worldPtr, double dt)
{
    m_afWorldCommPtr->set_wall_time(worldPtr->getWallTime());
    m_afWorldCommPtr->set_sim_time(worldPtr->getSimulationTime());
    m_afWorldCommPtr->set_time_stamp(worldPtr->getCurrentTimeStamp());
    m_afWorldCommPtr->set_graphics_loop_freq(worldPtr->getGraphicsFrequency());
    m_afWorldCommPtr->set_physics_loop_freq(worldPtr->getPhysicsFrequency());
    m_afWorldCommPtr->set_num_devices(worldPtr->getNumDevices());
}

#endif
