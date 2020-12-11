#include <adf_loader_interface.h>

int main(){

    ADFLoaderInterface loader;

    afLaunchAttributes launchAttribs;

    loader.loadLaunchFileAttribs("/home/adnan/ambf/ambf_models/descriptions/launch.yaml", &launchAttribs);

    afWorldAttributes worldAttribs;

    afPath worldFilePath = launchAttribs.m_path / launchAttribs.m_worldFilePath;

    loader.loadWorldAttribs(worldFilePath.c_str(), &worldAttribs);

    for (int i = 0 ; i < launchAttribs.m_multiBodyFilepaths.size() ; i++){
        afMultiBodyAttributes mbAttribs;

        afPath mbFilePath = launchAttribs.m_path / launchAttribs.m_multiBodyFilepaths[0];

        loader.loadMultiBodyAttribs(mbFilePath.c_str(), &mbAttribs);

        printf("Loaded Multbody File: %i \n", i);
    }

    afAllInputDevicesAttributes allInputDevsAttribs;

    afPath inputDevFilePath = launchAttribs.m_path / launchAttribs.m_inputDevicesFilepath;

    loader.loadAllInputDevicesAttribs(inputDevFilePath.c_str(), &allInputDevsAttribs);

    printf("Loaded Launch File \n");

    return 0;
}
