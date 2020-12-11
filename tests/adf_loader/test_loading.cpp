#include <adf_loader_interface.h>

int main(){

    ADFLoaderInterface loader;

    afLaunchAttributes launchAttribs;

    loader.loadLaunchFileAttribs("/home/adnan/ambf/ambf_models/descriptions/launch.yaml", &launchAttribs);

    afWorldAttributes worldAttribs;

    loader.loadWorldAttribs(launchAttribs.m_worldFilePath.c_str(), &worldAttribs);

    for (int i = 0 ; i < launchAttribs.m_multiBodyFilepaths.size() ; i++){
        afMultiBodyAttributes mbAttribs;

        loader.loadMultiBodyAttribs(launchAttribs.m_multiBodyFilepaths[i].c_str(), &mbAttribs);

        printf("Loaded Multbody File: %i \n", i);
    }

    afAllInputDevicesAttributes allInputDevsAttribs;

    loader.loadAllInputDevicesAttribs(launchAttribs.m_inputDevicesFilepath.c_str(), &allInputDevsAttribs);

    printf("Loaded Launch File \n");

    return 0;
}
