#include <adf_loader_interface.h>

int main(){

    ADFLoaderInterface loader;

    afLaunchAttributes launchAttribs;

    loader.loadLaunchFileAttribs("../../ambf_models/descriptions/launch.yaml", &launchAttribs);

    afWorldAttributes worldAttribs;

    loader.loadWorldAttribs(launchAttribs.m_worldFilepath.c_str(), &worldAttribs);

    for (int i = 0 ; i < launchAttribs.m_modelFilepaths.size() ; i++){
        afModelAttributes mbAttribs;

        loader.loadModelAttribs(launchAttribs.m_modelFilepaths[i].c_str(), &mbAttribs);

        printf("Loaded Multbody File: %i \n", i);
    }

    vector<afTeleRoboticUnitAttributes> tuAttribs;

    // Device indices defined in the input_device.yaml file to load.
    vector<int> dev_indexes = {0,1,2,3,4,5,6,7};

    loader.loadTeleRoboticUnitsAttribs(launchAttribs.m_inputDevicesFilepath.c_str(), &tuAttribs, dev_indexes);

    printf("Loaded Launch File \n");

    return 0;
}
