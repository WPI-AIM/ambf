#include "volumetric_drilling.h"

cVoxelObject* g_volObject;
cImagePtr g_colorLUT;

string g_basePath;

int afVolmetricDrillingPlugin::init(int argc, char **argv, const afWorldPtr a_afWorld){

    double maxStiffness = 1.0;
    double g_opticalDensity = 1.2;

    // create a volumetric model
    g_volObject = new cVoxelObject();

    // add object to world
    a_afWorld->addSceneObjectToWorld(g_volObject);

    // position object
    g_volObject->setLocalPos(0.0, 0.0, 0.0);

    // rotate object
    g_volObject->rotateExtrinsicEulerAnglesDeg(0, 0, 65, C_EULER_ORDER_XYZ);

    // set the dimensions by assigning the position of the min and max corners
    g_volObject->m_minCorner.set(-0.5,-0.5,-0.5);
    g_volObject->m_maxCorner.set( 0.5, 0.5, 0.5);

    // set the texture coordinate at each corner.
    g_volObject->m_minTextureCoord.set(0.0, 0.0, 0.0);
    g_volObject->m_maxTextureCoord.set(1.0, 1.0, 1.0);

    // set quality of graphic rendering
    g_volObject->setQuality(0.5);

    // create multi image
    cMultiImagePtr image = cMultiImage::create();

    g_basePath = "../../ambf_plugins/volumetric_drilling";

    int filesloaded = image->loadFromFiles(g_basePath + "/resources/volumes/ear3/plane0", "png", 512);
    if (filesloaded == 0) {
#if defined(_MSVC)
        filesloaded = image->loadFromFiles("../../../drilling_simulator/resources/volumes/ear3/plane0", "png", 512);
#endif
    }
    if (filesloaded == 0) {
        cout << "Error - Failed to load volume data planeXXXX.png." << endl;
        close();
        return -1;
    }

    // create texture
    cTexture3dPtr texture = cTexture3d::create();

    // assign volumetric image to texture
    texture->setImage(image);

    // assign texture to voxel object
    g_volObject->setTexture(texture);

    // create texture
    texture = cTexture3d::create();

    // assign volumetric image to texture
    texture->setImage(image);

    // assign texture to voxel object
    g_volObject->setTexture(texture);

    // initially select an isosurface corresponding to the bone/heart level
    g_volObject->setIsosurfaceValue(0.45);

    // set optical density factor
    g_volObject->setOpticalDensity(g_opticalDensity);


    //--------------------------------------------------------------------------
    // LOAD COLORMAPS
    //--------------------------------------------------------------------------

    g_colorLUT = cImage::create();
    bool fileLoaded = g_colorLUT->loadFromFile(g_basePath + "/resources/volumes/colormap_bone.png");
    if (!fileLoaded) {
#if defined(_MSVC)
        fileLoaded = boneLUT->loadFromFile("../../../drilling_simulator/resources/volumes/heart/colormap_bone.png");
#endif
    }
    if (!fileLoaded)
    {
        cout << "Error - Failed to load colormap." << endl;
        close();
        return -1;
    }

    // tell the voxel object to load the colour look-up table as a texture
    g_volObject->m_colorMap->setImage(g_colorLUT);

    // set graphic rendering mode
    //    object->setRenderingModeIsosurfaceColorMap();
    //    object->setRenderingModeDVRColorMap();
    //    object->setRenderingModeVoxelColors();
    //    object->setRenderingModeBasic();
    g_volObject->setRenderingModeIsosurfaceColors();
    //    object->setRenderingModeIsosurfaceMaterial();
    return 1;
}

void afVolmetricDrillingPlugin::keyboardUpdate(GLFWwindow *a_window, int a_key, int a_scancode, int a_action, int a_mods){

}

void afVolmetricDrillingPlugin::mouseBtnsUpdate(GLFWwindow *a_window, int a_button, int a_action, int a_modes){

}

void afVolmetricDrillingPlugin::mouseScrollUpdate(GLFWwindow *a_window, double x_pos, double y_pos){

}

void afVolmetricDrillingPlugin::graphicsUpdate(){

}

void afVolmetricDrillingPlugin::physicsUpdate(double dt){

}

void afVolmetricDrillingPlugin::reset(){

}

bool afVolmetricDrillingPlugin::close()
{
    return 0;
}
