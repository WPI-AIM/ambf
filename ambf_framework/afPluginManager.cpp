#include <afPluginManager.h>
#include <afFramework.h>

using namespace std;
using namespace  ambf;

void afSimulatorPluginManager::init(int argc, char** argv, const afWorldPtr a_afWorld){
    for (vector<afSimulatorPlugin*>::iterator it = m_plugins.begin() ; it != m_plugins.end() ; ++it){
        (*it)->init(argc, argv, a_afWorld);
    }
}

void afSimulatorPluginManager::keyboardUpdate(GLFWwindow* a_window, int a_key, int a_scancode, int a_action, int a_mods){
    for (vector<afSimulatorPlugin*>::iterator it = m_plugins.begin() ; it != m_plugins.end() ; ++it){
        (*it)->keyboardUpdate(a_window, a_key, a_scancode, a_action, a_mods);
    }
}

void afSimulatorPluginManager::mouseBtnsUpdate(GLFWwindow* a_window, int a_button, int a_action, int a_modes){
    for (vector<afSimulatorPlugin*>::iterator it = m_plugins.begin() ; it != m_plugins.end() ; ++it){
        (*it)->mouseBtnsUpdate(a_window, a_button, a_action, a_modes);
    }
}

void afSimulatorPluginManager::mousePosUpdate(GLFWwindow* a_window, double x_pos, double y_pos){
    for (vector<afSimulatorPlugin*>::iterator it = m_plugins.begin() ; it != m_plugins.end() ; ++it){
        (*it)->mousePosUpdate(a_window, x_pos, y_pos);
    }
}

void afSimulatorPluginManager::mouseScrollUpdate(GLFWwindow* a_window, double x_pos, double y_pos){
    for (vector<afSimulatorPlugin*>::iterator it = m_plugins.begin() ; it != m_plugins.end() ; ++it){
        (*it)->mouseScrollUpdate(a_window, x_pos, y_pos);
    }
}

void afSimulatorPluginManager::graphicsUpdate(){
    for (vector<afSimulatorPlugin*>::iterator it = m_plugins.begin() ; it != m_plugins.end() ; ++it){
        (*it)->graphicsUpdate();
    }
}

void afSimulatorPluginManager::physicsUpdate(double dt){
    for (vector<afSimulatorPlugin*>::iterator it = m_plugins.begin() ; it != m_plugins.end() ; ++it){
        (*it)->physicsUpdate(dt);
    }
}

void afSimulatorPluginManager::reset(){
    for (vector<afSimulatorPlugin*>::iterator it = m_plugins.begin() ; it != m_plugins.end() ; ++it){
        (*it)->reset();
    }
}

bool afSimulatorPluginManager::close(){
    for (vector<afSimulatorPlugin*>::iterator it = m_plugins.begin() ; it != m_plugins.end() ; ++it){
        (*it)->close();
    }
}

void ambf::afWorldPluginManager::init(const afWorldPtr a_afWorld, const afWorldAttribsPtr a_worldAttribs)
{
    for (vector<afWorldPlugin*>::iterator it = m_plugins.begin() ; it != m_plugins.end() ; ++it){
        (*it)->init(a_afWorld, a_worldAttribs);
    }
}

void afWorldPluginManager::onModelAdd(const afModelPtr a_modelPtr)
{
    for (vector<afWorldPlugin*>::iterator it = m_plugins.begin() ; it != m_plugins.end() ; ++it){
        (*it)->onModelAdd(a_modelPtr);
    }
}

void afWorldPluginManager::onModelRemoval(const afModelPtr a_modelPtr)
{
    for (vector<afWorldPlugin*>::iterator it = m_plugins.begin() ; it != m_plugins.end() ; ++it){
        (*it)->onModelRemoval(a_modelPtr);
    }
}

void afWorldPluginManager::onObjectAdd(const afBaseObjectPtr a_objectPtr)
{
    for (vector<afWorldPlugin*>::iterator it = m_plugins.begin() ; it != m_plugins.end() ; ++it){
        (*it)->onObjectAdd(a_objectPtr);
    }
}

void afWorldPluginManager::onObjectRemoval(const afBaseObjectPtr a_objectPtr)
{
    for (vector<afWorldPlugin*>::iterator it = m_plugins.begin() ; it != m_plugins.end() ; ++it){
        (*it)->onObjectRemoval(a_objectPtr);
    }
}

void ambf::afWorldPluginManager::physicsUpdate(double dt)
{
    for (vector<afWorldPlugin*>::iterator it = m_plugins.begin() ; it != m_plugins.end() ; ++it){
        (*it)->physicsUpdate(dt);
    }
}

void ambf::afWorldPluginManager::reset()
{
    for (vector<afWorldPlugin*>::iterator it = m_plugins.begin() ; it != m_plugins.end() ; ++it){
        (*it)->reset();
    }
}

bool ambf::afWorldPluginManager::close()
{
    for (vector<afWorldPlugin*>::iterator it = m_plugins.begin() ; it != m_plugins.end() ; ++it){
        (*it)->close();
    }
}

void afModelPluginManager::init(const afModelPtr a_afModel, const afModelAttribsPtr a_modelAttribs)
{
    for (vector<afModelPlugin*>::iterator it = m_plugins.begin() ; it != m_plugins.end() ; ++it){
        (*it)->init(a_afModel, a_modelAttribs);
    }
}

void afModelPluginManager::onObjectAdd(const afBaseObjectPtr a_objectPtr)
{
    for (vector<afModelPlugin*>::iterator it = m_plugins.begin() ; it != m_plugins.end() ; ++it){
        (*it)->onObjectAdd(a_objectPtr);
    }
}

void afModelPluginManager::onObjectRemoval(const afBaseObjectPtr a_objectPtr)
{
    for (vector<afModelPlugin*>::iterator it = m_plugins.begin() ; it != m_plugins.end() ; ++it){
        (*it)->onObjectRemoval(a_objectPtr);
    }
}

void afModelPluginManager::physicsUpdate(double dt)
{
    for (vector<afModelPlugin*>::iterator it = m_plugins.begin() ; it != m_plugins.end() ; ++it){
        (*it)->physicsUpdate(dt);
    }
}

void afModelPluginManager::reset()
{
    for (vector<afModelPlugin*>::iterator it = m_plugins.begin() ; it != m_plugins.end() ; ++it){
        (*it)->reset();
    }
}

bool afModelPluginManager::close()
{

    for (vector<afModelPlugin*>::iterator it = m_plugins.begin() ; it != m_plugins.end() ; ++it){
        (*it)->close();
    }
}

void afBaseObjectPluginManager::init(const afBaseObjectPtr a_afObjectPtr, afBaseObjectAttribsPtr a_objectAttribs)
{
    for (vector<afObjectPlugin*>::iterator it = m_plugins.begin() ; it != m_plugins.end() ; ++it){
        (*it)->init(a_afObjectPtr, a_objectAttribs);
    }
}

void afBaseObjectPluginManager::physicsUpdate(double dt)
{
    for (vector<afObjectPlugin*>::iterator it = m_plugins.begin() ; it != m_plugins.end() ; ++it){
        (*it)->physicsUpdate(dt);
    }
}

void afBaseObjectPluginManager::reset()
{
    for (vector<afObjectPlugin*>::iterator it = m_plugins.begin() ; it != m_plugins.end() ; ++it){
        (*it)->reset();
    }
}

bool afBaseObjectPluginManager::close()
{
    for (vector<afObjectPlugin*>::iterator it = m_plugins.begin() ; it != m_plugins.end() ; ++it){
        (*it)->close();
    }
}
