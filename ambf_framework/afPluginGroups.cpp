#include <afPluginGroups.h>
#include <afFramework.h>

using namespace std;
using namespace  ambf;

void afSimulatorPluginGroup::init(int argc, char** argv, const afWorldPtr a_afWorld){
    for (vector<afSimulatorPlugin*>::iterator it = m_plugins.begin() ; it != m_plugins.end() ; ++it){
        (*it)->init(argc, argv, a_afWorld);
    }
}

void afSimulatorPluginGroup::keyboardUpdate(int a_key, int a_scancode, int a_action, int a_mods){
    for (vector<afSimulatorPlugin*>::iterator it = m_plugins.begin() ; it != m_plugins.end() ; ++it){
        (*it)->keyboardUpdate(a_key, a_scancode, a_action, a_mods);
    }
}

void afSimulatorPluginGroup::mouseBtnsUpdate(int a_button, int a_action, int a_modes){
    for (vector<afSimulatorPlugin*>::iterator it = m_plugins.begin() ; it != m_plugins.end() ; ++it){
        (*it)->mouseBtnsUpdate(a_button, a_action, a_modes);
    }
}

void afSimulatorPluginGroup::mousePosUpdate(double x_pos, double y_pos){
    for (vector<afSimulatorPlugin*>::iterator it = m_plugins.begin() ; it != m_plugins.end() ; ++it){
        (*it)->mousePosUpdate(x_pos, y_pos);
    }
}

void afSimulatorPluginGroup::mouseScrollUpdate(double x_pos, double y_pos){
    for (vector<afSimulatorPlugin*>::iterator it = m_plugins.begin() ; it != m_plugins.end() ; ++it){
        (*it)->mouseScrollUpdate(x_pos, y_pos);
    }
}

void afSimulatorPluginGroup::graphicsUpdate(){
    for (vector<afSimulatorPlugin*>::iterator it = m_plugins.begin() ; it != m_plugins.end() ; ++it){
        (*it)->graphicsUpdate();
    }
}

void afSimulatorPluginGroup::physicsUpdate(){
    for (vector<afSimulatorPlugin*>::iterator it = m_plugins.begin() ; it != m_plugins.end() ; ++it){
        (*it)->physicsUpdate();
    }
}

void afSimulatorPluginGroup::reset(){
    for (vector<afSimulatorPlugin*>::iterator it = m_plugins.begin() ; it != m_plugins.end() ; ++it){
        (*it)->reset();
    }
}

bool afSimulatorPluginGroup::close(){
    for (vector<afSimulatorPlugin*>::iterator it = m_plugins.begin() ; it != m_plugins.end() ; ++it){
        (*it)->close();
    }
}

void ambf::afWorldPluginGroup::init(const afWorldPtr a_afWorld, const afWorldAttribsPtr a_worldAttribs)
{
    for (vector<afWorldPlugin*>::iterator it = m_plugins.begin() ; it != m_plugins.end() ; ++it){
        (*it)->init(a_afWorld, a_worldAttribs);
    }
}

void afWorldPluginGroup::onModelAdd(const afModelPtr a_modelPtr)
{
    for (vector<afWorldPlugin*>::iterator it = m_plugins.begin() ; it != m_plugins.end() ; ++it){
        (*it)->onModelAdd(a_modelPtr);
    }
}

void afWorldPluginGroup::onModelRemoval(const afModelPtr a_modelPtr)
{
    for (vector<afWorldPlugin*>::iterator it = m_plugins.begin() ; it != m_plugins.end() ; ++it){
        (*it)->onModelRemoval(a_modelPtr);
    }
}

void afWorldPluginGroup::onObjectAdd(const afBaseObjectPtr a_objectPtr)
{
    for (vector<afWorldPlugin*>::iterator it = m_plugins.begin() ; it != m_plugins.end() ; ++it){
        (*it)->onObjectAdd(a_objectPtr);
    }
}

void afWorldPluginGroup::onObjectRemoval(const afBaseObjectPtr a_objectPtr)
{
    for (vector<afWorldPlugin*>::iterator it = m_plugins.begin() ; it != m_plugins.end() ; ++it){
        (*it)->onObjectRemoval(a_objectPtr);
    }
}

void ambf::afWorldPluginGroup::update()
{
    for (vector<afWorldPlugin*>::iterator it = m_plugins.begin() ; it != m_plugins.end() ; ++it){
        (*it)->update();
    }
}

void ambf::afWorldPluginGroup::reset()
{
    for (vector<afWorldPlugin*>::iterator it = m_plugins.begin() ; it != m_plugins.end() ; ++it){
        (*it)->reset();
    }
}

bool ambf::afWorldPluginGroup::close()
{
    for (vector<afWorldPlugin*>::iterator it = m_plugins.begin() ; it != m_plugins.end() ; ++it){
        (*it)->close();
    }
}

void afModelPluginGroup::init(const afModelPtr a_afModel, const afModelAttribsPtr a_modelAttribs)
{
    for (vector<afModelPlugin*>::iterator it = m_plugins.begin() ; it != m_plugins.end() ; ++it){
        (*it)->init(a_afModel, a_modelAttribs);
    }
}

void afModelPluginGroup::onObjectAdd(const afBaseObjectPtr a_objectPtr)
{
    for (vector<afModelPlugin*>::iterator it = m_plugins.begin() ; it != m_plugins.end() ; ++it){
        (*it)->onObjectAdd(a_objectPtr);
    }
}

void afModelPluginGroup::onObjectRemoval(const afBaseObjectPtr a_objectPtr)
{
    for (vector<afModelPlugin*>::iterator it = m_plugins.begin() ; it != m_plugins.end() ; ++it){
        (*it)->onObjectRemoval(a_objectPtr);
    }
}

void afModelPluginGroup::update()
{
    for (vector<afModelPlugin*>::iterator it = m_plugins.begin() ; it != m_plugins.end() ; ++it){
        (*it)->update();
    }
}

void afModelPluginGroup::reset()
{
    for (vector<afModelPlugin*>::iterator it = m_plugins.begin() ; it != m_plugins.end() ; ++it){
        (*it)->reset();
    }
}

bool afModelPluginGroup::close()
{

    for (vector<afModelPlugin*>::iterator it = m_plugins.begin() ; it != m_plugins.end() ; ++it){
        (*it)->close();
    }
}

void afBaseObjectPluginGroup::init(const afBaseObjectPtr a_afObjectPtr, afBaseObjectAttribsPtr a_objectAttribs)
{
    for (vector<afObjectPlugin*>::iterator it = m_plugins.begin() ; it != m_plugins.end() ; ++it){
        (*it)->init(a_afObjectPtr, a_objectAttribs);
    }
}

void afBaseObjectPluginGroup::update()
{
    for (vector<afObjectPlugin*>::iterator it = m_plugins.begin() ; it != m_plugins.end() ; ++it){
        (*it)->update();
    }
}

void afBaseObjectPluginGroup::reset()
{
    for (vector<afObjectPlugin*>::iterator it = m_plugins.begin() ; it != m_plugins.end() ; ++it){
        (*it)->reset();
    }
}

bool afBaseObjectPluginGroup::close()
{
    for (vector<afObjectPlugin*>::iterator it = m_plugins.begin() ; it != m_plugins.end() ; ++it){
        (*it)->close();
    }
}
