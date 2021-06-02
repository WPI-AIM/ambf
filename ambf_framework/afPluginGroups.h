
#ifndef AF_PLUGIN_GROUPS_H
#define AF_PLUGIN_GROUPS_H

#include <afPluginInterface.h>

using namespace std;

namespace ambf {


// Forward declaration
class afWorld;
class afWorldAttributes;

class afModel;
class afModelAttributes;

class afBaseObject;
class afBaseObjectAttributes;

typedef afWorld* afWorldPtr;
typedef afWorldAttributes* afWorldAttribsPtr;

typedef afModel* afModelPtr;
typedef afModelAttributes* afModelAttribsPtr;

typedef afBaseObject* afBaseObjectPtr;
typedef afBaseObjectAttributes* afBaseObjectAttribsPtr;


template <class T>
class afBasePluginGroup{
public:
    ~afBasePluginGroup(){
        for (typename vector<T*>::iterator it = m_plugins.begin() ; it != m_plugins.end() ; ++it){
            delete (*it);
        }
    }

    virtual bool add(string lib_name, string plugin_name){
        T* plugin = T::Create(lib_name, plugin_name);
        if (plugin){
            m_plugins.push_back(plugin);
            return true;
        }
        else{
            return false;
        }
    }

    vector<T*>* getPlugins(){
        return &m_plugins;
    }


protected:
    vector<T*> m_plugins;
};


class afWorldPluginGroup: public afBasePluginGroup<afWorldPlugin>{
public:
    void init(const afWorldPtr a_afWorld, const afWorldAttribsPtr a_worldAttribs);

    void onModelAdd(const afModelPtr a_modelPtr);

    void onModelRemoval(const afModelPtr a_modelPtr);

    void onObjectAdd(const afBaseObjectPtr a_objectPtr);

    void onObjectRemoval(const afBaseObjectPtr a_objectPtr);

    void update();

    void reset();

    bool close();

};


class afModelPluginGroup: public afBasePluginGroup<afModelPlugin>{
public:
    void init(const afModelPtr a_afModel, const afModelAttribsPtr a_modelAttribs);

    void onObjectAdd(const afBaseObjectPtr a_objectPtr);

    void onObjectRemoval(const afBaseObjectPtr a_objectPtr);

    void update();

    void reset();

    bool close();

};


class afBaseObjectPluginGroup: public afBasePluginGroup<afObjectPlugin>{
public:
    void init(const afBaseObjectPtr a_afObjectPtr, afBaseObjectAttribsPtr a_objectAttribs);

    void update();

    void reset();

    bool close();
};


class afSimulatorPluginGroup: public afBasePluginGroup<afSimulatorPlugin>{
public:

    void init(int argc, char** argv, const afWorldPtr a_afWorld);

    void keyboardUpdate(int a_key, int a_scancode, int a_action, int a_mods);

    void mouseBtnsUpdate(int a_button, int a_action, int a_modes);

    void mousePosUpdate(double x_pos, double y_pos);

    void mouseScrollUpdate(double x_pos, double y_pos);

    void graphicsUpdate();

    void physicsUpdate();

    void reset();

    bool close();
};

}


#endif // AF_PLUGIN_CALLBACK_HELPERS_H
