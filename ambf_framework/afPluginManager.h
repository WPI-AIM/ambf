
#ifndef AF_PLUGIN_MANAGER_H
#define AF_PLUGIN_MANAGER_H

#include <afPluginInterface.h>

using namespace std;

namespace ambf {


// Forward declaration
class afWorld;
class afModel;
class afBaseObject;


typedef afWorld* afWorldPtr;
typedef afModel* afModelPtr;
typedef afBaseObject* afBaseObjectPtr;

class afWorldAttributes;
class afModelAttributes;
class afBaseObjectAttributes;

typedef afWorldAttributes* afWorldAttribsPtr;
typedef afModelAttributes* afModelAttribsPtr;
typedef afBaseObjectAttributes* afBaseObjectAttribsPtr;


template <class T>
class afBasePluginManager{
public:
    ~afBasePluginManager(){
        for (typename vector<T*>::iterator it = m_plugins.begin() ; it != m_plugins.end() ; ++it){
            delete (*it);
        }
    }

    virtual bool add(string lib_name, string plugin_name){
        T* plugin = T::Create(lib_name, plugin_name);
        return add(plugin);
    }

    virtual bool add(T* plugin){
        if (plugin){
            m_plugins.push_back(plugin);
            return true;
        }
        else{
            cerr << "FAILED TO LOAD PLUGIN" << endl;
            return false;
        }
    }

    vector<T*>* getPlugins(){
        return &m_plugins;
    }


protected:
    vector<T*> m_plugins;
};

class afSimulatorPluginManager: public afBasePluginManager<afSimulatorPlugin>{
public:

    void init(int argc, char** argv, const afWorldPtr a_afWorld);

    void keyboardUpdate(GLFWwindow* a_window, int a_key, int a_scancode, int a_action, int a_mods);

    void mouseBtnsUpdate(GLFWwindow* a_window, int a_button, int a_action, int a_modes);

    void mousePosUpdate(GLFWwindow* a_window, double x_pos, double y_pos);

    void mouseScrollUpdate(GLFWwindow* a_window, double x_pos, double y_pos);

    void graphicsUpdate();

    void physicsUpdate(double dt);

    void reset();

    bool close();
};


class afWorldPluginManager: public afBasePluginManager<afWorldPlugin>{
public:
    void init(const afWorldPtr a_afWorld, const afWorldAttribsPtr a_worldAttribs);

    void onModelAdd(const afModelPtr a_modelPtr);

    void onModelRemoval(const afModelPtr a_modelPtr);

    void onObjectAdd(const afBaseObjectPtr a_objectPtr);

    void onObjectRemoval(const afBaseObjectPtr a_objectPtr);

    void graphicsUpdate();

    void physicsUpdate(double dt);

    void reset();

    bool close();

};


class afModelPluginManager: public afBasePluginManager<afModelPlugin>{
public:
    void init(const afModelPtr a_afModel, const afModelAttribsPtr a_modelAttribs);

    void onObjectAdd(const afBaseObjectPtr a_objectPtr);

    void onObjectRemoval(const afBaseObjectPtr a_objectPtr);

    void graphicsUpdate();

    void physicsUpdate(double dt);

    void reset();

    bool close();

};


class afBaseObjectPluginManager: public afBasePluginManager<afObjectPlugin>{
public:
    void init(const afBaseObjectPtr a_afObjectPtr, afBaseObjectAttribsPtr a_objectAttribs);

    void graphicsUpdate();

    void physicsUpdate(double dt);

    void reset();

    bool close();
};

}


#endif // AF_PLUGIN_CALLBACK_HELPERS_H
