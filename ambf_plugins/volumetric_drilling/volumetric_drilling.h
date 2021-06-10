#include <afFramework.h>

using namespace std;
using namespace ambf;

class afVolmetricDrillingPlugin: public afSimulatorPlugin{
    virtual void init(int argc, char** argv, const afWorldPtr a_afWorld);
    virtual void keyboardUpdate(GLFWwindow* a_window, int a_key, int a_scancode, int a_action, int a_mods);
    virtual void mouseBtnsUpdate(GLFWwindow* a_window, int a_button, int a_action, int a_modes);
    virtual void mousePosUpdate(GLFWwindow* a_window, double x_pos, double y_pos){}
    virtual void mouseScrollUpdate(GLFWwindow* a_window, double x_pos, double y_pos);
    virtual void graphicsUpdate(const afWorldPtr a_afWorld);
    virtual void physicsUpdate(const afWorldPtr a_afWorld);
    virtual void reset();
    virtual bool close();
};


AF_REGISTER_SIMULATOR_PLUGIN(afVolmetricDrillingPlugin)
