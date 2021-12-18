#include <afFramework.h>

using namespace std;
using namespace ambf;

class afTestSimulatorPlugin: public afSimulatorPlugin{
    virtual int init(int argc, char** argv){return 1;}
    virtual void keyboardUpdate(GLFWwindow* a_window, int a_key, int a_scancode, int a_action, int a_mods){
        cerr << "KEYBOARD INPUT RECEIVED FOR SIMULATION PLUGIN" << a_key << endl;
    }
    virtual void mouseBtnsUpdate(GLFWwindow* a_window, int a_button, int a_action, int a_modes){}
    virtual void mousePosUpdate(GLFWwindow* a_window, double x_pos, double y_pos){}
    virtual void mouseScrollUpdate(GLFWwindow* a_window, double x_pos, double y_pos){}
    virtual void graphicsUpdate(const afWorldPtr a_afWorld){}
    virtual void physicsUpdate(const afWorldPtr a_afWorld){}
    virtual void reset(){}
    virtual bool close(){return 0;}
};


AF_REGISTER_SIMULATOR_PLUGIN(afTestSimulatorPlugin)
