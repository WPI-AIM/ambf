#include <afFramework.h>

using namespace std;
using namespace ambf;

class afTestSimulatorPlugin: public afSimulatorPlugin{
    virtual void init(int argc, char** argv){}
    virtual void keyboardUpdate(int a_key, int a_scancode, int a_action, int a_mods){
        cerr << "KEYBOARD INPUT RECEIVED FOR SIMULATION PLUGIN" << a_key << endl;
    }
    virtual void mouseBtnsUpdate(int a_button, int a_action, int a_modes){}
    virtual void mousePosUpdate(double x_pos, double y_pos){}
    virtual void mouseScrollUpdate(double x_pos, double y_pos){}
    virtual void graphicsUpdate(const afWorldPtr a_afWorld){}
    virtual void physicsUpdate(const afWorldPtr a_afWorld){}
    virtual void reset(){}
    virtual bool close(){}
};


AF_REGISTER_SIMULATOR_PLUGIN(afTestSimulatorPlugin)
