#include <afFramework.h>

using namespace ambf;

class afTestWorldPlugin: public afWorldPlugin{
    virtual int init(const afWorldPtr a_afWorld, const afWorldAttribsPtr a_worldAttribs){return 1;}
    virtual void graphicsUpdate(){}
    virtual void physicsUpdate(const afWorldPtr a_afWorld){
        std::cerr << "UPDATE FROM PLUGIN CALLED" << std::endl;
        std::cerr << "Number of objects " << a_afWorld->getRigidBodyMap()->size() << std::endl;
//        std::cerr << "Number of objects " << a_afWorld->getAFRigidBodies().size() << std::endl;

        for (afBaseObjectMap::iterator i = a_afWorld->getRigidBodyMap()->begin() ; i != a_afWorld->getRigidBodyMap()->end() ; ++i){
            std::cerr << ": " << i->second->getQualifiedName().c_str() << std::endl;
        }
    }
    virtual void reset(){}
    virtual bool close(){return 0;}
};


AF_REGISTER_WORLD_PLUGIN(afTestWorldPlugin)
