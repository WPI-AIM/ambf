#include <afConversions.h>

btVector3 to_btVector(const cVector3d &vec){
    return afUtils::convertDataType<btVector3, cVector3d>(vec);
}

btVector3 to_btVector(const afVector3d &vec){
    return afUtils::convertDataType<btVector3, afVector3d>(vec);
}

btTransform to_btTransform(const cTransform &trans){
    return afUtils::convertDataType<btTransform, cTransform>(trans);
}

btTransform to_btTransform(const afTransform &trans){
    return afUtils::convertDataType<btTransform, afTransform>(trans);
}

cVector3d to_cVector3d(const btVector3 &vec){
    return afUtils::convertDataType<cVector3d, btVector3>(vec);
}

cVector3d to_cVector3d(const afVector3d &vec){
    return afUtils::convertDataType<cVector3d, afVector3d>(vec);
}

cTransform to_cMatrix3d(const btMatrix3x3 &mat){
    return afUtils::convertDataType<cMatrix3d, btMatrix3x3>(mat);
}

cMatrix3d to_cMatrix3d(const afMatrix3d &mat){
    return afUtils::convertDataType<cMatrix3d, afMatrix3d>(mat);
}

cTransform to_cTransform(const btTransform &trans){
    return afUtils::convertDataType<cTransform, btTransform>(trans);
}

cTransform to_cTransform(const afTransform &trans){
    return afUtils::convertDataType<cTransform, afTransform>(trans);
}
