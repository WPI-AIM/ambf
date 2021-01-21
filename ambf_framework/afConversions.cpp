#include <afConversions.h>

template<>
///
/// \brief afConversions::convertDataTypes<afVector3d, btVector3>
/// \param p
/// \return
///
afVector3d afConversions::convertDataType<afVector3d, btVector3>(const btVector3 &p){
    afVector3d aPos(p.x(), p.y(), p.z());
    return aPos;
}

template<>
///
/// \brief afConversions::convertDataTypes<afVector3d, cVector3d>
/// \param p
/// \return
///
afVector3d afConversions::convertDataType<afVector3d, cVector3d>(const cVector3d &p){
    afVector3d aPos(p.x(), p.y(), p.z());
    return aPos;
}

template<>
///
/// \brief afConversions::convertDataTypes<afMatrix3d, btMatrix3x3>
/// \param r
/// \return
///
afMatrix3d afConversions::convertDataType<afMatrix3d, btMatrix3x3>(const btMatrix3x3 &r){
    afMatrix3d aMat;
    double ro,pi,ya;
    r.getEulerYPR(ya, pi, ro);
    aMat.setRPY(ro, pi, ya);
    return aMat;
}


template<>
///
/// \brief afConversions::convertDataTypes<btMatrix3x3, cMatrix3d>
/// \param r
/// \return
///
afMatrix3d afConversions::convertDataType<afMatrix3d, cMatrix3d>(const cMatrix3d &r){
    afMatrix3d aMat;
    aMat(0,0) = r(0,0); aMat(0,1) = r(0,1); aMat(0,2) = r(0,2);
    aMat(1,0) = r(1,0); aMat(1,1) = r(1,1); aMat(0,2) = r(1,2);
    aMat(2,0) = r(2,0); aMat(2,1) = r(2,1); aMat(0,2) = r(2,2);
    return aMat;
}

template<>
///
/// \brief afConversions::convertDataTypes<afTransform, btTransform>
/// \param t
/// \return
///
afTransform afConversions::convertDataType<afTransform, btTransform>(const btTransform &t){
    afMatrix3d aRot = afConversions::convertDataType<afMatrix3d, btMatrix3x3>(t.getBasis());
    afVector3d aPos = afConversions::convertDataType<afVector3d, btVector3>(t.getOrigin());
    afTransform aMat(aRot, aPos);
    return aMat;
}


template<>
///
/// \brief afConversions::convertDataTypes<afTransform, cTransform>
/// \param t
/// \return
///
afTransform afConversions::convertDataType<afTransform, cTransform>(const cTransform &t){
    afMatrix3d aRot = afConversions::convertDataType<afMatrix3d, cMatrix3d>(t.getLocalRot());
    afVector3d aPos = afConversions::convertDataType<afVector3d, cVector3d>(t.getLocalPos());
    afTransform aMat(aRot, aPos);
    return aMat;
}

template<>
///
/// \brief afConversions::convertDataTypes<btVector3, cVector3d>
/// \param p
/// \return
///
btVector3 afConversions::convertDataType<btVector3, afVector3d>(const afVector3d &p){
    btVector3 btPos(p(0), p(1), p(2));
    return btPos;
}

template<>
///
/// \brief afConversions::convertDataTypes<btVector3, cVector3d>
/// \param p
/// \return
///
btVector3 afConversions::convertDataType<btVector3, cVector3d>(const cVector3d &p){
    btVector3 btPos(p.x(), p.y(), p.z());
    return btPos;
}

template<>
///
/// \brief afConversions::convertDataTypes<cQuaternion, btQuaternion>
/// \param q
/// \return
///
btQuaternion afConversions::convertDataType<btQuaternion, cQuaternion>(const cQuaternion &q){
    btQuaternion btQuat(q.x, q.y, q.z, q.w);
    return btQuat;
}


template<>
///
/// \brief afConversions::convertDataTypes<btMatrix3x3, cMatrix3d>
/// \param r
/// \return
///
btMatrix3x3 afConversions::convertDataType<btMatrix3x3, afMatrix3d>(const afMatrix3d &mat){
    btMatrix3x3 btMat;
    double r, p, y;
    mat.getRPY(r, p, y);
    btMat.setEulerYPR(y, p, r);
    return btMat;
}

template<>
///
/// \brief afConversions::convertDataTypes<btMatrix3x3, cMatrix3d>
/// \param r
/// \return
///
btMatrix3x3 afConversions::convertDataType<btMatrix3x3, cMatrix3d>(const cMatrix3d &r){
    cQuaternion cQuat;
    cQuat.fromRotMat(r);

    btQuaternion btQuat(cQuat.x, cQuat.y, cQuat.z, cQuat.w);
    btMatrix3x3 btMat;
    btMat.setRotation(btQuat);

    return btMat;
}


template<>
///
/// \brief afConversions::convertDataTypes<btTransform, cTransform>
/// \param t
/// \return
///
btTransform afConversions::convertDataType<btTransform, afTransform>(const afTransform &t){
    btMatrix3x3 btRot = afConversions::convertDataType<btMatrix3x3, afMatrix3d>(t.getRotation());
    btVector3 btPos = afConversions::convertDataType<btVector3, afVector3d>(t.getPosition());
    btTransform btMat(btRot, btPos);
    return btMat;
}


template<>
///
/// \brief afConversions::convertDataTypes<btTransform, cTransform>
/// \param t
/// \return
///
btTransform afConversions::convertDataType<btTransform, cTransform>(const cTransform &t){
    btMatrix3x3 btRot = afConversions::convertDataType<btMatrix3x3, cMatrix3d>(t.getLocalRot());
    btVector3 btPos = afConversions::convertDataType<btVector3, cVector3d>(t.getLocalPos());
    btTransform btMat(btRot, btPos);
    return btMat;
}


template<>
///
/// \brief afConversions::convertDataTypes<cVector3d, afVector3d>
/// \param p
/// \return
///
cVector3d afConversions::convertDataType<cVector3d, afVector3d>(const afVector3d &p){
    cVector3d cPos(p(0), p(1), p(2));
    return cPos;
}


template<>
///
/// \brief afConversions::convertDataTypes<cVector3d, btVector3>
/// \param p
/// \return
///
cVector3d afConversions::convertDataType<cVector3d, btVector3>(const btVector3 &p){
    cVector3d cPos(p.x(), p.y(), p.z());
    return cPos;
}


template<>
///
/// \brief afConversions::convertDataTypes<cQuaternion, btQuaternion>
/// \param q
/// \return
///
cQuaternion afConversions::convertDataType<cQuaternion, btQuaternion>(const btQuaternion &q){
    cQuaternion cQuat(q.w(), q.x(), q.y(), q.z());
    return cQuat;
}


template<>
///
/// \brief afConversions::convertDataTypes<cMatrix3d, afMatrix3d>
/// \param r
/// \return
///
cMatrix3d afConversions::convertDataType<cMatrix3d, afMatrix3d>(const afMatrix3d &r){
    cMatrix3d cMat;
    cMat.set(r(0,0), r(0,1), r(0,2),
             r(1,0), r(1,1), r(1,2),
             r(2,0), r(2,1), r(2,2));
    return cMat;
}


template<>
///
/// \brief afConversions::convertDataTypes<cMatrix3d, btMatrix3x3>
/// \param r
/// \return
///
cMatrix3d afConversions::convertDataType<cMatrix3d, btMatrix3x3>(const btMatrix3x3 &r){
    btQuaternion btQuat;
    r.getRotation(btQuat);

    cQuaternion cQuat(btQuat.w(), btQuat.x(), btQuat.y(), btQuat.z());

    cMatrix3d cMat;
    cQuat.toRotMat(cMat);

    return cMat;
}


template<>
///
/// \brief afConversions::convertDataTypes<cTransform, afTransform>
/// \param t
/// \return
///
cTransform afConversions::convertDataType<cTransform, afTransform>(const afTransform &t){
    cMatrix3d cMat = afConversions::convertDataType<cMatrix3d, afMatrix3d>(t.getRotation());
    cVector3d cPos = afConversions::convertDataType<cVector3d, afVector3d>(t.getPosition());
    cTransform cTrans(cPos, cMat);
    return cTrans;
}


template<>
///
/// \brief afConversions::convertDataTypes<cTransform, btTransform>
/// \param t
/// \return
///
cTransform afConversions::convertDataType<cTransform, btTransform>(const btTransform &t){
    cMatrix3d cRot = afConversions::convertDataType<cMatrix3d, btMatrix3x3>(t.getBasis());
    cVector3d cPos = afConversions::convertDataType<cVector3d, btVector3>(t.getOrigin());
    cTransform cMat(cPos, cRot);
    return cMat;
}

///////


btVector3 to_btVector(const cVector3d &vec){
    return afConversions::convertDataType<btVector3, cVector3d>(vec);
}

btVector3 to_btVector(const afVector3d &vec){
    return afConversions::convertDataType<btVector3, afVector3d>(vec);
}

btTransform to_btTransform(const cTransform &trans){
    return afConversions::convertDataType<btTransform, cTransform>(trans);
}

btTransform to_btTransform(const afTransform &trans){
    return afConversions::convertDataType<btTransform, afTransform>(trans);
}

cVector3d to_cVector3d(const btVector3 &vec){
    return afConversions::convertDataType<cVector3d, btVector3>(vec);
}

cVector3d to_cVector3d(const afVector3d &vec){
    return afConversions::convertDataType<cVector3d, afVector3d>(vec);
}

cTransform to_cMatrix3d(const btMatrix3x3 &mat){
    return afConversions::convertDataType<cMatrix3d, btMatrix3x3>(mat);
}

cMatrix3d to_cMatrix3d(const afMatrix3d &mat){
    return afConversions::convertDataType<cMatrix3d, afMatrix3d>(mat);
}

cTransform to_cTransform(const btTransform &trans){
    return afConversions::convertDataType<cTransform, btTransform>(trans);
}

cTransform to_cTransform(const afTransform &trans){
    return afConversions::convertDataType<cTransform, afTransform>(trans);
}

afVector3d &operator<<(afVector3d &lhs, const btVector3 &rhs){
    lhs = afConversions::convertDataType<afVector3d, btVector3>(rhs);
    return lhs;
}

afVector3d &operator<<(afVector3d &lhs, const cVector3d &rhs){
    lhs = afConversions::convertDataType<afVector3d, cVector3d>(rhs);
    return lhs;
}

afVector3d &operator*(cTransform &lhs, afVector3d &rhs){
    cVector3d tV = afConversions::convertDataType<cVector3d, afVector3d>(rhs);
    rhs << lhs * tV;
    return rhs;
}

afVector3d &operator*(btTransform &lhs, afVector3d &rhs){
    btVector3 tV = afConversions::convertDataType<btVector3, afVector3d>(rhs);
    rhs << lhs * tV;
    return rhs;
}

afTransform &operator<<(afTransform &lhs, const btTransform &rhs){
    lhs = afConversions::convertDataType<afTransform, btTransform>(rhs);
    return lhs;
}

afTransform &operator<<(afTransform &lhs, const cTransform &rhs){
    lhs = afConversions::convertDataType<afTransform, cTransform>(rhs);
    return lhs;
}

btVector3 &operator<<(btVector3 &lhs, const afVector3d &rhs){
    lhs = afConversions::convertDataType<btVector3, afVector3d>(rhs);
    return lhs;
}

btVector3 &operator<<(btVector3 &lhs, const cVector3d &rhs){
    lhs = afConversions::convertDataType<btVector3, cVector3d>(rhs);
    return lhs;
}

btVector3 &operator*(cTransform &lhs, btVector3 &rhs){
    cVector3d tV = afConversions::convertDataType<cVector3d, btVector3>(rhs);
    rhs << lhs * tV;
    return rhs;
}

btTransform &operator<<(btTransform &lhs, const afTransform &rhs){
    lhs = afConversions::convertDataType<btTransform, afTransform>(rhs);
    return lhs;
}

btTransform &operator<<(btTransform &lhs, const cTransform &rhs){
    lhs = afConversions::convertDataType<btTransform, cTransform>(rhs);
    return lhs;
}

cVector3d &operator<<(cVector3d &lhs, const afVector3d &rhs){
    lhs = afConversions::convertDataType<cVector3d, afVector3d>(rhs);
    return lhs;
}

cVector3d &operator<<(cVector3d &lhs, const btVector3 &rhs){
    lhs = afConversions::convertDataType<cVector3d, btVector3>(rhs);
    return lhs;
}

cVector3d &operator*(btTransform &lhs, cVector3d &rhs){
    btVector3 tV = afConversions::convertDataType<btVector3, cVector3d>(rhs);
    rhs << lhs * tV;
    return rhs;
}

cMatrix3d &operator<<(cMatrix3d &lhs, const afMatrix3d &rhs){
    lhs = afConversions::convertDataType<cMatrix3d, afMatrix3d>(rhs);
    return lhs;
}

cMatrix3d &operator<<(cMatrix3d &lhs, const btMatrix3x3 &rhs){
    lhs = afConversions::convertDataType<cMatrix3d, btMatrix3x3>(rhs);
    return lhs;
}

cTransform &operator<<(cTransform &lhs, const afTransform &rhs){
    lhs = afConversions::convertDataType<cTransform, afTransform>(rhs);
    return lhs;
}

cTransform &operator<<(cTransform &lhs, const btTransform &rhs){
    lhs = afConversions::convertDataType<cTransform, btTransform>(rhs);
    return lhs;
}
