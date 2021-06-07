//==============================================================================
/*
    Software License Agreement (BSD License)
    Copyright (c) 2019-2021, AMBF
    (https://github.com/WPI-AIM/ambf)

    All rights reserved.

    Redistribution and use in source and binary forms, with or without
    modification, are permitted provided that the following conditions
    are met:

    * Redistributions of source code must retain the above copyright
    notice, this list of conditions and the following disclaimer.

    * Redistributions in binary form must reproduce the above
    copyright notice, this list of conditions and the following
    disclaimer in the documentation and/or other materials provided
    with the distribution.

    * Neither the name of authors nor the names of its contributors may
    be used to endorse or promote products derived from this software
    without specific prior written permission.

    THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
    "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
    LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
    FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
    COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
    INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
    BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
    LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
    CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
    LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
    ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
    POSSIBILITY OF SUCH DAMAGE.

    \author    Adnan Munawar
    \author    <amunawar@wpi.edu>
*/
//==============================================================================

//------------------------------------------------------------------------------
#ifndef AF_MATH_H
#define AF_MATH_H
//------------------------------------------------------------------------------

#include <math.h>
#include <assert.h>
#include <iostream>

using namespace std;
typedef unsigned int uint;

#define PI 3.141592653589793238
#define PI_2 1.5707963267948966

namespace ambf {

class afVector3d{
public:
    afVector3d(){
        set(0, 0, 0);
    }
    afVector3d(double x, double y, double z){
        set(x, y, z);
    }

    void set(double x, double y, double z){
        m_data[0] = x;
        m_data[1] = y;
        m_data[2] = z;
    }

    void getXYZ(double &x, double &y, double &z){
        x = m_data[0];
        y = m_data[1];
        z = m_data[2];
    }

    const double& operator() (const int index) const{
        assert(index < 3);
        return m_data[index];
    }

    double& operator() (const int index){
        assert(index < 3);
        return m_data[index];
    }

    void operator *=(double a_scale){
        (*this)(0) *= a_scale;
        (*this)(1) *= a_scale;
        (*this)(2) *= a_scale;
    }

    void operator +=(afVector3d vIn){
        (*this)(0) += vIn(0);
        (*this)(1) += vIn(1);
        (*this)(2) += vIn(2);
    }

    void operator -=(afVector3d vIn){
        (*this)(0) -= vIn(0);
        (*this)(1) -= vIn(1);
        (*this)(2) -= vIn(2);
    }

    void normalize(){
        double magInv = 1.0 / getNorm();
        (*this)(0) *= magInv;
        (*this)(1) *= magInv;
        (*this)(2) *= magInv;
    }

    double getNorm(){
        return (m_data[0] * m_data[0] + m_data[1] * m_data[1] + m_data[2] * m_data[2]);
    }

    void print(){
        cerr << "[ " << m_data[0] << ", " << m_data[1] << ", " << m_data[2] << "]\n" ;
    }

private:
    double m_data[3];
};

inline afVector3d operator +(const afVector3d& v1, const afVector3d& v2){
    afVector3d vOut;
    vOut(0) = v1(0) + v2(0);
    vOut(1) = v1(1) + v2(1);
    vOut(2) = v1(2) + v2(2);
    return vOut;
}

inline afVector3d operator -(const afVector3d& v1, const afVector3d& v2){
    afVector3d vOut;
    vOut(0) = v1(0) - v2(0);
    vOut(1) = v1(1) - v2(1);
    vOut(2) = v1(2) - v2(2);
    return vOut;
}

inline afVector3d operator -(const afVector3d& v1){
    afVector3d vOut;
    vOut(0) = -v1(0);
    vOut(1) = -v1(1);
    vOut(2) = -v1(2);
    return vOut;
}

inline afVector3d operator *(const afVector3d& v1, const afVector3d& v2){
    afVector3d vOut;
    vOut(0) = v1(0) * v2(0);
    vOut(1) = v1(1) * v2(1);
    vOut(2) = v1(2) * v2(2);
    return vOut;
}

inline afVector3d operator /(const afVector3d& v1, const afVector3d& v2){
    afVector3d vOut;
    vOut(0) = v1(0) / v2(0);
    vOut(1) = v1(1) / v2(1);
    vOut(2) = v1(2) / v2(2);
    return vOut;
}

inline afVector3d operator *(const afVector3d& v1, const double a_scale){
    afVector3d vOut;
    vOut(0) = v1(0) * a_scale;
    vOut(1) = v1(1) * a_scale;
    vOut(2) = v1(2) * a_scale;
    return vOut;
}

inline afVector3d operator /(const afVector3d& v1, const double a_scale){
    afVector3d vOut;
    vOut(0) = v1(0) / a_scale;
    vOut(1) = v1(1) / a_scale;
    vOut(2) = v1(2) / a_scale;
    return vOut;
}


class afMatrix3d{
public:
    afMatrix3d(){
        setIdentity();
    }

    afMatrix3d(double roll, double pitch, double yaw){
        setRPY(roll, pitch, yaw);
    }

    void setRPY(double roll, double pitch, double yaw){
        double cR = cos(roll);
        double cP = cos(pitch);
        double cY = cos(yaw);

        double sR = sin(roll);
        double sP = sin(pitch);
        double sY = sin(yaw);

        m_data[0][0] = cY * cP ; m_data[0][1] = cY * sP * sR - sY * cR; m_data[0][2] = cY * sP * cR + sY * sR;
        m_data[1][0] = sY * cP ; m_data[1][1] = sY * sP * sR + cY * cR; m_data[1][2] = sY * sP * cR - cY * sR;
        m_data[2][0] = -sP     ; m_data[2][1] = cP * sR               ; m_data[2][2] = cP * cR;
    }

    void getRPY(double& roll, double& pitch, double& yaw){
        yaw = atan2(m_data[1][0], m_data[0][0]);
        pitch = atan2(-m_data[2][0], sqrt( m_data[2][1] * m_data[2][1] + m_data[2][2] * m_data[2][2] ));
        roll = atan2(m_data[2][1], m_data[2][2]);
    }

    void getRPY(double& roll, double& pitch, double& yaw) const{
        yaw = atan2(m_data[1][0], m_data[0][0]);
        pitch = atan2(-m_data[2][0], sqrt( m_data[2][1] * m_data[2][1] + m_data[2][2] * m_data[2][2] ));
        roll = atan2(m_data[2][1], m_data[2][2]);
    }

    void setIdentity(){
        for (int r = 0 ; r < 3 ; r++){
            for (int c = 0 ; c < 3 ; c++){
                m_data[r][c] = 0.0;
            }
        }
        m_data[0][0] = 1.0;
        m_data[1][1] = 1.0;
        m_data[2][2] = 1.0;
    }

    afVector3d operator *(afVector3d vIn){
        afVector3d vOut;

        vOut(0) = m_data[0][0] * vIn(0) + m_data[0][1] * vIn(1) + m_data[0][2] * vIn(2);
        vOut(1) = m_data[1][0] * vIn(0) + m_data[1][1] * vIn(1) + m_data[1][2] * vIn(2);
        vOut(2) = m_data[2][0] * vIn(0) + m_data[2][1] * vIn(1) + m_data[2][2] * vIn(2);

        return vOut;
    }

    afMatrix3d operator *(afMatrix3d rIn){
        afMatrix3d rOut;
        rOut(0, 0) = m_data[0][0] * rIn(0, 0) + m_data[0][1] * rIn(1, 0) + m_data[0][2] * rIn(2, 0);
        rOut(1, 0) = m_data[1][0] * rIn(0, 0) + m_data[1][1] * rIn(1, 0) + m_data[1][2] * rIn(2, 0);
        rOut(2, 0) = m_data[2][0] * rIn(0, 0) + m_data[2][1] * rIn(1, 0) + m_data[2][2] * rIn(2, 0);

        rOut(0, 1) = m_data[0][0] * rIn(0, 1) + m_data[0][1] * rIn(1, 1) + m_data[0][2] * rIn(2, 1);
        rOut(1, 1) = m_data[1][0] * rIn(0, 1) + m_data[1][1] * rIn(1, 1) + m_data[1][2] * rIn(2, 1);
        rOut(2, 1) = m_data[2][0] * rIn(0, 1) + m_data[2][1] * rIn(1, 1) + m_data[2][2] * rIn(2, 1);

        rOut(0, 2) = m_data[0][0] * rIn(0, 2) + m_data[0][1] * rIn(1, 2) + m_data[0][2] * rIn(2, 2);
        rOut(1, 2) = m_data[1][0] * rIn(0, 2) + m_data[1][1] * rIn(1, 2) + m_data[1][2] * rIn(2, 2);
        rOut(2, 2) = m_data[2][0] * rIn(0, 2) + m_data[2][1] * rIn(1, 2) + m_data[2][2] * rIn(2, 2);

        return rOut;
    }

    void operator *=(afMatrix3d rIn){
        double d[3][3];
        d[0][0] = m_data[0][0] * rIn(0, 0) + m_data[0][1] * rIn(1, 0) + m_data[0][2] * rIn(2, 0);
        d[1][0] = m_data[1][0] * rIn(0, 0) + m_data[1][1] * rIn(1, 0) + m_data[1][2] * rIn(2, 0);
        d[2][0] = m_data[2][0] * rIn(0, 0) + m_data[2][1] * rIn(1, 0) + m_data[2][2] * rIn(2, 0);

        d[0][1] = m_data[0][0] * rIn(0, 1) + m_data[0][1] * rIn(1, 1) + m_data[0][2] * rIn(2, 1);
        d[1][1] = m_data[1][0] * rIn(0, 1) + m_data[1][1] * rIn(1, 1) + m_data[1][2] * rIn(2, 1);
        d[2][1] = m_data[2][0] * rIn(0, 1) + m_data[2][1] * rIn(1, 1) + m_data[2][2] * rIn(2, 1);

        d[0][2] = m_data[0][0] * rIn(0, 2) + m_data[0][1] * rIn(1, 2) + m_data[0][2] * rIn(2, 2);
        d[1][2] = m_data[1][0] * rIn(0, 2) + m_data[1][1] * rIn(1, 2) + m_data[1][2] * rIn(2, 2);
        d[2][2] = m_data[2][0] * rIn(0, 2) + m_data[2][1] * rIn(1, 2) + m_data[2][2] * rIn(2, 2);

        for (int r = 0 ; r < 3 ; r++){
            for (int c = 0 ; c < 3 ; c++){
                m_data[r][c] = d[r][c];
            }
        }
    }

    void transpose(){
        double d;

        d = m_data[0][1]; m_data[0][1] = m_data[1][0]; m_data[1][0] = d;
        d = m_data[0][2]; m_data[0][2] = m_data[2][0]; m_data[2][0] = d;
        d = m_data[1][2]; m_data[1][2] = m_data[2][1]; m_data[2][1] = d;
    }

    afMatrix3d getTranspose(){
        double d;
        afMatrix3d rot = (*this);
        rot.transpose();
        return rot;
    }

    double& operator () (const int m_row, const int m_col){
        assert(m_row < 0 || m_row < 3);
        assert(m_col < 0 || m_col < 3);
        return m_data[m_row][m_col];
    }

    double operator () (const int m_row, const int m_col) const{
        assert(m_row < 0 || m_row < 3);
        assert(m_col < 0 || m_col < 3);
        return m_data[m_row][m_col];
    }

    void print(){
        for (int i = 0 ; i < 3 ; i ++){
            cerr << "[ " << m_data[i][0] << ", " << m_data[i][1] << ", " << m_data[i][2] << "]\n" ;
        }
    }

private:
    double m_data[3][3];
};


class afTransform{
public:
    afTransform(){
        m_P.set(0, 0, 0);
        m_R.setIdentity();
    }

    afTransform(afMatrix3d rot, afVector3d pos){
        m_P = pos;
        m_R = rot;
    }

    afTransform operator *(afTransform tIn){
        afMatrix3d rot = m_R * tIn.getRotation();
        afVector3d pos = m_R * tIn.getPosition() + m_P;
        afTransform tOut(rot, pos);
        return tOut;
    }

    afVector3d operator *(afVector3d vIn){
        afVector3d vOut;
        vOut = m_R * vIn + m_P;
        return vOut;
    }

    afVector3d getPosition(){
        return m_P;
    }

    afVector3d getPosition() const{
        return m_P;
    }

    afMatrix3d getRotation(){
        return m_R;
    }

    afMatrix3d getRotation() const{
        return m_R;
    }

    void setPosition(const afVector3d& v){
        m_P = v;
    }

    void setPosition(afVector3d& v){
        m_P = v;
    }

    void setRotation(const afMatrix3d& m){
        m_R = m;
    }


    void setRotation(afMatrix3d& m){
        m_R = m;
    }

    void setIdentity(){
        m_R.setIdentity();
        m_P.set(0, 0, 0);
    }

    void invert(){
        afTransform invT = getInverse();
        m_P = invT.getPosition();
        m_R = invT.getRotation();
    }

    afTransform getInverse(){
        afMatrix3d invR = m_R.getTranspose();
        afVector3d invP = invR * (-m_P);
        return afTransform(invR, invP);
    }

    void print(){
        for (int i = 0 ; i < 3 ; i ++){
            cerr << "[ " << m_R(i, 0) << ", " << m_R(i, 1) << ", " << m_R(i, 2) << ", " << m_P(i) << "]\n" ;
        }
        cerr << "[ " << 0.0 << ", " << 0.0 << ", " << 0.0 << ", " << 1.0 << "]\n" ;
    }

private:
    afVector3d m_P;
    afMatrix3d m_R;
};


}

#endif
