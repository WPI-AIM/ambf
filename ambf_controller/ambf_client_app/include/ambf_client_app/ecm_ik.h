#ifndef ECM_IK_H
#define ECM_IK_H

#include<ambf_client_app/ecm_fk.h>
#include<ambf_client_app/utilities.h>

class ECM_IK
{
public:
    ECM_IK();

    std::vector<float> compute_IK(Matrix4f T_4_0);
    ~ECM_IK(void);

private:
    const float L_rcc_ = 0.3822;
    const float L_scopelen_ = 0.385495;
};

#endif // ECM_IK_H
