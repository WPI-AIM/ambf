#include <Eigen/Core>

class PDController
{
public:
//    template <typename Derived>
    PDController(const EigenBase<Derived>& Kp, const EigenBase<Derived>& Kd);

    ~PDController(void);

private:


};
