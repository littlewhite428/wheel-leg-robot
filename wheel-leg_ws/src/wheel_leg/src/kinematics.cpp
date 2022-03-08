#include "kinematics.h"
#include <cmath>
#include <iostream>
namespace Kinematics
{
    Eigen::Vector3d ik(const Eigen::Vector3d& transform, double L1, double L2){
        double x = transform(0);
        double y = transform(1);
        double z = transform(2);
        double D = sqrt(x*x+y*y+z*z);
        if(D>L1+L2-1e-5){
            x *= (L1+L2-1e-5)/D;
            y *= (L1+L2-1e-5)/D;
            z *= (L1+L2-1e-5)/D;
            D = sqrt(x*x+y*y+z*z);
        }
        double q3 = -atan(y/z);
        
        double q1 = 3.1415926 - acos((L1*L1+L2*L2-D*D)/(2*L1*L2)); // 0 < q2 < PI
        
        double x_ = sqrt(y*y+z*z);
        double y_ = x;
        double q2 = atan(y_/x_) - acos((L1*L1+D*D-L2*L2)/(2*L1*D));

        return Eigen::Vector3d(q3,q2,q1);
    }

    Eigen::Vector3d k(const Eigen::Vector3d& joint_angles, double L1, double L2){
        return Eigen::Vector3d();
    }
} // namespace Kinematics

