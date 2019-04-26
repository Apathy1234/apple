#ifndef _MATH_UTILS_HPP_
#define _MATH_UTILS_HPP_


#include <pose_estimate/my_include.h>

namespace slam_mono
{
  inline Eigen::Quaterniond QuaternionFromSmallAngle(const Eigen::Vector3d& dtheta)
  {
    double dq_square = dtheta.squaredNorm() / 4.0;

    if (dq_square <= 1)
    {
      return Eigen::Quaterniond(sqrt(1-dq_square), dtheta(0) * 0.5, dtheta(1) * 0.5, dtheta(2) * 0.5);
    }
    else
    {
      const double w = 1.0 / sqrt(1 + dq_square);
      const double f = w * 0.5;
      return Eigen::Quaterniond(w, dtheta(0) * f, dtheta(1) * f, dtheta(2) * f);
    }
  }
/*
 *  @brief Create a skew-symmetric matrix from a 3-element vector.
 *  @note Performs the operation:
 *  w   ->  [  0 -w3  w2]
 *          [ w3   0 -w1]
 *          [-w2  w1   0]
 */
    inline Eigen::Matrix3d skewSymmetric(const Eigen::Vector3d& w)
    {
        Eigen::Matrix3d w_sym;

        w_sym(0, 0) =     0;
        w_sym(0, 1) = -w(2);
        w_sym(0, 2) =  w(1);
        w_sym(1, 0) =  w(2);
        w_sym(1, 1) =     0;
        w_sym(1, 2) = -w(0);
        w_sym(2, 0) = -w(1);
        w_sym(2, 1) =  w(0);
        w_sym(2, 2) =     0;
        return w_sym;
    }

/*
 * @brief Normalize the given quaternion to unit quaternion.
 */
    inline void quaternionNormalize(Eigen::Vector4d& q)
    {
        double norm = q.norm();
        q = q / norm;
    }


/*
 * @brief Convert a rotation matrix to a quaternion.
 * @note Pay attention to the convention used. The function follows the
 *    conversion in "Indirect Kalman Filter for 3D Attitude Estimation:
 *    A Tutorial for Quaternion Algebra", Equation (78).
 *
 *    The input quaternion should be in the form
 *      [q1, q2, q3, q4(scalar)]^T
 */
  inline Eigen::Quaterniond rotationToQuaternion(const Eigen::Matrix3d& R) 
  {
    Eigen::Vector4d score;
    score(0) = R(0, 0);
    score(1) = R(1, 1);
    score(2) = R(2, 2);
    score(3) = R.trace();

    int max_row = 0, max_col = 0;
    score.maxCoeff(&max_row, &max_col);

    Eigen::Vector4d q = Eigen::Vector4d::Zero();
    if (max_row == 0) {
      q(0) = std::sqrt(1+2*R(0, 0)-R.trace()) / 2.0;
      q(1) = (R(0, 1)+R(1, 0)) / (4*q(0));
      q(2) = (R(0, 2)+R(2, 0)) / (4*q(0));
      q(3) = (R(1, 2)-R(2, 1)) / (4*q(0));
    } else if (max_row == 1) {
      q(1) = std::sqrt(1+2*R(1, 1)-R.trace()) / 2.0;
      q(0) = (R(0, 1)+R(1, 0)) / (4*q(1));
      q(2) = (R(1, 2)+R(2, 1)) / (4*q(1));
      q(3) = (R(2, 0)-R(0, 2)) / (4*q(1));
    } else if (max_row == 2) {
      q(2) = std::sqrt(1+2*R(2, 2)-R.trace()) / 2.0;
      q(0) = (R(0, 2)+R(2, 0)) / (4*q(2));
      q(1) = (R(1, 2)+R(2, 1)) / (4*q(2));
      q(3) = (R(0, 1)-R(1, 0)) / (4*q(2));
    } else {
      q(3) = std::sqrt(1+R.trace()) / 2.0;
      q(0) = (R(1, 2)-R(2, 1)) / (4*q(3));
      q(1) = (R(2, 0)-R(0, 2)) / (4*q(3));
      q(2) = (R(0, 1)-R(1, 0)) / (4*q(3));
    }

    if (q(3) < 0) q = -q;
    quaternionNormalize(q);

    return Eigen::Quaterniond(q(3), q(0), q(1), q(2));
  }
  
  /*namespace end*/
}



#endif