/*! \file
 *  \brief Contains a class for an ES individual.
 */

#ifndef ES_RAYES_INDIVIDUAL_H
#define ES_RAYES_INDIVIDUAL_H

#include <Eigen/Dense>

#include <fstream>

namespace es {
namespace rayes {

/*! \brief An individual for the evolution strategy.
 */
class Individual {
 public:
    Individual();

    double f() const;
    void f(double fVal);

    Eigen::VectorXd ray() const;
    void ray(const Eigen::VectorXd &rayVal);

    Eigen::VectorXd rayNormalized() const;

    Eigen::VectorXd bestOnRay() const;
    void bestOnRay(const Eigen::VectorXd &bestOnRayVal);

    int bestOnRayDirection() const;
    void bestOnRayDirection(const int bestOnRayDirection);

    double sigma() const;
    void sigma(double sigmaVal);

    Eigen::VectorXd rayOrigin() const;
    void rayOrigin(const Eigen::VectorXd &rayOriginVal);

    double sigmaRayOrigin() const;
    void sigmaRayOrigin(double sigmaRayOrigin);

 private:
    double m_f;
    Eigen::VectorXd m_ray;
    Eigen::VectorXd m_bestOnRay;
    int m_bestOnRayDirection;
    double m_sigma;
    Eigen::VectorXd m_rayOrigin;
    double m_sigmaRayOrigin;
};

std::ostream &operator<<(std::ostream &os,
                         const Individual &individual);

}
}

#endif
