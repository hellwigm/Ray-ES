#include "es/rayes/Individual.h"

namespace es {
namespace rayes {

Individual::Individual()
    : m_f(0.0)
    , m_ray()
    , m_bestOnRay()
    , m_sigma(0.0)
    , m_rayOrigin()
    , m_sigmaRayOrigin(0.0)
{
}

double Individual::f() const {
    return m_f;
}

void Individual::f(double fVal) {
    m_f = fVal;
}

Eigen::VectorXd Individual::ray() const {
    return m_ray;
}

void Individual::ray(const Eigen::VectorXd &rayVal) {
    m_ray = rayVal;
}

Eigen::VectorXd Individual::rayNormalized() const {
    double norm = m_ray.norm();
    return m_ray / (std::abs(norm) > 1e-6 ? norm : 1.0);
}

Eigen::VectorXd Individual::bestOnRay() const {
    return m_bestOnRay;
}

void Individual::bestOnRay(const Eigen::VectorXd &bestOnRayVal) {
    m_bestOnRay = bestOnRayVal;
}

int Individual::bestOnRayDirection() const {
    return m_bestOnRayDirection;
}

void Individual::bestOnRayDirection(const int bestOnRayDirectionVal) {
    m_bestOnRayDirection = bestOnRayDirectionVal;
}

double Individual::sigma() const {
    return m_sigma;
}

void Individual::sigma(double sigmaVal) {
    m_sigma = sigmaVal;
}

Eigen::VectorXd Individual::rayOrigin() const {
    return m_rayOrigin;
}

void Individual::rayOrigin(const Eigen::VectorXd &rayOriginVal) {
    m_rayOrigin = rayOriginVal;
}

double Individual::sigmaRayOrigin() const {
    return m_sigmaRayOrigin;
}

void Individual::sigmaRayOrigin(double sigmaRayOriginVal) {
    m_sigmaRayOrigin = sigmaRayOriginVal;
}

std::ostream &operator<<(std::ostream &os,
                         const Individual &individual) {
    os << "["
       << "f = " << individual.f()
       << ", sigma = " << individual.sigma()
       << "]";
    return os;
}

}
}
