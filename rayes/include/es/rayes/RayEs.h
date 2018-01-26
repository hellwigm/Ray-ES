/*! \file
 *  \brief Classes for an evolution strategy that evolves a ray.
 */

#ifndef ES_RAYES_RAYES_H
#define ES_RAYES_RAYES_H

#include "es/rayes/Info.h"

#include <functional>
#include <set>

#include <Eigen/Dense>

namespace es {
namespace rayes {

    enum class LineSearchAlg {
        Standard,
        Modified
    };

    namespace {
        struct LineSearchResult {
            LineSearchResult()
                : bestOnRay()
                , f(0.0)
                , feasibleFound(false)
                , numFitnessEvaluations(0)
                , bestOnRayDirection(0) {
            }

            Eigen::VectorXd bestOnRay;
            double f;
            bool feasibleFound;
            int numFitnessEvaluations;
            int bestOnRayDirection;
        };
    }

/*! \brief Evolution strategy that evolves a ray.
 */
class RayEs {
 public:
    explicit RayEs(
          const std::function<double(const Eigen::VectorXd &)> &objectiveFun,
          const std::function<Eigen::VectorXd(
                              const Eigen::VectorXd &)> &constraintFun,
          const Eigen::VectorXd &lbnds,
          const Eigen::VectorXd &ubnds,
          const Eigen::VectorXd &rayOriginInit,
          const LineSearchAlg lineSearchAlg);

    Info run();

 private:
    LineSearchResult lineSearch(const Eigen::VectorXd &rayNormalized,
                                const double lineSearchLineLength,
                                const int lineSearchPartitions,
                                const Eigen::VectorXd &rayOrigin,
                                const double epsilon);
    LineSearchResult lineSearch2(const Eigen::VectorXd &rayNormalized,
                                 const Eigen::VectorXd &rayOrigin,
                                 const double stepSizeInit,
                                 const double stepSizeIncreaseFactor,
                                 const double stepSizeDecreaseFactor,
                                 const double epsilon,
                                 const std::set<int> &directions);

    bool isFeasible(const Eigen::VectorXd &x);

    std::function<double(const Eigen::VectorXd &)> m_objectiveFun;
    std::function<Eigen::VectorXd(const Eigen::VectorXd &)> m_constraintFun;
    Eigen::VectorXd m_lbnds;
    Eigen::VectorXd m_ubnds;
    Eigen::VectorXd m_rayOriginInit;
    LineSearchAlg m_lineSearchAlg;
};

}
}

#endif
